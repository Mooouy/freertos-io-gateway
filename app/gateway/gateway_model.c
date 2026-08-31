/*
 * 网关统一 I/O 数据模型实现(纯逻辑, 见 gateway_model.h)。
 *
 * 全部为值拷贝与整数运算: 无动态内存、无 RTOS 句柄、无 I/O 副作用。
 * 物理量换算复用 bsm0404 的量程换算函数。
 */

#include "gateway_model.h"

#include <string.h>

/*
 * 按通道模式把 AI 原始值换算为整数物理量(VOLTAGE:mV / CURRENT:uA)。
 * 模式非法或原始值越界时返回 GATEWAY_AI_VALUE_INVALID, 绝不静默按电压处理。
 */
static int32_t gateway_ai_physical(uint16_t raw, GatewayAiMode_t mode)
{
    uint16_t out = 0U;
    bool ok;

    switch (mode)
    {
        case GATEWAY_AI_MODE_VOLTAGE:
            ok = Bsm_RawToMillivolts(raw, &out);
            break;
        case GATEWAY_AI_MODE_CURRENT:
            ok = Bsm_RawToMicroamps(raw, &out);
            break;
        default:
            ok = false; /* 非法模式: 不换算 */
            break;
    }
    return ok ? (int32_t)out : GATEWAY_AI_VALUE_INVALID;
}

/* AO 物理量固定: DA1(通道 0) 为电压 mV, DA2(通道 1) 为电流 uA(规格 §2.2)。 */
static int32_t gateway_ao_physical(uint16_t raw, uint8_t channel)
{
    uint16_t out = 0U;
    bool ok;

    if (channel == 1U)
    {
        ok = Bsm_RawToMicroamps(raw, &out);
    }
    else
    {
        ok = Bsm_RawToMillivolts(raw, &out);
    }
    return ok ? (int32_t)out : 0;
}

/* 按失败状态累计到对应诊断计数(非超时/CRC/异常者归入 other_error)。 */
static void gateway_count_failure(GatewayDiagCounters_t *diag, ModbusStatus_t status)
{
    switch (status)
    {
        case MODBUS_STATUS_TIMEOUT:
            diag->timeout++;
            break;
        case MODBUS_STATUS_CRC_ERROR:
            diag->crc_error++;
            break;
        case MODBUS_STATUS_EXCEPTION:
            diag->exception++;
            break;
        default:
            diag->other_error++;
            break;
    }
}

/* 校验 AI 模式: 仅 VOLTAGE/CURRENT 合法, 其它一律归一化为 INVALID(不静默降级)。 */
static GatewayAiMode_t gateway_normalize_mode(GatewayAiMode_t mode)
{
    if (mode == GATEWAY_AI_MODE_VOLTAGE || mode == GATEWAY_AI_MODE_CURRENT)
    {
        return mode;
    }
    return GATEWAY_AI_MODE_INVALID;
}

void GatewayModel_Init(GatewayModel_t *model,
                       GatewayAiMode_t ai1_mode,
                       GatewayAiMode_t ai2_mode)
{
    if (model == NULL)
    {
        return;
    }

    memset(model, 0, sizeof(*model)); /* 清零: link 落到 INIT(0), 未设 mode 落到 INVALID(0) */
    model->ai_mode[0] = gateway_normalize_mode(ai1_mode);
    model->ai_mode[1] = gateway_normalize_mode(ai2_mode);
}

void GatewayModel_ApplySuccess(GatewayModel_t *model,
                               const BsmSnapshot_t *snapshot,
                               uint32_t now_ms)
{
    uint8_t i;

    if (model == NULL || snapshot == NULL)
    {
        return;
    }

    model->snapshot = *snapshot;
    for (i = 0U; i < BSM_AI_COUNT; i++)
    {
        model->ai_value[i] = gateway_ai_physical(snapshot->ai_raw[i], model->ai_mode[i]);
    }
    for (i = 0U; i < BSM_AO_COUNT; i++)
    {
        model->ao_value[i] = gateway_ao_physical(snapshot->ao_raw[i], i);
    }

    model->sequence++;
    model->last_success_ms = now_ms;
    model->last_attempt_ms = now_ms;
    model->valid = true;
    model->stale = false;
    model->last_status = MODBUS_STATUS_OK;
    model->link_state = GATEWAY_LINK_ONLINE;
    model->consecutive_failures = 0U;
    model->diag.success++;
}

void GatewayModel_ApplyFailure(GatewayModel_t *model,
                               ModbusStatus_t status,
                               uint32_t now_ms,
                               uint16_t offline_threshold)
{
    if (model == NULL)
    {
        return;
    }

    model->last_attempt_ms = now_ms;
    model->last_status = status;
    model->stale = true;
    gateway_count_failure(&model->diag, status);

    if (model->consecutive_failures < UINT16_MAX)
    {
        model->consecutive_failures++;
    }
    if (offline_threshold != 0U && model->consecutive_failures >= offline_threshold)
    {
        model->link_state = GATEWAY_LINK_OFFLINE;
    }
    else
    {
        model->link_state = GATEWAY_LINK_DEGRADED;
    }
}
