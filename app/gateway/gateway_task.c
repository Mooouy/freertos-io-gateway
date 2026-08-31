/*
 * 网关编排层实现(CubeMX mbMasterTask 入口, 强符号覆盖 freertos.c 的 __weak 默认)。
 *
 * 职责:
 *  - 周期分组读取 BSM 全量 I/O 快照, 经 modbus_service 串行访问(同步提交, 天然无重叠请求);
 *  - 用 gateway_model 维护快照/物理量/时序/链路状态/诊断计数;
 *  - 用 gateway_rules 执行 AI 高报警滞回、继电器互锁、点动;
 *  - 消费显式控制命令, 命令优先于轮询但每周期处理数量受限(防轮询饥饿);
 *  - 首次成功快照前(model.valid=false)拒绝一切写输出命令, 重启后先读实际输出、不主动写默认值(规格 §8.3);
 *  - 继电器写入一律写后回读确认, 实际位匹配才算成功;
 *  - 仅在状态变化/关键动作时打印紧凑一行, 不每个 loop 刷屏。
 * 不直接访问 USART3/RS485, 不含 MQTT/ESP8266 逻辑(属 v0.5.0)。
 */

#include "gateway_task.h"

#include "gateway_config.h"
#include "gateway_model.h"
#include "gateway_rules.h"
#include "bsm0404.h"
#include "modbus_service.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* 进行中的点动状态(每继电器通道)。 */
typedef struct
{
    bool active;
    uint32_t deadline_ms;
} GatewayPulse_t;

/* 显式命令队列(静态分配)。 */
static QueueHandle_t cmd_queue;
static StaticQueue_t cmd_queue_control;
static uint8_t cmd_queue_storage[GATEWAY_CMD_QUEUE_LEN * sizeof(GatewayCommand_t)];

/* 命令执行结果队列(静态分配): 带 request_id 的命令执行后回传结果给 MQTT 任务。 */
static QueueHandle_t result_queue;
static StaticQueue_t result_queue_control;
static uint8_t result_queue_storage[GATEWAY_CMD_QUEUE_LEN * sizeof(GatewayCommandResult_t)];

/* 供遥测读取的一致模型快照(每轮末在临界区发布, 避免跨任务撕裂读)。 */
static GatewayModel_t shared_model;

/* 网关运行态。 */
static GatewayModel_t model;
static bool ai_alarm[BSM_AI_COUNT];
static GatewayPulse_t pulse[BSM_CHANNEL_COUNT];
static GatewayLinkState_t prev_link_state;

/* 继电器互锁配置(由 gateway_config 宏构造)。 */
static const GatewayInterlock_t interlock = {
    GATEWAY_INTERLOCK_CH_A,
    GATEWAY_INTERLOCK_CH_B,
    (GATEWAY_INTERLOCK_ENABLED != 0),
};

static const char *gateway_link_str(GatewayLinkState_t s)
{
    switch (s)
    {
        case GATEWAY_LINK_INIT:     return "INIT";
        case GATEWAY_LINK_ONLINE:   return "ONLINE";
        case GATEWAY_LINK_DEGRADED: return "DEGRADED";
        case GATEWAY_LINK_OFFLINE:  return "OFFLINE";
        default:                    return "INVALID";
    }
}

/* 03 读 count 个保持寄存器到 out; 经 modbus_service 串行执行。 */
static ModbusStatus_t gateway_read(uint16_t addr, uint16_t count, uint16_t *out)
{
    ModbusRequest_t req = {0};
    ModbusResult_t res = {0};
    ModbusStatus_t st;

    req.slave = GATEWAY_BSM_SLAVE_ADDR;
    req.operation = MODBUS_OP_READ_HOLDING;
    req.address = addr;
    req.count = count;
    req.timeout_ms = GATEWAY_REQUEST_TIMEOUT_MS;

    st = Modbus_Service_Submit(&req, &res);
    if (st == MODBUS_STATUS_OK)
    {
        for (uint16_t i = 0; i < count; i++)
        {
            out[i] = res.registers[i];
        }
    }
    return st;
}

/* 06 写单个保持寄存器。 */
static ModbusStatus_t gateway_write(uint16_t addr, uint16_t value)
{
    ModbusRequest_t req = {0};
    ModbusResult_t res = {0};

    req.slave = GATEWAY_BSM_SLAVE_ADDR;
    req.operation = MODBUS_OP_WRITE_SINGLE;
    req.address = addr;
    req.count = 1U;
    req.values[0] = value;
    req.timeout_ms = GATEWAY_REQUEST_TIMEOUT_MS;

    return Modbus_Service_Submit(&req, &res);
}

/* 分组读取全量 I/O 快照(计数器/AO/AI/继电器/DI); 任一块失败即返回其状态。 */
static ModbusStatus_t gateway_read_snapshot(BsmSnapshot_t *snap)
{
    uint16_t counters[BSM_COUNTER_REG_COUNT];
    uint16_t ao[BSM_AO_COUNT];
    uint16_t ai[BSM_AI_COUNT];
    uint16_t relay = 0U;
    uint16_t di = 0U;
    ModbusStatus_t st;

    st = gateway_read(BSM_REG_DI_COUNTER_BASE, BSM_COUNTER_REG_COUNT, counters);
    if (st != MODBUS_STATUS_OK) { return st; }
    st = gateway_read(BSM_REG_AO_BASE, BSM_AO_COUNT, ao);
    if (st != MODBUS_STATUS_OK) { return st; }
    st = gateway_read(BSM_REG_AI_BASE, BSM_AI_COUNT, ai);
    if (st != MODBUS_STATUS_OK) { return st; }
    st = gateway_read(BSM_REG_RELAY_BITMAP, 1U, &relay);
    if (st != MODBUS_STATUS_OK) { return st; }
    st = gateway_read(BSM_REG_DI_BITMAP, 1U, &di);
    if (st != MODBUS_STATUS_OK) { return st; }

    Bsm_DecodeSnapshot(counters, ao, ai, relay, di, snap);
    return MODBUS_STATUS_OK;
}

/* 写继电器位图并回读确认: 实际低 4 位与期望一致才返回 true。 */
static bool gateway_write_relay(uint8_t desired)
{
    uint16_t readback = 0U;

    if (gateway_write(BSM_REG_RELAY_BITMAP, desired) != MODBUS_STATUS_OK)
    {
        return false;
    }
    if (gateway_read(BSM_REG_RELAY_BITMAP, 1U, &readback) != MODBUS_STATUS_OK)
    {
        return false;
    }
    return (uint8_t)(readback & BSM_CHANNEL_MASK) == (uint8_t)(desired & BSM_CHANNEL_MASK);
}

/* 链路状态变化时打印一行(含诊断计数); 无变化不打印。 */
static void gateway_log_link_change(void)
{
    if (model.link_state == prev_link_state)
    {
        return;
    }
    printf("GW link %s->%s seq=%lu ok=%lu to=%lu crc=%lu exc=%lu oth=%lu\r\n",
           gateway_link_str(prev_link_state), gateway_link_str(model.link_state),
           (unsigned long)model.sequence, (unsigned long)model.diag.success,
           (unsigned long)model.diag.timeout, (unsigned long)model.diag.crc_error,
           (unsigned long)model.diag.exception, (unsigned long)model.diag.other_error);
    prev_link_state = model.link_state;
}

/* 对每路 AI 求高报警滞回; 仅在报警状态翻转时打印一行。 */
static void gateway_eval_alarms(void)
{
    static const uint16_t high_on[BSM_AI_COUNT] = {GATEWAY_AI1_HIGH_RAW, GATEWAY_AI2_HIGH_RAW};
    static const uint16_t deadband[BSM_AI_COUNT] = {GATEWAY_AI1_DEADBAND_RAW,
                                                    GATEWAY_AI2_DEADBAND_RAW};
    uint8_t i;

    for (i = 0; i < BSM_AI_COUNT; i++)
    {
        bool next = ai_alarm[i];
        uint16_t off = (uint16_t)(high_on[i] - deadband[i]);
        GatewayRuleStatus_t rs =
            GatewayRules_HighAlarm(ai_alarm[i], model.snapshot.ai_raw[i], high_on[i], off, &next);
        if (rs == GATEWAY_RULE_OK && next != ai_alarm[i])
        {
            ai_alarm[i] = next;
            printf("GW AI%u alarm %s raw=%u\r\n", (unsigned int)(i + 1U),
                   next ? "SET" : "CLEAR", (unsigned int)model.snapshot.ai_raw[i]);
        }
    }
}

/* 一次周期轮询: 读快照 -> 更新模型 -> 求报警/链路日志。 */
static void gateway_poll(uint32_t now_ms)
{
    BsmSnapshot_t snap = {0};
    ModbusStatus_t st = gateway_read_snapshot(&snap);

    if (st == MODBUS_STATUS_OK)
    {
        GatewayModel_ApplySuccess(&model, &snap, now_ms);
        gateway_eval_alarms();
    }
    else
    {
        GatewayModel_ApplyFailure(&model, st, now_ms, GATEWAY_OFFLINE_FAILURE_THRESHOLD);
    }
    gateway_log_link_change();
}

/* RELAY_SET: 经互锁校验后写继电器并回读确认; 显式置位取消该通道点动。 */
static GatewayCmdResult_t gateway_exec_relay_set(uint8_t ch, bool on, int32_t *actual)
{
    uint8_t next = 0U;
    GatewayRuleStatus_t rs =
        GatewayRules_RelaySet(model.snapshot.relay_bits, ch, on, &interlock, &next);

    if (rs != GATEWAY_RULE_OK)
    {
        printf("GW relay ch%u %s rejected rs=%d\r\n", (unsigned int)(ch + 1U),
               on ? "ON" : "OFF", (int)rs);
        return GATEWAY_RESULT_REJECTED;
    }
    if (gateway_write_relay(next))
    {
        model.snapshot.relay_bits = next;
        if (ch < BSM_CHANNEL_COUNT)
        {
            pulse[ch].active = false;
        }
        printf("GW relay ch%u %s ok 0x%X\r\n", (unsigned int)(ch + 1U), on ? "ON" : "OFF",
               (unsigned int)next);
        *actual = (int32_t)((next >> ch) & 1U);
        return GATEWAY_RESULT_OK;
    }
    printf("GW relay ch%u write/readback failed\r\n", (unsigned int)(ch + 1U));
    return GATEWAY_RESULT_FAILED;
}

/* RELAY_PULSE: 经互锁校验后立即 ON 并记录到期 tick(到期由 gateway_service_pulses 关断)。 */
static GatewayCmdResult_t gateway_exec_pulse(uint8_t ch, uint32_t duration_ms, uint32_t now_ms,
                                             int32_t *actual)
{
    uint8_t next = 0U;
    GatewayRuleStatus_t rs;

    if (ch >= BSM_CHANNEL_COUNT)
    {
        printf("GW pulse ch%u invalid\r\n", (unsigned int)(ch + 1U));
        return GATEWAY_RESULT_REJECTED;
    }
    rs = GatewayRules_RelaySet(model.snapshot.relay_bits, ch, true, &interlock, &next);
    if (rs != GATEWAY_RULE_OK)
    {
        printf("GW pulse ch%u rejected rs=%d\r\n", (unsigned int)(ch + 1U), (int)rs);
        return GATEWAY_RESULT_REJECTED;
    }
    if (gateway_write_relay(next))
    {
        model.snapshot.relay_bits = next;
        pulse[ch].active = true;
        pulse[ch].deadline_ms = GatewayRules_PulseDeadline(now_ms, duration_ms);
        printf("GW pulse ch%u ON %lums\r\n", (unsigned int)(ch + 1U), (unsigned long)duration_ms);
        *actual = (int32_t)((next >> ch) & 1U);
        return GATEWAY_RESULT_OK;
    }
    printf("GW pulse ch%u write failed\r\n", (unsigned int)(ch + 1U));
    return GATEWAY_RESULT_FAILED;
}

/* AO_SET: 原始值越界拒绝; 否则写 DA 寄存器。 */
static GatewayCmdResult_t gateway_exec_ao_set(uint8_t ch, uint16_t raw, int32_t *actual)
{
    if ((ch >= BSM_AO_COUNT) || (raw > BSM_ANALOG_RAW_MAX))
    {
        printf("GW AO ch%u invalid raw=%u\r\n", (unsigned int)(ch + 1U), (unsigned int)raw);
        return GATEWAY_RESULT_REJECTED;
    }
    if (gateway_write((uint16_t)(BSM_REG_AO_BASE + ch), raw) == MODBUS_STATUS_OK)
    {
        printf("GW AO%u=%u ok\r\n", (unsigned int)(ch + 1U), (unsigned int)raw);
        *actual = (int32_t)raw;
        return GATEWAY_RESULT_OK;
    }
    printf("GW AO%u write failed\r\n", (unsigned int)(ch + 1U));
    return GATEWAY_RESULT_FAILED;
}

/* AI_MODE_SET: 改 RAM 中该 AI 通道解释模式; 下次成功采样起按新模式换算物理量(规格 §2.2)。 */
static GatewayCmdResult_t gateway_exec_ai_mode_set(uint8_t ch, uint16_t mode, int32_t *actual)
{
    if ((ch >= BSM_AI_COUNT) || ((mode != (uint16_t)GATEWAY_AI_MODE_VOLTAGE) &&
                                 (mode != (uint16_t)GATEWAY_AI_MODE_CURRENT)))
    {
        printf("GW AI%u mode invalid m=%u\r\n", (unsigned int)(ch + 1U), (unsigned int)mode);
        return GATEWAY_RESULT_REJECTED;
    }
    model.ai_mode[ch] = (GatewayAiMode_t)mode;
    *actual = (int32_t)mode;
    printf("GW AI%u mode=%s\r\n", (unsigned int)(ch + 1U),
           (mode == (uint16_t)GATEWAY_AI_MODE_VOLTAGE) ? "voltage" : "current");
    return GATEWAY_RESULT_OK;
}

static void gateway_execute(const GatewayCommand_t *cmd, uint32_t now_ms)
{
    GatewayCmdResult_t result;
    int32_t actual = 0;

    /* 首次成功快照前不写输出: 重启后须先读模块实际输出, 拒绝命令且不发任何 Modbus 写。 */
    if (!model.valid)
    {
        printf("GW cmd skipped before first snapshot\r\n");
        result = GATEWAY_RESULT_NOT_READY;
    }
    else
    {
        switch (cmd->type)
        {
            case GATEWAY_CMD_RELAY_SET:
                result = gateway_exec_relay_set(cmd->channel, cmd->value != 0U, &actual);
                break;
            case GATEWAY_CMD_RELAY_PULSE:
                result = gateway_exec_pulse(cmd->channel, cmd->duration_ms, now_ms, &actual);
                break;
            case GATEWAY_CMD_AO_SET:
                result = gateway_exec_ao_set(cmd->channel, cmd->value, &actual);
                break;
            case GATEWAY_CMD_AI_MODE_SET:
                result = gateway_exec_ai_mode_set(cmd->channel, cmd->value, &actual);
                break;
            default:
                result = GATEWAY_RESULT_INVALID;
                break;
        }
    }

    /* 带 request_id 的命令执行后回传结果, 供 MQTT 任务发布 reply(队列满则丢弃, 不阻塞)。 */
    if ((cmd->request_id != 0U) && (result_queue != NULL))
    {
        GatewayCommandResult_t r = {cmd->request_id, result, actual};
        (void)xQueueSend(result_queue, &r, 0);
    }
}

/*
 * 到期点动关断(关断不触发互锁): 仅在写继电器并回读确认成功后才清除 active 并打印 OFF;
 * 写/回读失败则保持 active=true, 下个循环继续重试关断, 避免误判为已安全关断。
 */
static void gateway_service_pulses(uint32_t now_ms)
{
    uint8_t ch;

    for (ch = 0; ch < BSM_CHANNEL_COUNT; ch++)
    {
        if (pulse[ch].active && !GatewayRules_PulseActive(now_ms, pulse[ch].deadline_ms))
        {
            uint8_t next = 0U;
            (void)GatewayRules_RelaySet(model.snapshot.relay_bits, ch, false, &interlock, &next);
            if (gateway_write_relay(next))
            {
                model.snapshot.relay_bits = next;
                pulse[ch].active = false; /* 确认关断后才结束点动 */
                printf("GW pulse ch%u OFF deadline\r\n", (unsigned int)(ch + 1U));
            }
            else
            {
                /* 关断未确认: 保持 active, 下个循环重试 */
                printf("GW pulse ch%u OFF retry failed\r\n", (unsigned int)(ch + 1U));
            }
        }
    }
}

/*
 * 距最早一个进行中点动到期的毫秒数: 无 active 点动返回 UINT32_MAX, 已到期返回 0。
 * 用有符号差值容忍 uint32_t tick 回绕(与 GatewayRules_PulseActive 一致)。
 */
static uint32_t gateway_next_pulse_wait_ms(uint32_t now_ms)
{
    uint32_t wait = UINT32_MAX;
    uint8_t ch;

    for (ch = 0; ch < BSM_CHANNEL_COUNT; ch++)
    {
        if (pulse[ch].active)
        {
            int32_t diff = (int32_t)(pulse[ch].deadline_ms - now_ms);
            uint32_t until = (diff > 0) ? (uint32_t)diff : 0U;
            if (until < wait)
            {
                wait = until;
            }
        }
    }
    return wait;
}

#if GATEWAY_DEMO_ENABLED
/* v0.4.0 本地演示: 每 GATEWAY_DEMO_PERIOD_MS 经命令队列投放一条命令, 轮转演示互锁/点动/AO。 */
static void gateway_demo(uint32_t now_ms)
{
    static uint32_t next_ms = 0U;
    static uint8_t step = 0U;
    GatewayCommand_t cmd = {0};

    if (!model.valid)
    {
        return; /* 首次成功快照前不投放写命令(执行层亦会拒绝); 首次成功后从 step 0 起转 */
    }
    if ((int32_t)(now_ms - next_ms) < 0)
    {
        return;
    }
    next_ms = now_ms + GATEWAY_DEMO_PERIOD_MS;

    switch (step)
    {
        case 0: cmd.type = GATEWAY_CMD_RELAY_SET; cmd.channel = 0U; cmd.value = 1U; break;
        case 1: cmd.type = GATEWAY_CMD_RELAY_SET; cmd.channel = 1U; cmd.value = 1U; break; /* 互锁拒绝 */
        case 2: cmd.type = GATEWAY_CMD_RELAY_SET; cmd.channel = 0U; cmd.value = 0U; break;
        case 3:
            cmd.type = GATEWAY_CMD_RELAY_PULSE;
            cmd.channel = 0U;
            cmd.duration_ms = GATEWAY_DEMO_PULSE_MS;
            break;
        case 4: cmd.type = GATEWAY_CMD_AO_SET; cmd.channel = 0U; cmd.value = GATEWAY_DEMO_DA1_RAW; break;
        default: cmd.type = GATEWAY_CMD_AO_SET; cmd.channel = 1U; cmd.value = GATEWAY_DEMO_DA2_RAW; break;
    }
    (void)GatewayTask_SubmitCommand(&cmd);
    step = (uint8_t)((step + 1U) % 6U);
}
#endif /* GATEWAY_DEMO_ENABLED */

void GatewayTask_Init(void)
{
    cmd_queue = xQueueCreateStatic(GATEWAY_CMD_QUEUE_LEN, sizeof(GatewayCommand_t),
                                   cmd_queue_storage, &cmd_queue_control);
    result_queue = xQueueCreateStatic(GATEWAY_CMD_QUEUE_LEN, sizeof(GatewayCommandResult_t),
                                      result_queue_storage, &result_queue_control);
    GatewayModel_Init(&model, GATEWAY_AI1_STARTUP_MODE, GATEWAY_AI2_STARTUP_MODE);
    prev_link_state = model.link_state;
    shared_model = model; /* 初始发布, 供遥测在首次成功采样前也能读到一致的初始态 */
}

bool GatewayTask_ReceiveResult(GatewayCommandResult_t *out, uint32_t timeout_ms)
{
    if ((out == NULL) || (result_queue == NULL))
    {
        return false;
    }
    return xQueueReceive(result_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdPASS;
}

void GatewayTask_CopyModel(GatewayModel_t *out)
{
    if (out == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *out = shared_model;
    taskEXIT_CRITICAL();
}

GatewaySubmitResult_t GatewayTask_SubmitCommand(const GatewayCommand_t *command)
{
    if ((command == NULL) || (command->type == GATEWAY_CMD_INVALID) || (cmd_queue == NULL))
    {
        return GATEWAY_SUBMIT_INVALID;
    }
    if (xQueueSend(cmd_queue, command, 0) != pdPASS)
    {
        return GATEWAY_SUBMIT_FULL;
    }
    return GATEWAY_SUBMIT_OK;
}

/*
 * mbMasterTask 入口(强符号, 覆盖 freertos.c 的 __weak StartModbusMasterTask)。
 * 启动先轮询一次以学习实际输出(规格 §8.3, 不主动写默认值), 随后事件驱动:
 * 阻塞等待命令至下次轮询时刻; 命令优先并限量处理, 再服务点动到期与周期轮询。
 */
void StartModbusMasterTask(void *argument)
{
    uint32_t next_poll_ms;
    GatewayCommand_t cmd;

    (void)argument;

    printf("Gateway task started\r\n");
    gateway_poll(osKernelGetTickCount());
    next_poll_ms = osKernelGetTickCount() + GATEWAY_POLL_PERIOD_MS;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        int32_t until_poll = (int32_t)(next_poll_ms - now);
        uint32_t wait = (until_poll > 0) ? (uint32_t)until_poll : 0U;
        uint32_t pulse_wait = gateway_next_pulse_wait_ms(now);

        if (pulse_wait < wait)
        {
            wait = pulse_wait; /* 取较小值: 点动到期不晚于一个 poll 周期被关断 */
        }

        if (xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(wait)) == pdPASS)
        {
            uint8_t n = 0U;
            do
            {
                gateway_execute(&cmd, osKernelGetTickCount());
                n++;
            } while ((n < GATEWAY_MAX_CMDS_PER_CYCLE) &&
                     (xQueueReceive(cmd_queue, &cmd, 0) == pdPASS));
        }

        now = osKernelGetTickCount();
        gateway_service_pulses(now);

        if ((int32_t)(now - next_poll_ms) >= 0)
        {
            gateway_poll(now);
            next_poll_ms = now + GATEWAY_POLL_PERIOD_MS;
        }
#if GATEWAY_DEMO_ENABLED
        gateway_demo(osKernelGetTickCount());
#endif

        /* 发布一致模型快照供遥测读取(整体复制, 与本任务更新互斥, 避免撕裂读)。 */
        taskENTER_CRITICAL();
        shared_model = model;
        taskEXIT_CRITICAL();
    }
}
