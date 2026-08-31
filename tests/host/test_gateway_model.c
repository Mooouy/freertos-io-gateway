/*
 * 网关统一 I/O 数据模型测试(纯逻辑)。
 *
 * 覆盖: 初始化清零与模式设定、成功采样更新序号/时间/值与物理量换算、
 * 失败采样保留旧值并标记 stale、诊断计数按状态分类、连续失败转 OFFLINE 与恢复,
 * 以及 gateway_config.h 关键配置值。不依赖 HAL/FreeRTOS。
 */

#include "gateway_config.h"
#include "gateway_model.h"
#include "test_assert.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 构造一份带计数器的快照(di_count[0] 固定, 便于校验搬运)。 */
static BsmSnapshot_t make_snapshot(uint8_t di, uint8_t relay,
                                   uint16_t ai0, uint16_t ai1,
                                   uint16_t ao0, uint16_t ao1)
{
    BsmSnapshot_t snap = {0};

    snap.di_bits = di;
    snap.relay_bits = relay;
    snap.di_count[0] = 0x00020001U;
    snap.ai_raw[0] = ai0;
    snap.ai_raw[1] = ai1;
    snap.ao_raw[0] = ao0;
    snap.ao_raw[1] = ao1;
    return snap;
}

/* Init 应清零脏数据并设定 AD1/AD2 模式。 */
static int test_init_clears_and_sets_modes(void)
{
    GatewayModel_t model;
    int fails = 0;

    memset(&model, 0xAB, sizeof(model)); /* 脏数据, 验证 Init 真正清零 */
    GatewayModel_Init(&model, GATEWAY_AI_MODE_CURRENT, GATEWAY_AI_MODE_VOLTAGE);

    fails += TEST_EQ_U32(0U, model.sequence);
    fails += TEST_EQ_U32(0U, model.last_success_ms);
    fails += TEST_EQ_U32(0U, model.last_attempt_ms);
    fails += TEST_EQ_U16(0U, (uint16_t)model.valid);
    fails += TEST_EQ_U16(0U, (uint16_t)model.stale);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_INIT, (uint16_t)model.link_state);
    fails += TEST_EQ_U16(0U, model.consecutive_failures);
    fails += TEST_EQ_U32(0U, model.diag.success);
    fails += TEST_EQ_U32(0U, model.diag.timeout);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_CURRENT, (uint16_t)model.ai_mode[0]);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_VOLTAGE, (uint16_t)model.ai_mode[1]);
    return fails;
}

/* 成功采样更新序号/时间/快照/物理量, 标记有效且新鲜。 */
static int test_success_updates_sequence_time_values(void)
{
    GatewayModel_t model = {0};
    BsmSnapshot_t s1 = make_snapshot(0x05U, 0x0AU, 2000U, 2000U, 2000U, 2000U);
    BsmSnapshot_t s2 = make_snapshot(0x03U, 0x0CU, 1000U, 3000U, 1000U, 3000U);
    int fails = 0;

    GatewayModel_Init(&model, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);

    GatewayModel_ApplySuccess(&model, &s1, 1000U);
    fails += TEST_EQ_U32(1U, model.sequence);
    fails += TEST_EQ_U32(1000U, model.last_success_ms);
    fails += TEST_EQ_U32(1000U, model.last_attempt_ms);
    fails += TEST_EQ_U16(1U, (uint16_t)model.valid);
    fails += TEST_EQ_U16(0U, (uint16_t)model.stale);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_ONLINE, (uint16_t)model.link_state);
    fails += TEST_EQ_U32(1U, model.diag.success);
    fails += TEST_EQ_U16(0x05U, model.snapshot.di_bits);
    fails += TEST_EQ_U16(0x0AU, model.snapshot.relay_bits);
    fails += TEST_EQ_U32(0x00020001U, model.snapshot.di_count[0]);
    fails += TEST_EQ_U16(2000U, model.snapshot.ai_raw[0]);
    /* AI1 电压 2000->5000mV, AI2 电流 2000->10000uA */
    fails += TEST_EQ_U32(5000U, (uint32_t)model.ai_value[0]);
    fails += TEST_EQ_U32(10000U, (uint32_t)model.ai_value[1]);
    /* DA1 电压 2000->5000mV, DA2 电流 2000->10000uA */
    fails += TEST_EQ_U32(5000U, (uint32_t)model.ao_value[0]);
    fails += TEST_EQ_U32(10000U, (uint32_t)model.ao_value[1]);

    GatewayModel_ApplySuccess(&model, &s2, 2000U);
    fails += TEST_EQ_U32(2U, model.sequence);
    fails += TEST_EQ_U32(2000U, model.last_success_ms);
    fails += TEST_EQ_U16(0x03U, model.snapshot.di_bits);
    fails += TEST_EQ_U32(2500U, (uint32_t)model.ai_value[0]);  /* 1000->2500mV */
    fails += TEST_EQ_U32(15000U, (uint32_t)model.ai_value[1]); /* 3000->15000uA */
    fails += TEST_EQ_U32(2U, model.diag.success);
    return fails;
}

/* 失败采样保留旧值, 标记 stale, 计诊断并进入 DEGRADED。 */
static int test_failure_preserves_values_marks_stale(void)
{
    GatewayModel_t model = {0};
    BsmSnapshot_t s1 = make_snapshot(0x05U, 0x0AU, 2000U, 2000U, 2000U, 2000U);
    int fails = 0;

    GatewayModel_Init(&model, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);
    GatewayModel_ApplySuccess(&model, &s1, 1000U);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_TIMEOUT, 2000U,
                              GATEWAY_OFFLINE_FAILURE_THRESHOLD);

    /* 序号与成功时间不变, 旧快照/物理量保留 */
    fails += TEST_EQ_U32(1U, model.sequence);
    fails += TEST_EQ_U32(1000U, model.last_success_ms);
    fails += TEST_EQ_U16(1U, (uint16_t)model.valid);
    fails += TEST_EQ_U16(0x05U, model.snapshot.di_bits);
    fails += TEST_EQ_U32(5000U, (uint32_t)model.ai_value[0]);
    /* 失败标记与诊断 */
    fails += TEST_EQ_U32(2000U, model.last_attempt_ms);
    fails += TEST_EQ_U16(1U, (uint16_t)model.stale);
    fails += TEST_EQ_U16((uint16_t)MODBUS_STATUS_TIMEOUT, (uint16_t)model.last_status);
    fails += TEST_EQ_U32(1U, model.diag.timeout);
    fails += TEST_EQ_U16(1U, model.consecutive_failures);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_DEGRADED, (uint16_t)model.link_state);
    return fails;
}

/* 不同失败状态累计到各自诊断计数(threshold=0 时不进入 OFFLINE)。 */
static int test_failure_counts_by_status(void)
{
    GatewayModel_t model = {0};
    int fails = 0;

    GatewayModel_Init(&model, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_TIMEOUT, 1U, 0U);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_CRC_ERROR, 2U, 0U);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_EXCEPTION, 3U, 0U);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_INVALID_RESPONSE, 4U, 0U);
    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_QUEUE_FULL, 5U, 0U);

    fails += TEST_EQ_U32(1U, model.diag.timeout);
    fails += TEST_EQ_U32(1U, model.diag.crc_error);
    fails += TEST_EQ_U32(1U, model.diag.exception);
    fails += TEST_EQ_U32(2U, model.diag.other_error); /* INVALID_RESPONSE + QUEUE_FULL */
    fails += TEST_EQ_U32(0U, model.diag.success);
    fails += TEST_EQ_U16(5U, model.consecutive_failures);
    return fails;
}

/* 连续失败达阈值进入 OFFLINE; 一次成功恢复 ONLINE 并清零连续失败。 */
static int test_offline_after_threshold_then_recover(void)
{
    GatewayModel_t model = {0};
    BsmSnapshot_t s1 = make_snapshot(0x01U, 0x01U, 100U, 100U, 100U, 100U);
    int fails = 0;

    GatewayModel_Init(&model, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);
    GatewayModel_ApplySuccess(&model, &s1, 1000U);

    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_TIMEOUT, 1100U, 3U);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_DEGRADED, (uint16_t)model.link_state);
    fails += TEST_EQ_U16(1U, model.consecutive_failures);

    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_TIMEOUT, 1200U, 3U);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_DEGRADED, (uint16_t)model.link_state);

    GatewayModel_ApplyFailure(&model, MODBUS_STATUS_TIMEOUT, 1300U, 3U);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_OFFLINE, (uint16_t)model.link_state);
    fails += TEST_EQ_U16(3U, model.consecutive_failures);

    GatewayModel_ApplySuccess(&model, &s1, 2000U);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_LINK_ONLINE, (uint16_t)model.link_state);
    fails += TEST_EQ_U16(0U, model.consecutive_failures);
    fails += TEST_EQ_U16(0U, (uint16_t)model.stale);
    fails += TEST_EQ_U32(2U, model.sequence);
    return fails;
}

/* gateway_config.h 关键配置值锁定(并对配置头做主机编译覆盖)。 */
static int test_config_values(void)
{
    int fails = 0;

    fails += TEST_EQ_U16(1U, GATEWAY_BSM_SLAVE_ADDR);
    fails += TEST_EQ_U16(1000U, GATEWAY_POLL_PERIOD_MS);
    fails += TEST_EQ_U16(3U, GATEWAY_OFFLINE_FAILURE_THRESHOLD);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_VOLTAGE, (uint16_t)GATEWAY_AI1_STARTUP_MODE);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_CURRENT, (uint16_t)GATEWAY_AI2_STARTUP_MODE);
    return fails;
}

/* NULL model/snapshot 不崩溃且不改变状态。 */
static int test_null_safe(void)
{
    GatewayModel_t model = {0};
    BsmSnapshot_t s1 = make_snapshot(0x05U, 0x0AU, 2000U, 2000U, 2000U, 2000U);
    int fails = 0;

    GatewayModel_Init(NULL, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);
    GatewayModel_ApplySuccess(NULL, &s1, 1U);
    GatewayModel_ApplyFailure(NULL, MODBUS_STATUS_TIMEOUT, 1U, 3U);

    GatewayModel_Init(&model, GATEWAY_AI_MODE_VOLTAGE, GATEWAY_AI_MODE_CURRENT);
    GatewayModel_ApplySuccess(&model, NULL, 1U);
    fails += TEST_EQ_U32(0U, model.sequence);
    fails += TEST_EQ_U16(0U, (uint16_t)model.valid);
    return fails;
}

/* 非法 AI mode: Init 归一化为 INVALID; 换算返回明确哨兵, 不静默按电压。 */
static int test_invalid_ai_mode_handling(void)
{
    GatewayModel_t model = {0};
    BsmSnapshot_t s = make_snapshot(0x00U, 0x00U, 2000U, 2000U, 2000U, 2000U);
    int fails = 0;

    /* 传入非法模式 → Init 归一化为 GATEWAY_AI_MODE_INVALID, 合法模式保持原样 */
    GatewayModel_Init(&model, (GatewayAiMode_t)99, GATEWAY_AI_MODE_CURRENT);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_INVALID, (uint16_t)model.ai_mode[0]);
    fails += TEST_EQ_U16((uint16_t)GATEWAY_AI_MODE_CURRENT, (uint16_t)model.ai_mode[1]);

    /* 非法模式通道不按电压(2000→5000), 返回哨兵; 合法电流通道正常(2000→10000) */
    GatewayModel_ApplySuccess(&model, &s, 1000U);
    fails += TEST_EQ_U32((uint32_t)GATEWAY_AI_VALUE_INVALID, (uint32_t)model.ai_value[0]);
    fails += TEST_EQ_U32(10000U, (uint32_t)model.ai_value[1]);
    return fails;
}

/* 新增显式 invalid/error 枚举值存在且与有效值不同。 */
static int test_enum_invalid_values(void)
{
    int fails = 0;

    /* AI: 0 = 非法哨兵(零初始化即非法), 与 VOLTAGE 不同 */
    fails += TEST_EQ_U16(0U, (uint16_t)GATEWAY_AI_MODE_INVALID);
    fails += TEST_EQ_U16(0, (GATEWAY_AI_MODE_INVALID == GATEWAY_AI_MODE_VOLTAGE) ? 1 : 0);
    /* link: INIT 仍为 0(Init 后状态), INVALID 为独立显式值 */
    fails += TEST_EQ_U16(0U, (uint16_t)GATEWAY_LINK_INIT);
    fails += TEST_EQ_U16(0, (GATEWAY_LINK_INVALID == GATEWAY_LINK_INIT) ? 1 : 0);
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_init_clears_and_sets_modes();
    fails += test_success_updates_sequence_time_values();
    fails += test_failure_preserves_values_marks_stale();
    fails += test_failure_counts_by_status();
    fails += test_offline_after_threshold_then_recover();
    fails += test_config_values();
    fails += test_null_safe();
    fails += test_invalid_ai_mode_handling();
    fails += test_enum_invalid_values();

    if (fails == 0)
    {
        printf("All gateway model tests passed\n");
    }
    else
    {
        printf("%d gateway model assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
