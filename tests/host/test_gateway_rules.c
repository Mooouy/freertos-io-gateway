/*
 * 网关确定性本地规则测试(纯逻辑)。
 *
 * 覆盖: AI 高报警滞回的 enter/hold/clear 与非法配置、继电器互锁拒绝同时 ON 并保持原状、
 * 互锁/参数非法、点动立即 ON 与 deadline OFF, 以及 deadline 的 uint32_t tick 回绕。
 * 不依赖 HAL/FreeRTOS。
 */

#include "gateway_rules.h"
#include "test_assert.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static GatewayInterlock_t make_lock(uint8_t a, uint8_t b, bool enabled)
{
    GatewayInterlock_t lock = {0};

    lock.channel_a = a;
    lock.channel_b = b;
    lock.enabled = enabled;
    return lock;
}

/* 高报警滞回: 低于 high_off 清除, 带内保持, 达到 high_on 进入。 */
static int test_high_alarm_enter_hold_clear(void)
{
    bool active = false;
    int fails = 0;

    /* 低于 high_off -> 不报警 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_HighAlarm(false, 1500U, 3000U, 2000U, &active));
    fails += TEST_EQ_U16(0, (uint16_t)active);
    /* 带内(high_off<raw<high_on)且当前未报警 -> 维持未报警 */
    GatewayRules_HighAlarm(false, 2500U, 3000U, 2000U, &active);
    fails += TEST_EQ_U16(0, (uint16_t)active);
    /* 达到 high_on -> 进入报警 */
    GatewayRules_HighAlarm(false, 3000U, 3000U, 2000U, &active);
    fails += TEST_EQ_U16(1, (uint16_t)active);
    /* 带内且当前报警 -> 维持报警(滞回) */
    GatewayRules_HighAlarm(true, 2500U, 3000U, 2000U, &active);
    fails += TEST_EQ_U16(1, (uint16_t)active);
    /* 到达 high_off -> 清除报警 */
    GatewayRules_HighAlarm(true, 2000U, 3000U, 2000U, &active);
    fails += TEST_EQ_U16(0, (uint16_t)active);
    return fails;
}

/* 非法滞回配置(high_off>=high_on)与空指针: 明确返回状态, 状态保持。 */
static int test_high_alarm_invalid_config(void)
{
    bool active = true;
    int fails = 0;

    /* high_on==high_off -> INVALID_CONFIG, 保持 current_active=true */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID_CONFIG,
                         GatewayRules_HighAlarm(true, 5000U, 2000U, 2000U, &active));
    fails += TEST_EQ_U16(1, (uint16_t)active);
    /* high_off>high_on -> INVALID_CONFIG, 保持 current_active=false */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID_CONFIG,
                         GatewayRules_HighAlarm(false, 5000U, 2000U, 3000U, &active));
    fails += TEST_EQ_U16(0, (uint16_t)active);
    /* next_active 为空 -> INVALID */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID,
                         GatewayRules_HighAlarm(false, 100U, 3000U, 2000U, NULL));
    return fails;
}

/* 互锁: 拒绝使互锁对同时 ON 的请求, 且不改变 current relay state。 */
static int test_interlock_reject_and_preserve(void)
{
    GatewayInterlock_t lock = make_lock(0U, 1U, true);
    uint8_t next = 0xFFU;
    int fails = 0;

    /* 当前 Y1 ON(0x01), 请求开 Y2 -> 互锁对将同时 ON -> 拒绝, 保持 0x01 */
    fails += TEST_EQ_U16(GATEWAY_RULE_REJECTED,
                         GatewayRules_RelaySet(0x01U, 1U, true, &lock, &next));
    fails += TEST_EQ_U16(0x01U, next);
    /* 当前全关, 请求开 Y2 -> 允许 -> 0x02 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_RelaySet(0x00U, 1U, true, &lock, &next));
    fails += TEST_EQ_U16(0x02U, next);
    /* 关断永远允许(不会造成同时 ON): 0x01 关 Y1 -> 0x00 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_RelaySet(0x01U, 0U, false, &lock, &next));
    fails += TEST_EQ_U16(0x00U, next);
    /* 非互锁通道不受影响: 0x01 开 Y3 -> 0x05 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_RelaySet(0x01U, 2U, true, &lock, &next));
    fails += TEST_EQ_U16(0x05U, next);
    return fails;
}

/* 互锁/参数非法与禁用互锁的处理。 */
static int test_relay_invalid_and_disabled(void)
{
    GatewayInterlock_t bad = make_lock(1U, 1U, true);  /* a==b */
    GatewayInterlock_t oob = make_lock(0U, 9U, true);  /* 通道越界 */
    GatewayInterlock_t off = make_lock(0U, 1U, false); /* 禁用 */
    uint8_t next = 0xFFU;
    int fails = 0;

    /* a==b -> INVALID_CONFIG, 保持原状 0x02 */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID_CONFIG,
                         GatewayRules_RelaySet(0x02U, 0U, true, &bad, &next));
    fails += TEST_EQ_U16(0x02U, next);
    /* 互锁通道越界 -> INVALID_CONFIG, 保持原状 0x02 */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID_CONFIG,
                         GatewayRules_RelaySet(0x02U, 0U, true, &oob, &next));
    fails += TEST_EQ_U16(0x02U, next);
    /* 目标通道越界 -> INVALID */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID,
                         GatewayRules_RelaySet(0x00U, 9U, true, NULL, &next));
    /* next_bitmap 为空 -> INVALID */
    fails += TEST_EQ_U16(GATEWAY_RULE_INVALID,
                         GatewayRules_RelaySet(0x00U, 0U, true, NULL, NULL));
    /* 无互锁(NULL) -> 允许 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_RelaySet(0x00U, 0U, true, NULL, &next));
    fails += TEST_EQ_U16(0x01U, next);
    /* 互锁禁用 -> 即使会同时 ON 也允许 */
    fails += TEST_EQ_U16(GATEWAY_RULE_OK,
                         GatewayRules_RelaySet(0x01U, 1U, true, &off, &next));
    fails += TEST_EQ_U16(0x03U, next);
    return fails;
}

/* 点动: 立即 ON, 到 deadline OFF。 */
static int test_pulse_immediate_on_deadline_off(void)
{
    uint32_t deadline = GatewayRules_PulseDeadline(1000U, 500U);
    int fails = 0;

    fails += TEST_EQ_U32(1500U, deadline);
    fails += TEST_EQ_U16(1, (uint16_t)GatewayRules_PulseActive(1000U, deadline)); /* 立即 ON */
    fails += TEST_EQ_U16(1, (uint16_t)GatewayRules_PulseActive(1499U, deadline)); /* 到期前 ON */
    fails += TEST_EQ_U16(0, (uint16_t)GatewayRules_PulseActive(1500U, deadline)); /* 到期 OFF */
    fails += TEST_EQ_U16(0, (uint16_t)GatewayRules_PulseActive(1600U, deadline)); /* 过期 OFF */
    return fails;
}

/* 点动 deadline 的 uint32_t tick 回绕(有符号差值)。 */
static int test_pulse_tick_wrap(void)
{
    uint32_t start = 0xFFFFFF00U;
    uint32_t deadline = GatewayRules_PulseDeadline(start, 0x200U); /* 回绕到 0x100 */
    int fails = 0;

    fails += TEST_EQ_U32(0x00000100U, deadline);
    fails += TEST_EQ_U16(1, (uint16_t)GatewayRules_PulseActive(start, deadline));        /* 起始 ON */
    fails += TEST_EQ_U16(1, (uint16_t)GatewayRules_PulseActive(0xFFFFFFFFU, deadline));  /* 跨回绕 ON */
    fails += TEST_EQ_U16(1, (uint16_t)GatewayRules_PulseActive(0x000000FFU, deadline));  /* 到期前 ON */
    fails += TEST_EQ_U16(0, (uint16_t)GatewayRules_PulseActive(0x00000100U, deadline));  /* 到期 OFF */
    fails += TEST_EQ_U16(0, (uint16_t)GatewayRules_PulseActive(0x00000200U, deadline));  /* 过期 OFF */
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_high_alarm_enter_hold_clear();
    fails += test_high_alarm_invalid_config();
    fails += test_interlock_reject_and_preserve();
    fails += test_relay_invalid_and_disabled();
    fails += test_pulse_immediate_on_deadline_off();
    fails += test_pulse_tick_wrap();

    if (fails == 0)
    {
        printf("All gateway rules tests passed\n");
    }
    else
    {
        printf("%d gateway rules assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
