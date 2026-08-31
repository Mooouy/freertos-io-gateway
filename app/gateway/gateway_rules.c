/*
 * 网关确定性本地规则实现(纯逻辑, 见 gateway_rules.h)。
 *
 * 全部为整数/位运算与值拷贝: 无动态内存、无 RTOS/HAL 依赖、无 I/O 副作用。
 */

#include "gateway_rules.h"

GatewayRuleStatus_t GatewayRules_HighAlarm(bool current_active, uint16_t raw,
                                           uint16_t high_on, uint16_t high_off,
                                           bool *next_active)
{
    if (next_active == NULL)
    {
        return GATEWAY_RULE_INVALID;
    }
    *next_active = current_active; /* 默认: 维持原状态 */
    if (high_off >= high_on)
    {
        return GATEWAY_RULE_INVALID_CONFIG;
    }

    if (raw >= high_on)
    {
        *next_active = true;
    }
    else if (raw <= high_off)
    {
        *next_active = false;
    }
    /* high_off < raw < high_on: 维持(滞回) */
    return GATEWAY_RULE_OK;
}

GatewayRuleStatus_t GatewayRules_RelaySet(uint8_t current_bitmap, uint8_t channel,
                                          bool turn_on,
                                          const GatewayInterlock_t *interlock,
                                          uint8_t *next_bitmap)
{
    uint8_t proposed;

    if (next_bitmap == NULL || channel >= BSM_CHANNEL_COUNT)
    {
        return GATEWAY_RULE_INVALID;
    }
    *next_bitmap = current_bitmap; /* 默认: 保持原状 */

    proposed = current_bitmap;
    if (turn_on)
    {
        proposed = (uint8_t)(proposed | (uint8_t)(1U << channel));
    }
    else
    {
        proposed = (uint8_t)(proposed & (uint8_t)(~(1U << channel)));
    }

    if (interlock != NULL && interlock->enabled)
    {
        uint8_t mask;
        if (interlock->channel_a >= BSM_CHANNEL_COUNT ||
            interlock->channel_b >= BSM_CHANNEL_COUNT ||
            interlock->channel_a == interlock->channel_b)
        {
            return GATEWAY_RULE_INVALID_CONFIG;
        }
        mask = (uint8_t)((1U << interlock->channel_a) | (1U << interlock->channel_b));
        if ((uint8_t)(proposed & mask) == mask)
        {
            return GATEWAY_RULE_REJECTED; /* 互锁对将同时 ON: 拒绝并保持原状 */
        }
    }

    *next_bitmap = proposed;
    return GATEWAY_RULE_OK;
}

uint32_t GatewayRules_PulseDeadline(uint32_t start_ms, uint32_t duration_ms)
{
    return start_ms + duration_ms; /* uint32_t 自然回绕 */
}

bool GatewayRules_PulseActive(uint32_t now_ms, uint32_t deadline_ms)
{
    /* 有符号差值容忍 tick 回绕: now 早于 deadline 为正(ON), 到/过为非正(OFF)。 */
    return (int32_t)(deadline_ms - now_ms) > 0;
}
