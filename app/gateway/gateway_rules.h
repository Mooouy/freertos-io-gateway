#ifndef GATEWAY_RULES_H
#define GATEWAY_RULES_H

/*
 * 网关确定性本地规则(纯逻辑层): AI 高报警滞回、继电器互锁、继电器点动。
 *
 * 全部为值语义纯函数: 不依赖 HAL/FreeRTOS/Modbus service/printf/硬件状态;
 * 时间(tick, ms)与配置阈值均由调用者传入。规则只计算"期望结果",
 * 实际 I/O 由编排任务(GatewayTask)执行。无动态内存。
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsm0404.h" /* BSM_CHANNEL_COUNT: 继电器/DI 通道数 */

/* 规则求值状态(public enum, 含显式 invalid 哨兵)。 */
typedef enum
{
    GATEWAY_RULE_INVALID = 0,    /* 显式无效/未知哨兵: 参数非法(空指针/通道越界) */
    GATEWAY_RULE_OK,             /* 求值成功 */
    GATEWAY_RULE_INVALID_CONFIG, /* 配置非法: high_off>=high_on 或互锁通道非法 */
    GATEWAY_RULE_REJECTED,       /* 请求被拒绝: 互锁对将同时 ON */
} GatewayRuleStatus_t;

/* 继电器互锁对配置: enabled 时禁止 channel_a 与 channel_b 同时 ON(0 基通道号)。 */
typedef struct
{
    uint8_t channel_a;
    uint8_t channel_b;
    bool enabled;
} GatewayInterlock_t;

/*
 * AI 高报警滞回(避免阈值附近反复触发):
 *  - raw >= high_on          -> 进入 active;
 *  - high_off < raw < high_on -> 保持 current_active;
 *  - raw <= high_off         -> 清除 active。
 * 要求 high_off < high_on, 否则返回 GATEWAY_RULE_INVALID_CONFIG。
 * next_active 为空返回 GATEWAY_RULE_INVALID; 非 OK 时 *next_active 置为 current_active(不变)。
 */
GatewayRuleStatus_t GatewayRules_HighAlarm(bool current_active,
                                           uint16_t raw,
                                           uint16_t high_on,
                                           uint16_t high_off,
                                           bool *next_active);

/*
 * 继电器置位(带互锁校验):
 *  - 在 current_bitmap 上对 channel 置 turn_on, 得到 proposed;
 *  - interlock 启用且 proposed 使 channel_a/b 同时 ON -> GATEWAY_RULE_REJECTED;
 *  - interlock 启用但通道非法(越界或 a==b)     -> GATEWAY_RULE_INVALID_CONFIG;
 *  - channel 越界或 next_bitmap 为空            -> GATEWAY_RULE_INVALID;
 *  - 否则 GATEWAY_RULE_OK, *next_bitmap = proposed。
 * REJECTED / INVALID_CONFIG 时 *next_bitmap = current_bitmap(保持原状);
 * INVALID(参数错误)时 *next_bitmap 可能不被写入。
 */
GatewayRuleStatus_t GatewayRules_RelaySet(uint8_t current_bitmap,
                                          uint8_t channel,
                                          bool turn_on,
                                          const GatewayInterlock_t *interlock,
                                          uint8_t *next_bitmap);

/* 点动: 由起始 tick 与持续时长算出结束 tick(uint32_t 自然回绕)。 */
uint32_t GatewayRules_PulseDeadline(uint32_t start_ms, uint32_t duration_ms);

/*
 * 点动求值: now 早于 deadline 返回 true(保持 ON), 到/过 deadline 返回 false(OFF)。
 * 用有符号差值 (int32_t)(deadline - now) > 0 容忍 uint32_t 回绕(脉宽需 < 2^31 ms)。
 */
bool GatewayRules_PulseActive(uint32_t now_ms, uint32_t deadline_ms);

#endif /* GATEWAY_RULES_H */
