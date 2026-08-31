#ifndef GATEWAY_MODEL_H
#define GATEWAY_MODEL_H

/*
 * 网关统一 I/O 数据模型(纯逻辑层, 见设计规格 §6)。
 *
 * 持有最近一次有效 I/O 快照、模拟量物理量、采样时序、Modbus 链路状态与显式诊断计数,
 * 并提供"应用一次成功/失败采样"的纯状态转换。
 *
 * 本文件刻意不依赖 HAL/FreeRTOS, 可在主机环境单元测试; 时间以毫秒由调用者传入,
 * 离线阈值等策略常量也由调用者(GatewayTask)从 gateway_config.h 传入, 模型本身不含配置。
 */

#include <stdbool.h>
#include <stdint.h>

#include "bsm0404.h"      /* BsmSnapshot_t, BSM_AI_COUNT, BSM_AO_COUNT, 换算函数 */
#include "modbus_types.h" /* ModbusStatus_t */

/* AD 通道物理解释模式(规格 §2.2: 由现场按钮选择, 软件不自动判断)。 */
typedef enum
{
    GATEWAY_AI_MODE_INVALID = 0, /* 未配置/非法: 零初始化即落此值, 不得当作有效模式 */
    GATEWAY_AI_MODE_VOLTAGE,     /* 0-10V, 物理量单位 mV */
    GATEWAY_AI_MODE_CURRENT,     /* 0-20mA, 物理量单位 uA */
} GatewayAiMode_t;

/*
 * AI 换算物理量哨兵: 模式非法或原始值越界、无法换算时返回。
 * 物理量恒为非负, 取 -1 不会与真实读数(0..20000)混淆, 便于下游识别为无效。
 */
#define GATEWAY_AI_VALUE_INVALID (-1)

/* Modbus 链路状态(规格 §8.1 的子集; RECOVERING 等全量重同步语义留待编排任务)。 */
typedef enum
{
    GATEWAY_LINK_INIT = 0, /* 已初始化, 尚无成功采样(Init 后的状态) */
    GATEWAY_LINK_ONLINE,   /* 最近一次采样成功 */
    GATEWAY_LINK_DEGRADED, /* 出现失败但未达离线阈值 */
    GATEWAY_LINK_OFFLINE,  /* 连续失败达到离线阈值 */
    GATEWAY_LINK_INVALID,  /* 显式非法/未知哨兵(满足"每个枚举具备 invalid 值") */
} GatewayLinkState_t;

/* 显式 Modbus 诊断计数器(规格 §6: 成功/超时/CRC/异常分别计数)。 */
typedef struct
{
    uint32_t success;     /* 成功采样次数 */
    uint32_t timeout;     /* 等待响应超时次数 */
    uint32_t crc_error;   /* 响应 CRC 错误次数 */
    uint32_t exception;   /* 从站异常响应次数 */
    uint32_t other_error; /* 其它失败(无效响应/队列满/离线等) */
} GatewayDiagCounters_t;

/* 统一网关 I/O 数据模型。纯数据 + 纯状态, 不含动态内存与 RTOS 句柄。 */
typedef struct
{
    BsmSnapshot_t snapshot;                /* 最近一次有效 I/O 快照(原始) */
    GatewayAiMode_t ai_mode[BSM_AI_COUNT]; /* AD1/AD2 解释模式 */
    int32_t ai_value[BSM_AI_COUNT];        /* AD 物理量(VOLTAGE:mV / CURRENT:uA; 非法→GATEWAY_AI_VALUE_INVALID) */
    int32_t ao_value[BSM_AO_COUNT];        /* DA 换算物理量(DA1:mV / DA2:uA) */

    bool valid;               /* 是否已有过一次成功采样 */
    bool stale;               /* 最近一次采样失败, 当前值为陈旧值 */
    uint32_t sequence;        /* 成功采样序号, 每次成功 +1 */
    uint32_t last_success_ms; /* 最近一次成功采样时间(ms) */
    uint32_t last_attempt_ms; /* 最近一次采样尝试时间(ms, 成功或失败) */

    GatewayLinkState_t link_state; /* Modbus 链路状态 */
    ModbusStatus_t last_status;    /* 最近一次采样结果状态 */
    uint16_t consecutive_failures; /* 连续失败次数 */

    GatewayDiagCounters_t diag; /* 显式诊断计数器 */
} GatewayModel_t;

/* 初始化为"尚无数据"状态(全部清零), 并设定 AD1/AD2 解释模式。model 为空则不操作。 */
void GatewayModel_Init(GatewayModel_t *model,
                       GatewayAiMode_t ai1_mode,
                       GatewayAiMode_t ai2_mode);

/*
 * 应用一次成功采样:
 *  - 覆盖快照并按各自模式换算 AI/AO 物理量;
 *  - sequence +1, 更新 last_success_ms 与 last_attempt_ms;
 *  - 置 valid=true、stale=false、link_state=ONLINE, 清零连续失败计数;
 *  - diag.success +1。
 * model 或 snapshot 为空则不操作。
 */
void GatewayModel_ApplySuccess(GatewayModel_t *model,
                               const BsmSnapshot_t *snapshot,
                               uint32_t now_ms);

/*
 * 应用一次失败采样:
 *  - 保留上一次快照与物理量, 不改 sequence/last_success_ms;
 *  - 置 stale=true, 更新 last_attempt_ms 与 last_status;
 *  - 连续失败 +1: 达到 offline_threshold(非 0)则置 OFFLINE, 否则 DEGRADED;
 *  - 按 status 累计对应诊断计数。
 * model 为空则不操作。
 */
void GatewayModel_ApplyFailure(GatewayModel_t *model,
                               ModbusStatus_t status,
                               uint32_t now_ms,
                               uint16_t offline_threshold);

#endif /* GATEWAY_MODEL_H */
