#ifndef GATEWAY_TASK_H
#define GATEWAY_TASK_H

/*
 * 网关编排层(GatewayTask)对外接口。
 *
 * GatewayTask 是本地网关业务的唯一编排者: 经 modbus_service 串行访问 BSM,
 * 用 gateway_model 维护统一 I/O 状态与诊断, 用 gateway_rules 执行滞回/互锁/点动,
 * 并消费显式控制命令。它不直接访问 USART3/RS485, 不含 MQTT/ESP8266 逻辑(属 v0.5.0)。
 *
 * 本头只依赖标准类型, 不暴露 HAL/FreeRTOS 句柄: 命令生产者(v0.5.0 的 MQTT 任务,
 * v0.4.0 的本地演示)通过 GatewayTask_SubmitCommand 投递命令。
 */

#include <stdbool.h>
#include <stdint.h>

#include "gateway_model.h" /* GatewayModel_t, GatewayAiMode_t */

/* 网关控制命令类型(public enum, 含显式 invalid 哨兵)。 */
typedef enum
{
    GATEWAY_CMD_INVALID = 0,  /* 显式无效哨兵: 不入队 */
    GATEWAY_CMD_RELAY_SET,    /* 置继电器通道 on/off(经互锁校验, 写后回读确认) */
    GATEWAY_CMD_RELAY_PULSE,  /* 继电器点动: 立即 ON, duration_ms 后自动 OFF */
    GATEWAY_CMD_AO_SET,       /* 设模拟量输出原始值(0-4000) */
    GATEWAY_CMD_AI_MODE_SET,  /* 设 AI 通道解释模式(value 复用为 GatewayAiMode_t) */
} GatewayCommandType_t;

/* 一条网关控制命令(值语义, 可放入队列)。 */
typedef struct
{
    GatewayCommandType_t type;
    uint8_t channel;      /* 继电器/AO/AI 通道(0 基) */
    uint16_t value;       /* RELAY_SET: 0/1; AO_SET: 原始值 0-4000; AI_MODE_SET: GatewayAiMode_t */
    uint32_t duration_ms; /* RELAY_PULSE 持续时长 */
    uint32_t request_id;  /* MQTT 命令关联 id; 0 表示无需回复(如本地来源) */
} GatewayCommand_t;

/* 命令提交结果(public enum, 含显式 invalid 哨兵)。 */
typedef enum
{
    GATEWAY_SUBMIT_INVALID = 0, /* 空命令 / 无效类型 / 队列未建 */
    GATEWAY_SUBMIT_OK,          /* 已入队 */
    GATEWAY_SUBMIT_FULL,        /* 队列已满, 未入队 */
} GatewaySubmitResult_t;

/* 命令执行结果码(供 MQTT 回复映射; 含显式 invalid 哨兵)。 */
typedef enum
{
    GATEWAY_RESULT_INVALID = 0, /* 命令类型不对 */
    GATEWAY_RESULT_OK,          /* 执行成功(写后回读确认) */
    GATEWAY_RESULT_REJECTED,    /* 互锁/参数拒绝, 未改变输出（命令合法，但规则不允许） */
    GATEWAY_RESULT_FAILED,      /* Modbus 写/回读失败 （命令可执行，但设备通信失败）*/
    GATEWAY_RESULT_NOT_READY,   /* 首次成功快照前, 拒绝写输出 （系统没拿到初始状态，拒绝执行）*/
} GatewayCmdResult_t;

/* 一条命令执行结果(值语义, 经结果队列回传给命令发起方/MQTT 任务)。 */
typedef struct
{
    uint32_t request_id;       /* 对应命令的关联 id */
    GatewayCmdResult_t result; /* 执行结果 */
    int32_t actual;            /* 实际值: 继电器位 / AO raw / AI 模式 */
} GatewayCommandResult_t;

/*
 * 初始化网关编排层: 创建静态命令队列, 并按 gateway_config 初始化数据模型。
 * 须在 RTOS 调度器启动前(MX_FREERTOS_Init 的 USER CODE Init)调用一次。
 */
void GatewayTask_Init(void);

/*
 * 提交一条显式控制命令(非阻塞, 可由其它任务调用)。
 * 命令为空 / 类型非法 / 队列未建返回 GATEWAY_SUBMIT_INVALID; 队列满返回 GATEWAY_SUBMIT_FULL。
 */
GatewaySubmitResult_t GatewayTask_SubmitCommand(const GatewayCommand_t *command);

/*
 * 取回一条命令执行结果(带超时)。有结果返回 true 并写 out; 超时/无结果返回 false。
 * 由 MQTT 任务消费, 据此发布 reply。仅带 request_id(≠0) 的命令执行后才入队结果。
 */
bool GatewayTask_ReceiveResult(GatewayCommandResult_t *out, uint32_t timeout_ms);

/*
 * 拷贝当前 I/O 数据模型的一致快照(用于遥测), 值语义。
 * 内部在临界区整体复制, 与 GatewayTask 周期更新互斥, 避免撕裂读; out 为空则不操作。
 */
void GatewayTask_CopyModel(GatewayModel_t *out);

#endif /* GATEWAY_TASK_H */
