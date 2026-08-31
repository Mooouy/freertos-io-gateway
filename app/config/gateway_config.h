#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

/*
 * 网关编译期配置(设备/通道/阈值)。
 *
 * 集中所有业务魔法数字: 从站地址、轮询周期、事务超时、离线阈值、AD 解释模式,
 * 以及每路 AI 的报警阈值与上报死区。其它源文件不得再散落这些常量;
 * 运行期(GatewayTask)据此初始化模型与规则。
 * Flash 持久化不属首版, 复位后恢复本文件的编译配置(见设计规格 §2.2)。
 */

#include "gateway_model.h" /* GatewayAiMode_t */

/* 现场从站与总线时序。 */
#define GATEWAY_BSM_SLAVE_ADDR 1U            /* BSM 默认 Modbus 站号 */
#define GATEWAY_POLL_PERIOD_MS 1000U         /* I/O 周期采集间隔(ms) */
#define GATEWAY_REQUEST_TIMEOUT_MS 200U      /* 单次 Modbus 事务等待上限(ms) */
#define GATEWAY_OFFLINE_FAILURE_THRESHOLD 3U /* 连续失败达到该值判定离线(规格 §8.1) */

/* AD1/AD2 启动解释模式(规格 §2.2: 软件不能从原始值自动判断, 由配置/命令指定)。 */
#define GATEWAY_AI1_STARTUP_MODE GATEWAY_AI_MODE_VOLTAGE
#define GATEWAY_AI2_STARTUP_MODE GATEWAY_AI_MODE_CURRENT

/* 每路 AI 报警阈值与上报死区(原始值域 0-4000, 与电压/电流模式无关)。 */
#define GATEWAY_AI1_HIGH_RAW 3200U   /* AD1 高报警阈值 */
#define GATEWAY_AI1_DEADBAND_RAW 40U /* AD1 变化上报死区 */
#define GATEWAY_AI2_HIGH_RAW 3200U   /* AD2 高报警阈值 */
#define GATEWAY_AI2_DEADBAND_RAW 40U /* AD2 变化上报死区 */

/*
 * GatewayTask 编排运行参数(v0.4.0)。高报警滞回带由 HIGH_RAW 与 DEADBAND_RAW 推得:
 * high_on = HIGH_RAW, high_off = HIGH_RAW - DEADBAND_RAW。
 */
#define GATEWAY_CMD_QUEUE_LEN 8U      /* 显式命令队列深度 */
#define GATEWAY_MAX_CMDS_PER_CYCLE 4U /* 每周期最多处理命令数(防轮询饥饿) */

/* 继电器互锁(默认 Y1<->Y2 互锁; 0 基通道, 禁止同时 ON)。 */
#define GATEWAY_INTERLOCK_ENABLED 1
#define GATEWAY_INTERLOCK_CH_A 0U
#define GATEWAY_INTERLOCK_CH_B 1U

/*
 * 本地演示激励: 无 MQTT 时经命令队列驱动互锁/点动/AO, 走完整编排路径。
 * v0.4.0 用于无 MQTT 时的 Windows 验收(置 1); v0.5.0 起 MQTT 接管命令下发, 置 0 关闭,
 * 以免本地演示与 MQTT 命令争用继电器/AO 输出(见 Plan 04 Task 5)。
 * 如需在无 MQTT 环境回归 v0.4.0 本地编排, 可临时置 1(demo 与 MQTT 命令互斥, 不要同时用)。
 */
#define GATEWAY_DEMO_ENABLED 0
#define GATEWAY_DEMO_PERIOD_MS 3000U /* 每 3s 投放一条演示命令 */
#define GATEWAY_DEMO_DA1_RAW 2000U   /* DA1=5V 原始值 */
#define GATEWAY_DEMO_DA2_RAW 2000U   /* DA2=10mA 原始值 */
#define GATEWAY_DEMO_PULSE_MS 1500U  /* 演示点动时长 */

/*
 * ESP8266/MQTT 下行诊断日志(编译期开关, 默认关闭)。
 * 置 1 时 esp8266_task 在任务上下文打印命令下行路径诊断: +MQTTSUBRECV 行、topic/payload 长度、
 * decode 状态与 id/type、submit 结果, 以及解析失败时的 parse 状态与限长原始行, 用于复现现场
 * MQTT 联调问题(如发送端给 payload 追加换行, 使 +MQTTSUBRECV 声明长度与可见长度不一致)。
 * release 固件默认置 0 以免刷屏; 核心状态日志(ESP8266 task started / ESP online / ESP backoff)
 * 不受此开关影响, 始终打印。仅影响诊断可见性, 不改变业务逻辑/ISR/MQTT 行为。
 */
#define GATEWAY_ESP_DIAG_LOG_ENABLED 0

#endif /* GATEWAY_CONFIG_H */
