#ifndef MODBUS_SERVICE_H
#define MODBUS_SERVICE_H

/*
 * Modbus service 接口契约。
 *
 * service 是 RS485(USART3)总线的唯一拥有者: 调用者按值提交 ModbusRequest_t,
 * service 串行执行总线事务, 并把 ModbusResult_t 交回请求方。
 * 本头只声明接口, 运行时实现见 modbus_service.c(v0.3.0 Task 2)。
 *
 * 接口层与 RTOS 解耦: 只依赖 modbus_types.h 的值契约, 不暴露 FreeRTOS 句柄,
 * 因此可在纯主机环境编译。
 */

#include "modbus_types.h"

/* 初始化 Modbus service: 创建请求/响应队列。需在 RTOS 调度器启动前调用一次。 */
void Modbus_Service_Init(void);

/*
 * 同步提交一次 Modbus 请求并等待结果。
 * - request、result 均为值语义, service 不保留调用者指针;
 * - 阻塞调用方任务最多 request->timeout_ms 毫秒;
 * - 返回最终状态, 同时写入 *result(含异常码与寄存器);
 * - 请求队列已满时立即返回 MODBUS_STATUS_QUEUE_FULL。
 */
ModbusStatus_t Modbus_Service_Submit(const ModbusRequest_t *request,
                                     ModbusResult_t *result);

/*
 * 固件中断集成钩子。
 * 由 Core/Src/main.c 的 HAL 回调按外设实例过滤后转发: USART3 字节收完调用
 * 前者, TIM2 帧间隔到点调用后者。service 在中断上下文只做字节收集与整帧投递,
 * 不打印/不解析/不阻塞, 用 FromISR API 投递完整帧。
 * 仅固件构建(定义 USE_HAL_DRIVER)可见; 纯主机契约测试不包含这一段, 故契约 API 仍与 HAL 解耦。
 */
#ifdef USE_HAL_DRIVER
#include "stm32f1xx_hal.h"
void Modbus_Service_OnUartRxCplt(UART_HandleTypeDef *huart);
void Modbus_Service_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
#endif /* USE_HAL_DRIVER */

#endif /* MODBUS_SERVICE_H */
