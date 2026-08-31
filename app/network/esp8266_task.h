#ifndef ESP8266_TASK_H
#define ESP8266_TASK_H

/*
 * ESP8266 Wi-Fi/MQTT 协处理器任务(v0.5.0, 设计规格 §3.4/§8.2, 参考行为见
 * docs/references/esp8266-mqtt-reference.md)。
 *
 * 独占 ESP8266 UART(USART2)与 AT 会话, 以非阻塞状态机复现参考工程已验证的接入顺序
 * (AT+CWMODE/CWJAP/MQTTUSERCFG/MQTTCONN/MQTTSUB/周期 MQTTPUB), 每条 AT 命令带
 * deadline 与 retry, 不用 HAL_Delay 盲等。UART ISR 只把字节投递到 FreeRTOS Stream Buffer;
 * 行装配/分类/JSON 解析全在任务上下文用 esp_at + mqtt_codec 完成。
 *
 * 与 GatewayTask 的边界: MQTT 命令经 mqtt_codec 校验后 GatewayTask_SubmitCommand 下发,
 * 命令结果经 GatewayTask_ReceiveResult 取回并发布 reply; 遥测经 GatewayTask_CopyModel
 * 读取一致快照后 mqtt_codec 编码发布。真实凭据来自本地 network_config.h(不入库)。
 */

/* 创建 RX Stream Buffer、初始化行解析器并武装 USART2 接收。在调度器启动前(USER CODE Init)调用。 */
void Esp8266_Init(void);

/* 创建 ESP8266 任务(静态分配, 不占用 FreeRTOS heap)。在 RTOS_THREADS 段调用。 */
void Esp8266_StartTask(void);

/*
 * 固件中断集成钩子: 由 Core/Src/main.c 的 HAL_UART_RxCpltCallback 按实例转发。
 * 仅当 huart 为 USART2 时把收到的字节投递到 Stream Buffer 并重新武装接收;
 * 中断中不解析、不 JSON、不 printf、不阻塞(规格 §4)。
 * 仅固件构建(USE_HAL_DRIVER)可见, 保持与纯逻辑模块的 HAL 解耦。
 */
#ifdef USE_HAL_DRIVER
#include "stm32f1xx_hal.h"
void Esp8266_OnUartRxCplt(UART_HandleTypeDef *huart);
#endif /* USE_HAL_DRIVER */

#endif /* ESP8266_TASK_H */
