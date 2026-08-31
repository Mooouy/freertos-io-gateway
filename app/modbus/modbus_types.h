#ifndef MODBUS_TYPES_H
#define MODBUS_TYPES_H

/*
 * Modbus service 的数据契约(请求/结果)。
 *
 * 本头刻意不依赖 HAL 和 FreeRTOS: 请求方句柄用不透明的 void* 占位,
 * service 层(modbus_service.c)负责转换为 FreeRTOS TaskHandle_t 并发任务通知。
 * 这样契约层可在纯主机环境编译与单元测试。
 *
 * 请求与结果都采用值语义并使用固定大小内联数组, 便于按值放入 FreeRTOS 队列,
 * 服务端不保留调用者缓冲区指针。
 */

#include <stdint.h>

/* 内联数组容量。 */
#define MODBUS_REQUEST_MAX_VALUES 8U    /* 单次写多寄存器的最大数据个数 */
#define MODBUS_RESULT_MAX_REGISTERS 16U /* 单次响应可回填的最大寄存器个数 */

/* 队列内存预算(字节): 限制请求/结果尺寸, 防止队列对象失控。 */
#define MODBUS_REQUEST_MAX_BYTES 48U
#define MODBUS_RESULT_MAX_BYTES 48U

/*
 * 请求方句柄。运行时底层是 FreeRTOS TaskHandle_t, 这里以不透明指针占位,
 * 让契约层与 RTOS 解耦; service 完成请求后据此通知请求方任务。
 */
typedef void *ModbusRequester_t;

/* Modbus 操作类型, 对应功能码 03/06/16。 */
typedef enum
{
    MODBUS_OP_READ_HOLDING = 0, /* 03 读保持寄存器 */
    MODBUS_OP_WRITE_SINGLE,     /* 06 写单个寄存器 */
    MODBUS_OP_WRITE_MULTIPLE,   /* 16 写多个寄存器 */
} ModbusOperation_t;

/* Modbus 请求最终状态, 供固件各层与上位机共享的契约。 */
typedef enum
{
    MODBUS_STATUS_OK = 0,          /* 成功 */
    MODBUS_STATUS_TIMEOUT,         /* 等待响应超时 */
    MODBUS_STATUS_CRC_ERROR,       /* 响应 CRC 校验失败 */
    MODBUS_STATUS_EXCEPTION,       /* 从站返回异常帧 */
    MODBUS_STATUS_INVALID_RESPONSE,/* 响应不匹配或格式非法 */
    MODBUS_STATUS_QUEUE_FULL,      /* 请求队列已满, 未入队 */
    MODBUS_STATUS_OFFLINE,         /* 从站连续无响应, 判定离线 */
} ModbusStatus_t;

/*
 * 一次 Modbus 请求(值语义)。
 * - operation 决定使用 address/count/values 的哪些字段;
 * - 读操作用 count 指定寄存器个数; 写多寄存器用 values[0..count-1];
 * - timeout_ms 为单次请求的等待上限;
 * - requester 供 service 完成后通知请求方任务。
 */
typedef struct
{
    ModbusRequester_t requester;                   /* 请求方任务句柄(不透明) */
    uint32_t timeout_ms;                           /* 等待响应上限(毫秒) */
    ModbusOperation_t operation;                   /* 操作类型 */
    uint16_t address;                              /* 起始寄存器地址 */
    uint16_t count;                                /* 寄存器个数 */
    uint16_t values[MODBUS_REQUEST_MAX_VALUES];    /* 写操作的数据 */
    uint8_t slave;                                 /* 从站地址 */
} ModbusRequest_t;

/*
 * 一次 Modbus 请求的结果(值语义)。
 * - status 为最终状态;
 * - exception_code 仅当 status 为 MODBUS_STATUS_EXCEPTION 时有效;
 * - registers[0..register_count-1] 为读操作回填的寄存器值。
 */
typedef struct
{
    ModbusStatus_t status;                          /* 最终状态 */
    uint16_t registers[MODBUS_RESULT_MAX_REGISTERS];/* 读回的寄存器值 */
    uint8_t register_count;                         /* 有效寄存器个数 */
    uint8_t exception_code;                          /* 异常码(异常时有效) */
} ModbusResult_t;

/* 队列内存预算: 在编译期约束请求/结果尺寸, 避免队列对象膨胀。 */
_Static_assert(sizeof(ModbusRequest_t) <= MODBUS_REQUEST_MAX_BYTES,
               "ModbusRequest_t exceeds queue memory budget");
_Static_assert(sizeof(ModbusResult_t) <= MODBUS_RESULT_MAX_BYTES,
               "ModbusResult_t exceeds queue memory budget");

#endif /* MODBUS_TYPES_H */
