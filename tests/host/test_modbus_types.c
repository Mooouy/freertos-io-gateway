/*
 * Modbus service 契约测试(纯类型层)。
 *
 * 只校验请求/结果契约的稳定性: 操作码与状态码取值、内联数组容量、
 * 结构体尺寸预算。刻意只包含 service/types 头, 不引入 HAL/FreeRTOS,
 * 以证明契约层可在主机独立编译。
 */

#include "modbus_service.h"
#include "modbus_types.h"
#include "test_assert.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* 操作码取值稳定: 与协议功能码映射一一对应, 0/1/2。 */
static int test_operation_enum_values(void)
{
    int fails = 0;
    fails += TEST_EQ_U16(0, MODBUS_OP_READ_HOLDING);
    fails += TEST_EQ_U16(1, MODBUS_OP_WRITE_SINGLE);
    fails += TEST_EQ_U16(2, MODBUS_OP_WRITE_MULTIPLE);
    return fails;
}

/* 结果状态取值稳定: 固件与上位机共享的契约, 0..6。 */
static int test_status_enum_values(void)
{
    int fails = 0;
    fails += TEST_EQ_U16(0, MODBUS_STATUS_OK);
    fails += TEST_EQ_U16(1, MODBUS_STATUS_TIMEOUT);
    fails += TEST_EQ_U16(2, MODBUS_STATUS_CRC_ERROR);
    fails += TEST_EQ_U16(3, MODBUS_STATUS_EXCEPTION);
    fails += TEST_EQ_U16(4, MODBUS_STATUS_INVALID_RESPONSE);
    fails += TEST_EQ_U16(5, MODBUS_STATUS_QUEUE_FULL);
    fails += TEST_EQ_U16(6, MODBUS_STATUS_OFFLINE);
    return fails;
}

/* 内联容量: 请求最多 8 个写入值, 结果最多 16 个寄存器。 */
static int test_inline_capacities(void)
{
    /* 用空指针成员的 sizeof 取容量, 无需实例, 避免未初始化变量。 */
    const size_t values_cap =
        sizeof(((ModbusRequest_t *)0)->values) / sizeof(uint16_t);
    const size_t registers_cap =
        sizeof(((ModbusResult_t *)0)->registers) / sizeof(uint16_t);
    int fails = 0;

    fails += TEST_EQ_U16(8, (uint16_t)values_cap);
    fails += TEST_EQ_U16(MODBUS_REQUEST_MAX_VALUES, (uint16_t)values_cap);
    fails += TEST_EQ_U16(16, (uint16_t)registers_cap);
    fails += TEST_EQ_U16(MODBUS_RESULT_MAX_REGISTERS, (uint16_t)registers_cap);
    return fails;
}

/* 队列内存预算: request/result 不得超过各自字节预算, 防止队列对象膨胀。 */
static int test_struct_size_budget(void)
{
    int fails = 0;
    fails += TEST_EQ_U16(1, (uint16_t)(sizeof(ModbusRequest_t) <= MODBUS_REQUEST_MAX_BYTES));
    fails += TEST_EQ_U16(1, (uint16_t)(sizeof(ModbusResult_t) <= MODBUS_RESULT_MAX_BYTES));
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_operation_enum_values();
    fails += test_status_enum_values();
    fails += test_inline_capacities();
    fails += test_struct_size_budget();

    if (fails == 0)
    {
        printf("All Modbus service contract tests passed\n");
    }
    else
    {
        printf("%d Modbus service contract assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
