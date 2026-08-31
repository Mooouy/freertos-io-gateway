#include "modbus_rtu.h"
#include "test_assert.h"

#include <stdint.h>
#include <stdio.h>

/* CRC16: 计划给定向量 {01,03,02,09,00,01} 期望 0xB055。 */
static int test_crc16_known_vector(void)
{
    const uint8_t frame[] = {0x01, 0x03, 0x02, 0x09, 0x00, 0x01};
    return TEST_EQ_U16(0xB055, Modbus_RTU_CRC16(frame, (uint16_t)sizeof frame));
}

/* 03 请求: 读从站 1 寄存器 0x0209 共 1 个, 期望 {01,03,02,09,00,01,55,B0}。 */
static int test_build_read03_request(void)
{
    const uint8_t expected[8] = {0x01, 0x03, 0x02, 0x09, 0x00, 0x01, 0x55, 0xB0};
    uint8_t out[8] = {0};
    uint16_t len;
    int fails = 0;

    len = Modbus_RTU_BuildRead03Request(0x01, 0x0209, 0x0001, out, (uint16_t)sizeof out);
    fails += TEST_EQ_U16(8, len);
    for (uint16_t i = 0; i < 8U; i++)
    {
        fails += TEST_EQ_U16(expected[i], out[i]);
    }
    return fails;
}

/* 03 正常响应: 01 03 02 12 34 CRC, 解析出寄存器 0x1234。 */
static int test_parse_read03_ok(void)
{
    uint8_t frame[7] = {0x01, 0x03, 0x02, 0x12, 0x34, 0x00, 0x00};
    uint8_t buf[2] = {0};
    ModbusRtuRead03Context_t ctx = {0};
    ModbusRtuRead03Result_t result = {0};
    ModbusRtuStatus_t st;
    uint16_t crc;
    int fails = 0;

    crc = Modbus_RTU_CRC16(frame, 5);
    frame[5] = (uint8_t)(crc & 0xFFU);
    frame[6] = (uint8_t)(crc >> 8);

    ctx.slave_addr = 0x01;
    ctx.reg_count = 0x0001;
    ctx.user_buf = buf;
    ctx.user_buf_len = (uint16_t)sizeof buf;

    st = Modbus_RTU_ParseRead03Response(frame, (uint16_t)sizeof frame, &ctx, &result);
    fails += TEST_EQ_U16(MODBUS_RTU_OK, st);
    fails += TEST_EQ_U16(2, result.byte_count);
    fails += TEST_EQ_U16(0x1234, ((uint16_t)buf[0] << 8) | buf[1]);
    return fails;
}

/* 构造 5 字节异常帧: slave/func/code + CRC, 返回长度 5。 */
static uint16_t make_exception_frame(uint8_t *frame, uint8_t slave,
                                     uint8_t func, uint8_t code)
{
    uint16_t crc;
    frame[0] = slave;
    frame[1] = func;
    frame[2] = code;
    crc = Modbus_RTU_CRC16(frame, 3);
    frame[3] = (uint8_t)(crc & 0xFFU);
    frame[4] = (uint8_t)(crc >> 8);
    return 5U;
}

/* 等 03 时收到 0x90(16 功能异常) 应判为 INVALID, 而非 03 异常。 */
static int test_parse_read03_wrong_func_exception(void)
{
    uint8_t frame[5] = {0};
    uint8_t buf[2] = {0};
    ModbusRtuRead03Context_t ctx = {0};
    ModbusRtuRead03Result_t result = {0};
    uint16_t len;

    len = make_exception_frame(frame, 0x01, 0x90, 0x02);
    ctx.slave_addr = 0x01;
    ctx.reg_count = 0x0001;
    ctx.user_buf = buf;
    ctx.user_buf_len = (uint16_t)sizeof buf;

    return TEST_EQ_U16(MODBUS_RTU_INVALID_RESPONSE,
                       Modbus_RTU_ParseRead03Response(frame, len, &ctx, &result));
}

/* 异常功能码正确(0x83)但帧长不是 5 时, 应判为 INVALID。 */
static int test_parse_read03_exception_bad_length(void)
{
    uint8_t frame[7] = {0x01, 0x83, 0x02, 0x00, 0x00, 0x00, 0x00};
    uint8_t buf[2] = {0};
    ModbusRtuRead03Context_t ctx = {0};
    ModbusRtuRead03Result_t result = {0};
    uint16_t crc;

    crc = Modbus_RTU_CRC16(frame, 5);
    frame[5] = (uint8_t)(crc & 0xFFU);
    frame[6] = (uint8_t)(crc >> 8);

    ctx.slave_addr = 0x01;
    ctx.reg_count = 0x0001;
    ctx.user_buf = buf;
    ctx.user_buf_len = (uint16_t)sizeof buf;

    return TEST_EQ_U16(MODBUS_RTU_INVALID_RESPONSE,
                       Modbus_RTU_ParseRead03Response(frame, (uint16_t)sizeof frame,
                                                      &ctx, &result));
}

/* 合法 03 异常帧(0x83 + 异常码, 长度 5): 返回 EXCEPTION 且回填异常码。 */
static int test_parse_read03_valid_exception(void)
{
    uint8_t frame[5] = {0};
    uint8_t buf[2] = {0};
    ModbusRtuRead03Context_t ctx = {0};
    ModbusRtuRead03Result_t result = {0};
    uint16_t len;
    int fails = 0;

    len = make_exception_frame(frame, 0x01, 0x83, 0x02);
    ctx.slave_addr = 0x01;
    ctx.reg_count = 0x0001;
    ctx.user_buf = buf;
    ctx.user_buf_len = (uint16_t)sizeof buf;

    fails += TEST_EQ_U16(MODBUS_RTU_EXCEPTION,
                         Modbus_RTU_ParseRead03Response(frame, len, &ctx, &result));
    fails += TEST_EQ_U16(0x02, result.exception_code);
    return fails;
}

/* 等 06 时收到 0x83(03 功能异常) 应判为 INVALID。 */
static int test_parse_write06_wrong_func_exception(void)
{
    uint8_t frame[5] = {0};
    ModbusRtuWrite06Context_t ctx = {0};
    uint16_t len;

    len = make_exception_frame(frame, 0x01, 0x83, 0x02);
    ctx.slave_addr = 0x01;
    ctx.reg_addr = 0x0001;
    ctx.value = 0x0001;

    return TEST_EQ_U16(MODBUS_RTU_INVALID_RESPONSE,
                       Modbus_RTU_ParseWrite06Response(frame, len, &ctx));
}

/* 等 16 时收到 0x83(03 功能异常) 应判为 INVALID。 */
static int test_parse_write16_wrong_func_exception(void)
{
    uint8_t frame[5] = {0};
    ModbusRtuWrite16Context_t ctx = {0};
    uint16_t len;

    len = make_exception_frame(frame, 0x01, 0x83, 0x02);
    ctx.slave_addr = 0x01;
    ctx.start_addr = 0x0001;
    ctx.reg_count = 0x0001;

    return TEST_EQ_U16(MODBUS_RTU_INVALID_RESPONSE,
                       Modbus_RTU_ParseWrite16Response(frame, len, &ctx));
}

/* 路由匹配: 地址 + 功能码(忽略异常位)命中当前请求, 才算是我们的响应。 */
static int test_frame_matches_read03(void)
{
    const uint8_t frame[] = {0x01, 0x03, 0x02, 0x12, 0x34, 0x00, 0x00};
    return TEST_EQ_U16(1, Modbus_RTU_FrameMatches(frame, (uint16_t)sizeof frame, 0x01,
                                                  MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
}

/* 异常帧(func | 0x80)仍应路由到对应功能码。 */
static int test_frame_matches_exception(void)
{
    const uint8_t frame[] = {0x01, 0x83, 0x02, 0x00, 0x00};
    return TEST_EQ_U16(1, Modbus_RTU_FrameMatches(frame, (uint16_t)sizeof frame, 0x01,
                                                  MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
}

/* 从站地址不符: 不是发给我们的帧, 应忽略(P2: 不能结束当前请求)。 */
static int test_frame_matches_wrong_slave(void)
{
    const uint8_t frame[] = {0x02, 0x03, 0x02, 0x12, 0x34, 0x00, 0x00};
    return TEST_EQ_U16(0, Modbus_RTU_FrameMatches(frame, (uint16_t)sizeof frame, 0x01,
                                                  MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
}

/* 功能码不符: 等 03 却来了 06, 应忽略。 */
static int test_frame_matches_wrong_func(void)
{
    const uint8_t frame[] = {0x01, 0x06, 0x02, 0x09, 0x00, 0x01, 0x00, 0x00};
    return TEST_EQ_U16(0, Modbus_RTU_FrameMatches(frame, (uint16_t)sizeof frame, 0x01,
                                                  MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
}

/* 帧太短(<2)或空指针无法路由, 一律不匹配。 */
static int test_frame_matches_too_short(void)
{
    const uint8_t frame[] = {0x01};
    int fails = 0;
    fails += TEST_EQ_U16(0, Modbus_RTU_FrameMatches(frame, 1U, 0x01,
                                                    MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
    fails += TEST_EQ_U16(0, Modbus_RTU_FrameMatches(NULL, 0U, 0x01,
                                                    MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS));
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_crc16_known_vector();
    fails += test_build_read03_request();
    fails += test_parse_read03_ok();
    fails += test_parse_read03_wrong_func_exception();
    fails += test_parse_read03_exception_bad_length();
    fails += test_parse_read03_valid_exception();
    fails += test_parse_write06_wrong_func_exception();
    fails += test_parse_write16_wrong_func_exception();
    fails += test_frame_matches_read03();
    fails += test_frame_matches_exception();
    fails += test_frame_matches_wrong_slave();
    fails += test_frame_matches_wrong_func();
    fails += test_frame_matches_too_short();

    if (fails == 0)
    {
        printf("All Modbus RTU tests passed\n");
    }
    else
    {
        printf("%d Modbus RTU assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
