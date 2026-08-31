#include "modbus_rtu.h"

/*
 * 标准 Modbus RTU CRC16，使用多项式 0xA001。
 * 返回值写入 RTU 帧时低字节在前，高字节在后。
 */
uint16_t Modbus_RTU_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/*
 * 路由判断: 帧的从站地址与功能码(忽略异常位 0x80)是否匹配指定请求。
 * 只看 frame[0] 与 frame[1], 不校验 CRC, 供上层在等待响应时过滤不相关帧:
 * 不匹配的帧应被忽略并继续等待, 而不是结束当前请求。
 * 匹配返回 1, 否则返回 0。
 */
int Modbus_RTU_FrameMatches(const uint8_t *frame,
                            uint16_t frame_len,
                            uint8_t slave_addr,
                            uint8_t func_code)
{
    if ((frame == 0) || (frame_len < 2U))
    {
        return 0;
    }

    if (frame[0] != slave_addr)
    {
        return 0;
    }

    if ((uint8_t)(frame[1] & 0x7FU) != func_code)
    {
        return 0;
    }

    return 1;
}

uint16_t Modbus_RTU_BuildRead03Request(uint8_t slave_addr,
                                       uint16_t start_addr,
                                       uint16_t num_regs,
                                       uint8_t *out_frame,
                                       uint16_t out_size)
{
    uint16_t crc;

    if ((out_frame == 0) || (out_size < MODBUS_RTU_READ03_REQ_LEN))
    {
        return 0;
    }

    out_frame[0] = slave_addr;
    out_frame[1] = MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS;
    out_frame[2] = (uint8_t)(start_addr >> 8);
    out_frame[3] = (uint8_t)(start_addr & 0xFFU);
    out_frame[4] = (uint8_t)(num_regs >> 8);
    out_frame[5] = (uint8_t)(num_regs & 0xFFU);

    crc = Modbus_RTU_CRC16(out_frame, 6);
    out_frame[6] = (uint8_t)(crc & 0xFFU);
    out_frame[7] = (uint8_t)(crc >> 8);

    return MODBUS_RTU_READ03_REQ_LEN;
}

/*
 * 构造 06 写单寄存器请求：
 * 从站地址 + 0x06 + 寄存器地址 + 写入值 + CRC。
 * 06 的正常响应会原样回显这 8 个字节。
 */
uint16_t Modbus_RTU_BuildWrite06Request(uint8_t slave_addr,
                                        uint16_t reg_addr,
                                        uint16_t value,
                                        uint8_t *out_frame,
                                        uint16_t out_size)
{
    uint16_t crc;

    if ((out_frame == 0) || (out_size < MODBUS_RTU_WRITE06_REQ_LEN))
    {
        return 0;
    }

    out_frame[0] = slave_addr;
    out_frame[1] = MODBUS_RTU_FUNC_WRITE_SINGLE_REGISTER;
    out_frame[2] = (uint8_t)(reg_addr >> 8);
    out_frame[3] = (uint8_t)(reg_addr & 0xFFU);
    out_frame[4] = (uint8_t)(value >> 8);
    out_frame[5] = (uint8_t)(value & 0xFFU);

    crc = Modbus_RTU_CRC16(out_frame, 6);
    out_frame[6] = (uint8_t)(crc & 0xFFU);
    out_frame[7] = (uint8_t)(crc >> 8);

    return MODBUS_RTU_WRITE06_REQ_LEN;
}

uint16_t Modbus_RTU_BuildWrite16Request(uint8_t slave_addr,
                                        uint16_t start_addr,
                                        uint16_t num_regs,
                                        const uint16_t *values,
                                        uint8_t *out_frame,
                                        uint16_t out_size)
{
    uint16_t crc;
    uint16_t data_len;
    uint16_t total_len;
    uint8_t byte_count;

    if ((values == 0) || (out_frame == 0) || (num_regs == 0U) ||
        (num_regs > MODBUS_RTU_WRITE16_MAX_REGS))
    {
        return 0;
    }

    byte_count = (uint8_t)(num_regs * 2U);
    data_len = (uint16_t)(7U + byte_count);
    total_len = (uint16_t)(data_len + 2U);
    if (out_size < total_len)
    {
        return 0;
    }

    out_frame[0] = slave_addr;
    out_frame[1] = MODBUS_RTU_FUNC_WRITE_MULTIPLE_REGISTERS;
    out_frame[2] = (uint8_t)(start_addr >> 8);
    out_frame[3] = (uint8_t)(start_addr & 0xFFU);
    out_frame[4] = (uint8_t)(num_regs >> 8);
    out_frame[5] = (uint8_t)(num_regs & 0xFFU);
    out_frame[6] = byte_count;

    for (uint16_t i = 0; i < num_regs; i++)
    {
        out_frame[7U + (i * 2U)] = (uint8_t)(values[i] >> 8);
        out_frame[8U + (i * 2U)] = (uint8_t)(values[i] & 0xFFU);
    }

    crc = Modbus_RTU_CRC16(out_frame, data_len);
    out_frame[data_len] = (uint8_t)(crc & 0xFFU);
    out_frame[data_len + 1U] = (uint8_t)(crc >> 8);

    return total_len;
}

/*
 * 校验 03 响应，并只把数据区复制到 user_buf。
 * 调用者提供期望的从站地址和寄存器数量，这样旧帧或不相关帧不会被误认为
 * 当前请求成功。
 */
ModbusRtuStatus_t Modbus_RTU_ParseRead03Response(const uint8_t *frame,
                                                 uint16_t frame_len,
                                                 const ModbusRtuRead03Context_t *ctx,
                                                 ModbusRtuRead03Result_t *result)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    uint8_t byte_count;

    if ((frame == 0) || (ctx == 0) || (result == 0) || (frame_len < 5U))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    crc_calculated = Modbus_RTU_CRC16(frame, (uint16_t)(frame_len - 2U));
    crc_received = (uint16_t)frame[frame_len - 2U] |
                   ((uint16_t)frame[frame_len - 1U] << 8);
    if (crc_calculated != crc_received)
    {
        return MODBUS_RTU_CRC_ERROR;
    }

    if (frame[0] != ctx->slave_addr)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if ((frame[1] & 0x80U) != 0U)
    {
        /* 异常帧必须是“当前功能码 | 0x80”且长度恰为 5, 否则是不相关帧。 */
        if (((frame[1] & 0x7FU) != MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS) ||
            (frame_len != 5U))
        {
            return MODBUS_RTU_INVALID_RESPONSE;
        }
        result->exception_code = frame[2];
        return MODBUS_RTU_EXCEPTION;
    }

    if (frame[1] != MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    byte_count = frame[2];
    if (byte_count != (ctx->reg_count * 2U))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if (frame_len != (uint16_t)(3U + byte_count + 2U))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if ((ctx->user_buf == 0) || (byte_count > ctx->user_buf_len))
    {
        return MODBUS_RTU_BUFFER_TOO_SMALL;
    }

    for (uint8_t i = 0; i < byte_count; i++)
    {
        ctx->user_buf[i] = frame[3U + i];
    }

    result->byte_count = byte_count;
    return MODBUS_RTU_OK;
}

/*
 * 校验 06 响应。Modbus RTU 中，06 成功响应会回显从站地址、功能码、
 * 寄存器地址和写入值。
 */
ModbusRtuStatus_t Modbus_RTU_ParseWrite06Response(const uint8_t *frame,
                                                  uint16_t frame_len,
                                                  const ModbusRtuWrite06Context_t *ctx)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    uint16_t reg_addr;
    uint16_t value;

    if ((frame == 0) || (ctx == 0) || (frame_len < 5U))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    crc_calculated = Modbus_RTU_CRC16(frame, (uint16_t)(frame_len - 2U));
    crc_received = (uint16_t)frame[frame_len - 2U] |
                   ((uint16_t)frame[frame_len - 1U] << 8);
    if (crc_calculated != crc_received)
    {
        return MODBUS_RTU_CRC_ERROR;
    }

    if (frame[0] != ctx->slave_addr)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if ((frame[1] & 0x80U) != 0U)
    {
        /* 异常帧必须是“当前功能码 | 0x80”且长度恰为 5, 否则是不相关帧。 */
        if (((frame[1] & 0x7FU) != MODBUS_RTU_FUNC_WRITE_SINGLE_REGISTER) ||
            (frame_len != 5U))
        {
            return MODBUS_RTU_INVALID_RESPONSE;
        }
        return MODBUS_RTU_EXCEPTION;
    }

    if (frame[1] != MODBUS_RTU_FUNC_WRITE_SINGLE_REGISTER)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if (frame_len != MODBUS_RTU_WRITE06_REQ_LEN)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    reg_addr = ((uint16_t)frame[2] << 8) | frame[3];
    value = ((uint16_t)frame[4] << 8) | frame[5];

    if ((reg_addr != ctx->reg_addr) || (value != ctx->value))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    return MODBUS_RTU_OK;
}

/*
 * 校验 16 响应。16 成功响应只回显起始地址和写入的寄存器数量，
 * 不会回显写入的数据区。
 */
ModbusRtuStatus_t Modbus_RTU_ParseWrite16Response(const uint8_t *frame,
                                                  uint16_t frame_len,
                                                  const ModbusRtuWrite16Context_t *ctx)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    uint16_t start_addr;
    uint16_t reg_count;

    if ((frame == 0) || (ctx == 0) || (frame_len < 5U))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    crc_calculated = Modbus_RTU_CRC16(frame, (uint16_t)(frame_len - 2U));
    crc_received = (uint16_t)frame[frame_len - 2U] |
                   ((uint16_t)frame[frame_len - 1U] << 8);
    if (crc_calculated != crc_received)
    {
        return MODBUS_RTU_CRC_ERROR;
    }

    if (frame[0] != ctx->slave_addr)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if ((frame[1] & 0x80U) != 0U)
    {
        /* 异常帧必须是“当前功能码 | 0x80”且长度恰为 5, 否则是不相关帧。 */
        if (((frame[1] & 0x7FU) != MODBUS_RTU_FUNC_WRITE_MULTIPLE_REGISTERS) ||
            (frame_len != 5U))
        {
            return MODBUS_RTU_INVALID_RESPONSE;
        }
        return MODBUS_RTU_EXCEPTION;
    }

    if (frame[1] != MODBUS_RTU_FUNC_WRITE_MULTIPLE_REGISTERS)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    if (frame_len != MODBUS_RTU_WRITE16_RESP_LEN)
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    start_addr = ((uint16_t)frame[2] << 8) | frame[3];
    reg_count = ((uint16_t)frame[4] << 8) | frame[5];

    if ((start_addr != ctx->start_addr) || (reg_count != ctx->reg_count))
    {
        return MODBUS_RTU_INVALID_RESPONSE;
    }

    return MODBUS_RTU_OK;
}
