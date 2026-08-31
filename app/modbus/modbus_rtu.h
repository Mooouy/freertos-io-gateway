#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/*
 * Modbus RTU 协议层辅助函数。
 *
 * 本层刻意不依赖 HAL 和 FreeRTOS，只负责构造请求字节、校验 RTU 响应帧、
 * 计算 CRC，并把已经校验通过的数据复制到调用者提供的缓冲区。
 */

#include <stdint.h>

#define MODBUS_RTU_READ03_REQ_LEN 8U
#define MODBUS_RTU_WRITE06_REQ_LEN 8U
#define MODBUS_RTU_WRITE16_RESP_LEN 8U
#define MODBUS_RTU_WRITE16_MAX_REGS 123U
#define MODBUS_RTU_FUNC_READ_HOLDING_REGISTERS 0x03U
#define MODBUS_RTU_FUNC_WRITE_SINGLE_REGISTER 0x06U
#define MODBUS_RTU_FUNC_WRITE_MULTIPLE_REGISTERS 0x10U

typedef enum
{
    MODBUS_RTU_OK = 0,
    MODBUS_RTU_CRC_ERROR,
    MODBUS_RTU_EXCEPTION,
    MODBUS_RTU_INVALID_RESPONSE,
    MODBUS_RTU_BUFFER_TOO_SMALL,
} ModbusRtuStatus_t;

/* 校验 03 响应时需要的“当前请求”信息。 */
typedef struct
{
    uint8_t slave_addr;
    uint16_t reg_count;
    uint8_t *user_buf;
    uint16_t user_buf_len;
} ModbusRtuRead03Context_t;

typedef struct
{
    uint8_t byte_count;
    uint8_t exception_code; /* 仅当返回值为 MODBUS_RTU_EXCEPTION 时有效 */
} ModbusRtuRead03Result_t;

/* 校验 06 响应时需要的回显字段。 */
typedef struct
{
    uint8_t slave_addr;
    uint16_t reg_addr;
    uint16_t value;
} ModbusRtuWrite06Context_t;

/* 校验 16 响应时需要的回显字段。 */
typedef struct
{
    uint8_t slave_addr;
    uint16_t start_addr;
    uint16_t reg_count;
} ModbusRtuWrite16Context_t;

uint16_t Modbus_RTU_CRC16(const uint8_t *data, uint16_t len);
int Modbus_RTU_FrameMatches(const uint8_t *frame,
                            uint16_t frame_len,
                            uint8_t slave_addr,
                            uint8_t func_code);
uint16_t Modbus_RTU_BuildRead03Request(uint8_t slave_addr,
                                       uint16_t start_addr,
                                       uint16_t num_regs,
                                       uint8_t *out_frame,
                                       uint16_t out_size);
uint16_t Modbus_RTU_BuildWrite06Request(uint8_t slave_addr,
                                        uint16_t reg_addr,
                                        uint16_t value,
                                        uint8_t *out_frame,
                                        uint16_t out_size);
uint16_t Modbus_RTU_BuildWrite16Request(uint8_t slave_addr,
                                        uint16_t start_addr,
                                        uint16_t num_regs,
                                        const uint16_t *values,
                                        uint8_t *out_frame,
                                        uint16_t out_size);
ModbusRtuStatus_t Modbus_RTU_ParseRead03Response(const uint8_t *frame,
                                                 uint16_t frame_len,
                                                 const ModbusRtuRead03Context_t *ctx,
                                                 ModbusRtuRead03Result_t *result);
ModbusRtuStatus_t Modbus_RTU_ParseWrite06Response(const uint8_t *frame,
                                                  uint16_t frame_len,
                                                  const ModbusRtuWrite06Context_t *ctx);
ModbusRtuStatus_t Modbus_RTU_ParseWrite16Response(const uint8_t *frame,
                                                  uint16_t frame_len,
                                                  const ModbusRtuWrite16Context_t *ctx);

#endif /* MODBUS_RTU_H */
