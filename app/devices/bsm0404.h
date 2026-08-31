#ifndef BSM0404_H
#define BSM0404_H

/*
 * BSM-0404RB-2AD-2DA 设备驱动(纯逻辑层)。
 *
 * 隐藏寄存器地址、I/O 位图与模拟量换算; 不依赖 HAL/FreeRTOS, 可主机测试。
 * 上层(modbus_service 调用方)按本文件的寄存器地址读写, 再用这里的函数解码/换算,
 * 不在业务层散落硬编码寄存器地址。
 *
 * 模块: 4 路 DI(X1-X4, 含 32 位计数器), 4 路继电器(Y1-Y4),
 *       2 路模拟量输入(AD1/AD2), 2 路模拟量输出(DA1 0-10V / DA2 0-20mA)。
 * 模拟量协议范围 0-4000(12 位)。默认 Modbus 站号 1。
 */

#include <stdbool.h>
#include <stdint.h>

/* 4x 保持寄存器地址(见设计规格 §5.1)。 */
#define BSM_REG_DI_COUNTER_BASE 400U /* 400-407: X1-X4 32 位计数器(每路 低字,高字) */
#define BSM_REG_AO_BASE 500U         /* 500-501: DA1, DA2 模拟量输出 */
#define BSM_REG_AI_BASE 520U         /* 520-521: AD1, AD2 模拟量输入 */
#define BSM_REG_RELAY_BITMAP 560U    /* 560: Y1-Y4 继电器位图 */
#define BSM_REG_DI_BITMAP 630U       /* 630: X1-X4 数字量输入位图 */

#define BSM_CHANNEL_COUNT 4U         /* DI / 继电器通道数 */
#define BSM_CHANNEL_MASK 0x0FU       /* 低 4 位通道掩码 */
#define BSM_AI_COUNT 2U              /* 模拟量输入通道数 */
#define BSM_AO_COUNT 2U              /* 模拟量输出通道数 */
#define BSM_COUNTER_REG_COUNT (BSM_CHANNEL_COUNT * 2U) /* 计数器寄存器总数(8) */

/* 模拟量量程: 原始值 0-4000 对应 0-10V / 0-20mA。 */
#define BSM_ANALOG_RAW_MAX 4000U
#define BSM_VOLTAGE_MAX_MV 10000U
#define BSM_CURRENT_MAX_UA 20000U

/* 一次完整 I/O 快照(已解码的类型化视图)。 */
typedef struct
{
    uint8_t di_bits;                      /* X1-X4 数字量输入位图(低 4 位) */
    uint8_t relay_bits;                   /* Y1-Y4 继电器位图(低 4 位) */
    uint32_t di_count[BSM_CHANNEL_COUNT]; /* X1-X4 32 位累计计数 */
    uint16_t ai_raw[BSM_AI_COUNT];        /* AD1, AD2 原始值 0-4000 */
    uint16_t ao_raw[BSM_AO_COUNT];        /* DA1, DA2 原始值 0-4000 */
} BsmSnapshot_t;

/*
 * 把读到的各寄存器块解码为类型化快照。
 * counter_regs: 8 个寄存器(400-407, 每路 低字/高字);
 * ao_regs/ai_regs: 各 2 个寄存器(500-501 / 520-521);
 * relay_bitmap/di_bitmap: 560 / 630 的位图原始值。
 * 任一数组指针或 snapshot 为空则不修改 snapshot。
 */
void Bsm_DecodeSnapshot(const uint16_t *counter_regs,
                        const uint16_t *ao_regs,
                        const uint16_t *ai_regs,
                        uint16_t relay_bitmap,
                        uint16_t di_bitmap,
                        BsmSnapshot_t *snapshot);

/*
 * 模拟量原始值<->物理量换算(32 位运算, 整数截断)。
 * 入参越界(原始值>4000 / 电压>10000mV / 电流>20000uA)或出参指针为空时返回 false
 * 且不写出参; 成功返回 true 并写出参。
 */
bool Bsm_RawToMillivolts(uint16_t raw, uint16_t *millivolts);
bool Bsm_RawToMicroamps(uint16_t raw, uint16_t *microamps);
bool Bsm_MillivoltsToRaw(uint16_t millivolts, uint16_t *raw);
bool Bsm_MicroampsToRaw(uint16_t microamps, uint16_t *raw);

#endif /* BSM0404_H */
