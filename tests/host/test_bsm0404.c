/*
 * BSM-0404RB-2AD-2DA 设备驱动测试(纯逻辑)。
 *
 * 覆盖: DI/继电器位图掩码、32 位计数器低/高字拼装、AI/AO 原始值搬运,
 * 以及模拟量原始值<->物理量(mV/uA)的换算与量程拒绝。不依赖 HAL/FreeRTOS。
 */

#include "bsm0404.h"
#include "test_assert.h"

#include <stdint.h>
#include <stdio.h>

/* DI 位图只保留 4 个通道位: 0xFFF5 & 0x0F = 0x05。 */
static int test_decode_di_bits_mask(void)
{
    uint16_t counters[8] = {0};
    uint16_t ao[2] = {0};
    uint16_t ai[2] = {0};
    BsmSnapshot_t snap = {0};

    Bsm_DecodeSnapshot(counters, ao, ai, 0x0000U, 0xFFF5U, &snap);
    return TEST_EQ_U16(0x05U, snap.di_bits);
}

/* 继电器位图同样只保留 4 个通道位。 */
static int test_decode_relay_bits_mask(void)
{
    uint16_t counters[8] = {0};
    uint16_t ao[2] = {0};
    uint16_t ai[2] = {0};
    BsmSnapshot_t snap = {0};

    Bsm_DecodeSnapshot(counters, ao, ai, 0xFFF5U, 0x0000U, &snap);
    return TEST_EQ_U16(0x05U, snap.relay_bits);
}

/* 计数器: 每路 2 个寄存器, 低字在前, 拼成 32 位。 */
static int test_decode_counters(void)
{
    uint16_t counters[8] = {0x0001U, 0x0002U, 0x0000U, 0x0000U,
                            0xFFFFU, 0x0000U, 0x3412U, 0x7856U};
    uint16_t ao[2] = {0};
    uint16_t ai[2] = {0};
    BsmSnapshot_t snap = {0};
    int fails = 0;

    Bsm_DecodeSnapshot(counters, ao, ai, 0x0000U, 0x0000U, &snap);
    fails += TEST_EQ_U32(0x00020001U, snap.di_count[0]);
    fails += TEST_EQ_U32(0x00000000U, snap.di_count[1]);
    fails += TEST_EQ_U32(0x0000FFFFU, snap.di_count[2]);
    fails += TEST_EQ_U32(0x78563412U, snap.di_count[3]);
    return fails;
}

/* AI/AO 原始值按通道顺序搬运到快照。 */
static int test_decode_ai_ao(void)
{
    uint16_t counters[8] = {0};
    uint16_t ao[2] = {500U, 1000U};
    uint16_t ai[2] = {2000U, 4000U};
    BsmSnapshot_t snap = {0};
    int fails = 0;

    Bsm_DecodeSnapshot(counters, ao, ai, 0x0000U, 0x0000U, &snap);
    fails += TEST_EQ_U16(2000U, snap.ai_raw[0]);
    fails += TEST_EQ_U16(4000U, snap.ai_raw[1]);
    fails += TEST_EQ_U16(500U, snap.ao_raw[0]);
    fails += TEST_EQ_U16(1000U, snap.ao_raw[1]);
    return fails;
}

/* 0-10 V: mv = raw * 10000 / 4000。 */
static int test_raw_to_millivolts(void)
{
    uint16_t mv = 0;
    int fails = 0;

    fails += TEST_EQ_U16(1, (uint16_t)Bsm_RawToMillivolts(2000U, &mv));
    fails += TEST_EQ_U16(5000U, mv);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_RawToMillivolts(4000U, &mv));
    fails += TEST_EQ_U16(10000U, mv);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_RawToMillivolts(0U, &mv));
    fails += TEST_EQ_U16(0U, mv);
    return fails;
}

/* 0-20 mA: ua = raw * 20000 / 4000。 */
static int test_raw_to_microamps(void)
{
    uint16_t ua = 0;
    int fails = 0;

    fails += TEST_EQ_U16(1, (uint16_t)Bsm_RawToMicroamps(2000U, &ua));
    fails += TEST_EQ_U16(10000U, ua);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_RawToMicroamps(4000U, &ua));
    fails += TEST_EQ_U16(20000U, ua);
    return fails;
}

/* 原始值超过量程 4000 应被拒绝。 */
static int test_raw_out_of_range_rejected(void)
{
    uint16_t mv = 0;
    uint16_t ua = 0;
    int fails = 0;

    fails += TEST_EQ_U16(0, (uint16_t)Bsm_RawToMillivolts(4001U, &mv));
    fails += TEST_EQ_U16(0, (uint16_t)Bsm_RawToMicroamps(4001U, &ua));
    return fails;
}

/* 物理量反算原始值: raw = mv * 4000 / 10000 = ua * 4000 / 20000。 */
static int test_phys_to_raw(void)
{
    uint16_t raw = 0;
    int fails = 0;

    fails += TEST_EQ_U16(1, (uint16_t)Bsm_MillivoltsToRaw(5000U, &raw));
    fails += TEST_EQ_U16(2000U, raw);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_MillivoltsToRaw(10000U, &raw));
    fails += TEST_EQ_U16(4000U, raw);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_MicroampsToRaw(10000U, &raw));
    fails += TEST_EQ_U16(2000U, raw);
    fails += TEST_EQ_U16(1, (uint16_t)Bsm_MicroampsToRaw(20000U, &raw));
    fails += TEST_EQ_U16(4000U, raw);
    return fails;
}

/* 物理量超过量程应被拒绝。 */
static int test_phys_out_of_range_rejected(void)
{
    uint16_t raw = 0;
    int fails = 0;

    fails += TEST_EQ_U16(0, (uint16_t)Bsm_MillivoltsToRaw(10001U, &raw));
    fails += TEST_EQ_U16(0, (uint16_t)Bsm_MicroampsToRaw(20001U, &raw));
    return fails;
}

int main(void)
{
    int fails = 0;

    fails += test_decode_di_bits_mask();
    fails += test_decode_relay_bits_mask();
    fails += test_decode_counters();
    fails += test_decode_ai_ao();
    fails += test_raw_to_millivolts();
    fails += test_raw_to_microamps();
    fails += test_raw_out_of_range_rejected();
    fails += test_phys_to_raw();
    fails += test_phys_out_of_range_rejected();

    if (fails == 0)
    {
        printf("All BSM0404 tests passed\n");
    }
    else
    {
        printf("%d BSM0404 assertion(s) failed\n", fails);
    }
    return (fails == 0) ? 0 : 1;
}
