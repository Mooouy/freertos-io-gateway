#include "bsm0404.h"

#include <stddef.h>

void Bsm_DecodeSnapshot(const uint16_t *counter_regs,
                        const uint16_t *ao_regs,
                        const uint16_t *ai_regs,
                        uint16_t relay_bitmap,
                        uint16_t di_bitmap,
                        BsmSnapshot_t *snapshot)
{
    if ((counter_regs == NULL) || (ao_regs == NULL) || (ai_regs == NULL) ||
        (snapshot == NULL))
    {
        return;
    }

    snapshot->di_bits = (uint8_t)(di_bitmap & BSM_CHANNEL_MASK);
    snapshot->relay_bits = (uint8_t)(relay_bitmap & BSM_CHANNEL_MASK);

    /* 每路计数器占 2 个寄存器, 低字在前, 拼成 32 位。 */
    for (uint8_t i = 0; i < BSM_CHANNEL_COUNT; i++)
    {
        uint16_t lo = counter_regs[i * 2U];
        uint16_t hi = counter_regs[(i * 2U) + 1U];
        snapshot->di_count[i] = (uint32_t)lo | ((uint32_t)hi << 16);
    }

    for (uint8_t i = 0; i < BSM_AI_COUNT; i++)
    {
        snapshot->ai_raw[i] = ai_regs[i];
    }

    for (uint8_t i = 0; i < BSM_AO_COUNT; i++)
    {
        snapshot->ao_raw[i] = ao_regs[i];
    }
}

bool Bsm_RawToMillivolts(uint16_t raw, uint16_t *millivolts)
{
    if ((millivolts == NULL) || (raw > BSM_ANALOG_RAW_MAX))
    {
        return false;
    }

    *millivolts = (uint16_t)(((uint32_t)raw * BSM_VOLTAGE_MAX_MV) / BSM_ANALOG_RAW_MAX);
    return true;
}

bool Bsm_RawToMicroamps(uint16_t raw, uint16_t *microamps)
{
    if ((microamps == NULL) || (raw > BSM_ANALOG_RAW_MAX))
    {
        return false;
    }

    *microamps = (uint16_t)(((uint32_t)raw * BSM_CURRENT_MAX_UA) / BSM_ANALOG_RAW_MAX);
    return true;
}

bool Bsm_MillivoltsToRaw(uint16_t millivolts, uint16_t *raw)
{
    if ((raw == NULL) || (millivolts > BSM_VOLTAGE_MAX_MV))
    {
        return false;
    }

    *raw = (uint16_t)(((uint32_t)millivolts * BSM_ANALOG_RAW_MAX) / BSM_VOLTAGE_MAX_MV);
    return true;
}

bool Bsm_MicroampsToRaw(uint16_t microamps, uint16_t *raw)
{
    if ((raw == NULL) || (microamps > BSM_CURRENT_MAX_UA))
    {
        return false;
    }

    *raw = (uint16_t)(((uint32_t)microamps * BSM_ANALOG_RAW_MAX) / BSM_CURRENT_MAX_UA);
    return true;
}
