#include "crc16.h"

uint16_t CRC16_Init(void)
{
    return 0xFFFF;
}

uint16_t CRC16_Update(uint16_t crc, uint8_t data)
{
    uint8_t ucCRCHi = (uint8_t)(crc >> 8);
    uint8_t ucCRCLo = (uint8_t)(crc & 0xFF);

    uint8_t iIndex = ucCRCLo ^ data;

    ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex];
    ucCRCHi = aucCRCLo[iIndex];

    return ((uint16_t)ucCRCHi << 8) | ucCRCLo;
}

uint16_t CRC16_Calculate(const uint8_t *data,
                         uint16_t len)
{
    uint16_t crc = CRC16_Init();

    for (uint16_t i = 0; i < len; i++)
    {
        crc = CRC16_Update(crc, data[i]);
    }

    return crc;
}

uint16_t CRC16_Append(uint8_t *frame, uint16_t payload_len)
{
    if (frame == NULL)
    {
        return 0;
    }

    uint16_t crc = CRC16_Calculate(frame, payload_len);

    frame[payload_len]     = (uint8_t)(crc & 0xFF);        // CRC Low
    frame[payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF); // CRC High

    return crc;
}

uint8_t CRC16_Verify(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 3)
    {
        return 0;
    }

    uint16_t calculated_crc =
        CRC16_Calculate(frame, frame_len - 2);

    uint16_t received_crc =
        (uint16_t)frame[frame_len - 2] |
        ((uint16_t)frame[frame_len - 1] << 8);

    return (calculated_crc == received_crc);
}