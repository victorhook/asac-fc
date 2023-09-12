#include "stdint.h"
#include "stdio.h"


static uint8_t crc8_dvbs2(const uint8_t* data, const uint8_t len)
{
    uint8_t poly = 0xD5;
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc = crc << 1;
            }
        }

    }
    return crc;
}

int main()
{
    uint8_t data[] = {0x16, 0xe0, 0x23, 0x1f, 0x2d, 0xc0, 0xf7, 0x8b, 0xf2, 0xfd, 0xa2, 0x7c, 0xe5, 0x2b, 0x5f, 0xf9, 0xca, 0x07, 0x00, 0x00, 0x4c, 0x7c, 0xe2};
    uint8_t crc = crc8_dvbs2(data, 23);
    printf("CRC: 0x%02x\n", crc);
    return 0;
}