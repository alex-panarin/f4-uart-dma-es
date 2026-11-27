#include "crc16.h"

uint16_t crc16_tbl[256];

void crc16_init(void)
{
    for (uint32_t i = 0; i < 256U; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (uint32_t k = 0; k < 8U; k++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
        crc16_tbl[i] = crc;
    }
}

uint16_t crc16_ccitt(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0; i < len; i++) {
        crc = crc16_byte(crc, p[i]);
    }
    return crc;
}
