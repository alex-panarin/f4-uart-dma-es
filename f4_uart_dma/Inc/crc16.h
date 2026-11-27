#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

/*
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, без рефлексии, xorout 0.
 * W3-15: табличная реализация — 1 обращение на байт вместо 8 итераций.
 * Перед первым использованием обязателен crc16_init().
 */

extern uint16_t crc16_tbl[256];

void crc16_init(void);

static inline uint16_t crc16_byte(uint16_t crc, uint8_t b)
{
    return (uint16_t)((uint16_t)(crc << 8) ^ crc16_tbl[((crc >> 8) ^ b) & 0xFFU]);
}

uint16_t crc16_ccitt(const void *buf, uint32_t len);

#endif /* CRC16_H */
