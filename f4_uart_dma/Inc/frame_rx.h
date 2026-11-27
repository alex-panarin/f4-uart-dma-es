#ifndef FRAME_RX_H
#define FRAME_RX_H

#include <stdint.h>

/*
 * Кадр (наследие Недели 2):
 *   SYNC | LEN(2, LE) | CMD | PAYLOAD[LEN] | CRC16(2, LE)
 * CRC16 = CRC-16/CCITT-FALSE по байтам LEN..PAYLOAD включительно.
 *
 * Zero-copy: парсер не копирует байты, а выдаёт дескриптор payload
 * (индекс в кольце + длина). Индекс, а не указатель — потому что payload
 * может пересекать границу кольца.
 */

#define FRAME_SYNC        0xA5U
#define FRAME_MAX_PAYLOAD 300U
#define FRAME_HDR         4U     /* SYNC + LEN_lo + LEN_hi + CMD */
#define FRAME_CRC         2U
#define FRAME_OVERHEAD    (FRAME_HDR + FRAME_CRC)

/* Коды возврата frame_rx_poll. */
#define FRAME_NONE  0    /* данных не хватает, ничего не потреблено */
#define FRAME_OK    1    /* кадр разобран и потреблён, view заполнен */
#define FRAME_BAD   (-1) /* шум/битый CRC: потреблён 1 байт, вызывать снова */

typedef struct {
    uint8_t  cmd;
    uint16_t len;
    uint32_t idx;   /* индекс первого байта payload в кольце */
} frame_view_t;

typedef struct {
    uint32_t frames_ok;
    uint32_t crc_err;
    uint32_t skipped;
    uint32_t maxlen;
} frame_stats_t;

void frame_rx_init(void);
int  frame_rx_poll(frame_view_t *v);
const frame_stats_t *frame_rx_stats(void);

#endif /* FRAME_RX_H */
