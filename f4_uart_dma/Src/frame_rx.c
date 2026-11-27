#include "frame_rx.h"
#include "uart_dma.h"
#include "crc16.h"

static frame_stats_t g_st;

void frame_rx_init(void)
{
    g_st.frames_ok = 0U;
    g_st.crc_err   = 0U;
    g_st.skipped   = 0U;
    g_st.maxlen    = 0U;
}

const frame_stats_t *frame_rx_stats(void)
{
    return &g_st;
}

/*
 * Идемпотентный try-parse: кадр всегда пробуем от текущего tail.
 * Байты не потребляются, пока решение не принято (откат не нужен).
 */
int frame_rx_poll(frame_view_t *v)
{
    uint32_t avail = uart_dma_available();
    uint32_t t;

    if (avail == 0U)
    {
        return FRAME_NONE;
    }

    t = uart_dma_tail();

    if (uart_dma_at(t) != FRAME_SYNC)
    {
        uart_dma_drop(1U);
        g_st.skipped++;
        return FRAME_BAD;
    }

    if (avail < FRAME_HDR)
    {
        return FRAME_NONE;
    }

    uint16_t len = (uint16_t)uart_dma_at(t + 1U)
                 | (uint16_t)((uint16_t)uart_dma_at(t + 2U) << 8);
    uint8_t  cmd = uart_dma_at(t + 3U);

    if (len > FRAME_MAX_PAYLOAD)
    {
        uart_dma_drop(1U);
        g_st.skipped++;
        return FRAME_BAD;
    }

    uint32_t total = FRAME_OVERHEAD + (uint32_t)len;
    if (avail < total)
    {
        return FRAME_NONE;                 /* кадр ещё не доехал целиком */
    }

    /* CRC по LEN..PAYLOAD: 3 байта заголовка + len байт payload. */
    uint16_t crc = 0xFFFFU;
    crc = crc16_byte(crc, uart_dma_at(t + 1U));
    crc = crc16_byte(crc, uart_dma_at(t + 2U));
    crc = crc16_byte(crc, cmd);
    for (uint32_t i = 0; i < len; i++)
    {
        crc = crc16_byte(crc, uart_dma_at(t + FRAME_HDR + i));
    }

    uint16_t rx_crc = (uint16_t)uart_dma_at(t + FRAME_HDR + len)
                    | (uint16_t)((uint16_t)uart_dma_at(t + FRAME_HDR + len + 1U) << 8);

    if (crc != rx_crc)
    {
        uart_dma_drop(1U);                 /* ресинк по одному байту */
        g_st.crc_err++;
        return FRAME_BAD;
    }

    v->cmd = cmd;
    v->len = len;
    v->idx = (t + FRAME_HDR) & UART_DMA_RX_MASK;

    uart_dma_drop(total);

    g_st.frames_ok++;
    if ((uint32_t)len > g_st.maxlen)
    {
        g_st.maxlen = len;
    }
    return FRAME_OK;
}
