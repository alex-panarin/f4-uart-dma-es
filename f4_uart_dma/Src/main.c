/*
 * День 17: арбитраж шин, приоритеты, детектор обгона кольца.
 *
 * Команды:
 *   0x01 ECHO, 0x02 STATS, 0x03 TXSTATS
 *   0x04 RXSTATS  -> written, consumed, laps, lost, head, tail (24 Б)
 *   0x05 STALL    ms u32 -> управляемая занятость main (для Лабы B)
 *   0x10 JEDEC, 0x11 FERASE, 0x12 FWRITE, 0x13 FBENCH, 0x14 SPISTATS
 *   0x15 LOAD     addr u32, len u32 -> фоновое чтение SPI без блокировки main
 */

#include "uart_dma.h"
#include "frame_rx.h"
#include "crc16.h"
#include "spi_dma.h"
#include "w25_dma.h"
#include "clock.h"
#include "power.h"

#define CMD_ECHO     	0x01U
#define CMD_STATS    	0x02U
#define CMD_TXSTATS  	0x03U
#define CMD_RXSTATS  	0x04U
#define CMD_STALL    	0x05U
#define CMD_JEDEC    	0x10U
#define CMD_FERASE   	0x11U
#define CMD_FWRITE   	0x12U
#define CMD_FBENCH   	0x13U
#define CMD_SPISTATS 	0x14U
#define CMD_LOAD     	0x15U
#define CMD_SYSINFO  	0x16U
#define CMD_PING       	0x20U
#define CMD_CLOCK_INFO 	0x21U
#define CMD_FSTATUS 	0x23U
#define CMD_FSESSION 	0x24U
#define CMD_FPAGE    	0x25U
#define CMD_FSYNC    	0x26U
#define CMD_SLEEP     0x27U
#define CMD_STOP      0x28U
#define CMD_PWRSTATS  0x29U

#define FSESS_MAX_PAGES 256U   /* битмап 32 байта = до 64 КБ */

#define PROJECT_VERSION 0x00120000UL
#define PROJECT_MAGIC   0x314F4953UL   /* 'SIO1' */

#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFCUL)
#define CPU_HZ 		84000000UL

#define CHUNK      1024U
#define BENCH_MAX  65536UL

static uint8_t rbuf[2][CHUNK];
static uint8_t g_loadbuf[CHUNK];
static uint32_t g_fw_ok = 0U;
static uint32_t g_fw_fail = 0U;

static struct {
    uint8_t  active;
    uint32_t base;
    uint16_t total;
    uint8_t  bitmap[FSESS_MAX_PAGES / 8U];
} g_fsess;

static struct {
    uint8_t  active;
    uint32_t addr;
    uint32_t remaining;
    uint32_t bytes;
} g_load;

static void dwt_init(void)
{
    DEMCR      |= (1UL << 24);
    DWT_CYCCNT  = 0UL;
    DWT_CTRL   |= 1UL;
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_header(uint8_t *p, uint8_t cmd, uint16_t len, uint16_t *crc)
{
    p[0] = FRAME_SYNC;
    p[1] = (uint8_t)(len & 0xFFU);
    p[2] = (uint8_t)(len >> 8);
    p[3] = cmd;
    *crc = 0xFFFFU;
    *crc = crc16_byte(*crc, p[1]);
    *crc = crc16_byte(*crc, p[2]);
    *crc = crc16_byte(*crc, p[3]);
}

static void reply_buf(uint8_t cmd, const uint8_t *src, uint16_t len)
{
    uint8_t *p = uart_dma_tx_alloc();
    if (p == (uint8_t *)0) {
        return;
    }
    uint16_t crc;
    put_header(p, cmd, len, &crc);
    for (uint32_t i = 0; i < len; i++) {
        p[FRAME_HDR + i] = src[i];
        crc = crc16_byte(crc, src[i]);
    }
    p[FRAME_HDR + len]     = (uint8_t)(crc & 0xFFU);
    p[FRAME_HDR + len + 1] = (uint8_t)(crc >> 8);
    uart_dma_tx_commit((uint16_t)(FRAME_OVERHEAD + len));
}

static void reply_view(uint8_t cmd, const frame_view_t *v)
{
    uint8_t *p = uart_dma_tx_alloc();
    if (p == (uint8_t *)0) {
        return;
    }
    uint16_t crc;
    put_header(p, cmd, v->len, &crc);
    for (uint32_t i = 0; i < v->len; i++) {
        uint8_t b = uart_dma_at(v->idx + i);
        p[FRAME_HDR + i] = b;
        crc = crc16_byte(crc, b);
    }
    p[FRAME_HDR + v->len]     = (uint8_t)(crc & 0xFFU);
    p[FRAME_HDR + v->len + 1] = (uint8_t)(crc >> 8);
    uart_dma_tx_commit((uint16_t)(FRAME_OVERHEAD + v->len));
}

static void handle_fstatus(void)
{
    uint8_t b[8];
    wr_u32(&b[0], g_fw_ok);
    wr_u32(&b[4], g_fw_fail);
    reply_buf(CMD_FSTATUS | 0x80U, b, sizeof(b));
}

static void handle_sysinfo(void)
{
    uint8_t b[80];
    const uart_dma_stats_t *us = uart_dma_get_stats();
    const uart_tx_stats_t  *ts = uart_dma_tx_stats();
    const frame_stats_t    *fs = frame_rx_stats();
    uint32_t ovr, te, dme, fe, tmo;
    spi1_err_detail(&ovr, &te, &dme, &fe, &tmo);

    wr_u32(&b[0],  PROJECT_VERSION);
    wr_u32(&b[4],  uart_dma_written());
    wr_u32(&b[8],  us->rx_bytes);
    wr_u32(&b[12], uart_dma_laps());
    wr_u32(&b[16], uart_dma_lost());
    wr_u32(&b[20], us->usart_ore);
    wr_u32(&b[24], us->usart_idle);
    wr_u32(&b[28], ts->frames);
    wr_u32(&b[32], ts->maxq);
    wr_u32(&b[36], ts->stall);
    wr_u32(&b[40], spi1_done_count());
    wr_u32(&b[44], ovr);
    wr_u32(&b[48], te);
    wr_u32(&b[52], dme);
    wr_u32(&b[56], fe);
    wr_u32(&b[60], tmo);
    wr_u32(&b[64], fs->frames_ok);
    wr_u32(&b[68], fs->crc_err);
    wr_u32(&b[72], fs->skipped);
    wr_u32(&b[76], fs->maxlen);

    reply_buf(CMD_SYSINFO | 0x80U, b, sizeof(b));
}

static void handle_ping(const uint8_t *p, uint16_t len)
{
    uint8_t out[8];
    uint32_t ms = (len >= 4U) ? rd_u32(p) : 1U;

    if (ms > 5000U) {
        ms = 5000U;   /* защита: 5000 мс × 84 МГц = 420 млн тактов < 2^32 */
    }

    uint32_t t0 = DWT_CYCCNT;

    /* Активная задержка на DWT CYCCNT — считает реальные такты. */
    uint32_t lim = ms * (CPU_HZ / 1000UL);
    while ((DWT_CYCCNT - t0) < lim) {
    }

    uint32_t t1 = DWT_CYCCNT;

    wr_u32(&out[0], t0);
    wr_u32(&out[4], t1);

    reply_buf(CMD_PING | 0x80U, out, sizeof(out));
}

static void handle_clock_info(void)
{
    uint8_t out[4];
    wr_u32(out, clock_sysclk_source());
    reply_buf(CMD_CLOCK_INFO | 0x80U, out, sizeof(out));
}

static void handle_stats(void)
{
    uint8_t b[32];
    const frame_stats_t *fs = frame_rx_stats();
    const uart_dma_stats_t *us = uart_dma_get_stats();
    wr_u32(&b[0], fs->frames_ok);   wr_u32(&b[4], fs->crc_err);
    wr_u32(&b[8], fs->skipped);     wr_u32(&b[12], fs->maxlen);
    wr_u32(&b[16], us->rx_bytes);   wr_u32(&b[20], us->dma_tc);
    wr_u32(&b[24], us->usart_idle); wr_u32(&b[28], us->usart_ore);
    reply_buf(CMD_STATS | 0x80U, b, sizeof(b));
}

static void handle_txstats(void)
{
    uint8_t b[16];
    const uart_tx_stats_t *ts = uart_dma_tx_stats();
    wr_u32(&b[0], ts->frames); wr_u32(&b[4], ts->tc);
    wr_u32(&b[8], ts->maxq);   wr_u32(&b[12], ts->stall);
    reply_buf(CMD_TXSTATS | 0x80U, b, sizeof(b));
}

static void handle_rxstats(void)
{
    uint8_t b[24];
    wr_u32(&b[0],  uart_dma_written());
    wr_u32(&b[4],  uart_dma_get_stats()->rx_bytes);
    wr_u32(&b[8],  uart_dma_laps());
    wr_u32(&b[12], uart_dma_lost());
    wr_u32(&b[16], uart_dma_head());
    wr_u32(&b[20], uart_dma_tail());
    reply_buf(CMD_RXSTATS | 0x80U, b, sizeof(b));

}

static void handle_sleep(const uint8_t *p, uint16_t len)
{
    uint32_t ms = (len >= 4U) ? rd_u32(p) : 100U;
    uint32_t cycles = power_sleep_ms(ms);

    uint8_t b[8];
    wr_u32(&b[0], cycles);
    wr_u32(&b[4], ms);
    reply_buf(CMD_SLEEP | 0x80U, b, sizeof(b));
}

static void handle_stop(void)
{
    /* Ответ уходит ПОСЛЕ пробуждения: хост обязан разбудить устройство. */
    uint32_t cycles = power_stop();

    uint8_t b[8];
    wr_u32(&b[0], cycles);
    wr_u32(&b[4], clock_sysclk_source());
    reply_buf(CMD_STOP | 0x80U, b, sizeof(b));
}

static void handle_pwrstats(void)
{
    uint8_t b[28];
    const power_stats_t *ps = power_stats();

    wr_u32(&b[0],  ps->sleep_entries);
    wr_u32(&b[4],  ps->stop_entries);
    wr_u32(&b[8],  ps->wake_exti);
    wr_u32(&b[12], ps->clock_restores);
    wr_u32(&b[16], ps->drain_timeouts);
    wr_u32(&b[20], ps->last_wake_cycles);
    wr_u32(&b[24], ps->rx_flushed);

    reply_buf(CMD_PWRSTATS | 0x80U, b, sizeof(b));
}
static void handle_stall(const uint8_t *p, uint16_t len)
{
    uint32_t ms = (len >= 4U) ? rd_u32(p) : 0U;
    if (ms > 2000U) {
        ms = 2000U;
    }
    uint32_t t0 = DWT_CYCCNT;
    uint32_t lim = ms * (CPU_HZ / 1000UL);
    while ((DWT_CYCCNT - t0) < lim) {
    }
    uint8_t st = 0U;
    reply_buf(CMD_STALL | 0x80U, &st, 1U);
}

static void handle_load(const uint8_t *p, uint16_t len)
{
    uint8_t st = 1U;
    if (len >= 8U) {
        g_load.addr      = rd_u32(p);
        g_load.remaining = rd_u32(p + 4);
        g_load.bytes     = 0U;
        g_load.active    = 1U;
        st = 0U;
    }
    reply_buf(CMD_LOAD | 0x80U, &st, 1U);
}

static void handle_spistats(void)
{
    uint8_t b[28];
    uint32_t ovr, te, dme, fe, tmo;
    spi1_err_detail(&ovr, &te, &dme, &fe, &tmo);
    wr_u32(&b[0],  spi1_done_count());
    wr_u32(&b[4],  spi1_err_count());
    wr_u32(&b[8],  ovr);
    wr_u32(&b[12], te);
    wr_u32(&b[16], dme);
    wr_u32(&b[20], fe);
    wr_u32(&b[24], tmo);
    reply_buf(CMD_SPISTATS | 0x80U, b, sizeof(b));
}

static void handle_fsession(const uint8_t *p, uint16_t len)
{
    uint8_t st = 1U;

    if (len >= 7U) {
        uint32_t base = rd_u32(p);
        uint16_t total = (uint16_t)(p[4] | (p[5] << 8));
        uint8_t  do_erase = p[6];

        if (total <= FSESS_MAX_PAGES) {
            g_fsess.base = base;
            g_fsess.total = total;
            for (uint32_t i = 0; i < sizeof(g_fsess.bitmap); i++) {
                g_fsess.bitmap[i] = 0U;
            }
            g_fsess.active = 1U;
            st = 0U;

            if (do_erase) {
                uint32_t nsec = ((uint32_t)total * 256U + 4095U) / 4096U;
                for (uint32_t k = 0; k < nsec; k++) {
                    if (w25_sector_erase(base + k * 4096U) != 0) {
                        st = 1U;
                        break;
                    }
                }
            }
        }
    }
    reply_buf(CMD_FSESSION | 0x80U, &st, 1U);
}

static void handle_fpage(const uint8_t *p, uint16_t len)
{
    uint16_t seq = 0U;
    uint8_t  st = 1U;

    if (g_fsess.active && len >= 2U) {
        seq = (uint16_t)(p[0] | (p[1] << 8));
        if (seq < g_fsess.total) {
            uint32_t addr = g_fsess.base + (uint32_t)seq * 256U;
            uint16_t dlen = (uint16_t)(len - 2U);
            if (w25_page_program(addr, &p[2], dlen) == 0) {
                g_fsess.bitmap[seq >> 3] |= (uint8_t)(1U << (seq & 7U));
                g_fw_ok++;
                st = 0U;
            } else {
                g_fw_fail++;
            }
        }
    }

    uint8_t resp[3];
    resp[0] = (uint8_t)(seq & 0xFFU);
    resp[1] = (uint8_t)(seq >> 8);
    resp[2] = st;
    reply_buf(CMD_FPAGE | 0x80U, resp, 3U);
}

static void handle_fsync(void)
{
    uint16_t bitmap_bytes = (uint16_t)((g_fsess.total + 7U) / 8U);
    uint8_t resp[2U + FSESS_MAX_PAGES / 8U];

    resp[0] = (uint8_t)(g_fsess.total & 0xFFU);
    resp[1] = (uint8_t)(g_fsess.total >> 8);
    for (uint16_t i = 0; i < bitmap_bytes; i++) {
        resp[2U + i] = g_fsess.bitmap[i];
    }
    reply_buf(CMD_FSYNC | 0x80U, resp, (uint16_t)(2U + bitmap_bytes));
}

static void handle_ferase(const uint8_t *p, uint16_t len)
{
    uint8_t st = 1U;
    if (len >= 4U) {
        st = (w25_sector_erase(rd_u32(p)) == 0) ? 0U : 1U;
    }
    reply_buf(CMD_FERASE | 0x80U, &st, 1U);
}

static void handle_fwrite(const uint8_t *p, uint16_t len)
{
    uint8_t st = 1U;
    if (len >= 5U) {
        st = (w25_page_program(rd_u32(p), &p[4], (uint16_t)(len - 4U)) == 0) ? 0U : 1U;
    }
    if (st == 0U) {
        g_fw_ok++;
    } else {
        g_fw_fail++;
    }
    reply_buf(CMD_FWRITE | 0x80U, &st, 1U);
}

static void handle_fbench(const uint8_t *p, uint16_t len)
{
    uint8_t out[10];

    if (len < 9U) {
        wr_u32(&out[0], 0U); wr_u32(&out[4], 0U); out[8] = 0U; out[9] = 0U;
        reply_buf(CMD_FBENCH | 0x80U, out, sizeof(out));
        return;
    }

    uint32_t addr  = rd_u32(p);
    uint32_t total = rd_u32(p + 4);
    uint8_t  mode  = p[8];

    if ((total == 0U) || (total > BENCH_MAX)) {
        wr_u32(&out[0], 0U); wr_u32(&out[4], 0U); out[8] = 0U; out[9] = 0U;
        reply_buf(CMD_FBENCH | 0x80U, out, sizeof(out));
        return;
    }

    uint16_t crc = 0xFFFFU;
    uint32_t first = (total < CHUNK) ? total : CHUNK;

    uint32_t t0 = DWT_CYCCNT;
    w25_read_async(addr, rbuf[0], (uint16_t)first);
    while (spi1_busy()) {
    }

    uint32_t off = 0U, idx = 0U;
    while (off < total) {
        uint32_t cur  = ((total - off) < CHUNK) ? (total - off) : CHUNK;
        uint32_t noff = off + cur;
        uint8_t *cb   = rbuf[idx & 1U];

        if (noff < total) {
            uint32_t nxt = ((total - noff) < CHUNK) ? (total - noff) : CHUNK;
            if (mode) {
                w25_read_async(addr + noff, rbuf[(idx + 1U) & 1U], (uint16_t)nxt);
                for (uint32_t i = 0; i < cur; i++) { crc = crc16_byte(crc, cb[i]); }
                while (spi1_busy()) { }
            } else {
                for (uint32_t i = 0; i < cur; i++) { crc = crc16_byte(crc, cb[i]); }
                w25_read_async(addr + noff, rbuf[(idx + 1U) & 1U], (uint16_t)nxt);
                while (spi1_busy()) { }
            }
        } else {
            for (uint32_t i = 0; i < cur; i++) { crc = crc16_byte(crc, cb[i]); }
        }
        off = noff;
        idx++;
    }

    uint32_t us  = (DWT_CYCCNT - t0) / (CPU_HZ / 1000000UL);
    uint32_t bps = (us > 0U) ? (uint32_t)((uint64_t)total * 1000000ULL / us) : 0UL;

    out[0] = (uint8_t)(crc & 0xFFU);
    out[1] = (uint8_t)(crc >> 8);
    wr_u32(&out[2], us);
    wr_u32(&out[6], bps);
    reply_buf(CMD_FBENCH | 0x80U, out, sizeof(out));
}

int main(void)
{
	clock_init();

	uart_dma_init(UART_DMA_BAUD_460800);
	uart_dma_tx_init();
	crc16_init();
	frame_rx_init();
	dwt_init();
	w25_init(SPI_BR_DIV8);
	power_init();

	uart_dma_puts("\r\n== Day19 clock 84 MHz ==\r\n");
	uart_dma_puts("sysclk source: ");
	uart_dma_putc('0' + (char)clock_sysclk_source());
	uart_dma_puts("\r\n");

    uart_dma_puts("\r\n== PROJECT#2 Stream-IO v0.18 ==\r\n");

    while (1) {
        /* Детектор обгона: до разбора кадров, каждый проход. */
        uart_dma_check_lap();

        /* Фоновая SPI-нагрузка: не блокирует main. */
        if (g_load.active && !spi1_busy()) {
            if (g_load.remaining > 0U) {
                uint32_t n = (g_load.remaining < CHUNK) ? g_load.remaining : CHUNK;
                w25_read_async(g_load.addr, g_loadbuf, (uint16_t)n);
                g_load.addr      += n;
                g_load.remaining -= n;
                g_load.bytes     += n;
            } else {
                g_load.active = 0U;
            }
        }

        if (uart_dma_tx_free() == 0U) {
            continue;
        }

        frame_view_t v;
        if (frame_rx_poll(&v) != FRAME_OK) {
            continue;
        }

        uint8_t pl[FRAME_MAX_PAYLOAD];
        for (uint32_t i = 0; i < v.len; i++) {
            pl[i] = uart_dma_at(v.idx + i);
        }

        switch (v.cmd) {
        case CMD_ECHO:     reply_view(CMD_ECHO | 0x80U, &v); break;
        case CMD_STATS:    handle_stats();                   break;
        case CMD_TXSTATS:  handle_txstats();                 break;
        case CMD_RXSTATS:  handle_rxstats();                 break;
        case CMD_STALL:    handle_stall(pl, v.len);          break;
        case CMD_SPISTATS: handle_spistats();                break;
        case CMD_LOAD:     handle_load(pl, v.len);           break;
        case CMD_FERASE:   handle_ferase(pl, v.len);         break;
        case CMD_FWRITE:   handle_fwrite(pl, v.len);         break;
        case CMD_FBENCH:   handle_fbench(pl, v.len);         break;
        case CMD_FSESSION: handle_fsession(pl, v.len); 		 break;
        case CMD_FPAGE:    handle_fpage(pl, v.len);    		 break;
        case CMD_FSYNC:    handle_fsync();             		 break;
        case CMD_FSTATUS:  handle_fstatus(); 				 break;
        case CMD_SYSINFO:  handle_sysinfo(); 				 break;
        case CMD_PING:     handle_ping(pl, v.len);  		 break;
        case CMD_CLOCK_INFO: handle_clock_info();    		 break;
        case CMD_SLEEP:    handle_sleep(pl, v.len); break;
        case CMD_STOP:     handle_stop();           break;
        case CMD_PWRSTATS: handle_pwrstats();       break;
        case CMD_JEDEC: {
            uint8_t b[4];
            wr_u32(b, w25_jedec());
            reply_buf(CMD_JEDEC | 0x80U, b, 4U);
            break;
        }
        default:
            reply_buf((uint8_t)(v.cmd | 0x80U), (const uint8_t *)0, 0U);
            break;
        }
    }
}
