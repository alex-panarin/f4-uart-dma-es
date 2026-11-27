#ifndef UART_DMA_H
#define UART_DMA_H

#include <stdint.h>

/*
 * UART1 + DMA2: RX Stream5 Ch4 (CIRC-кольцо), TX Stream7 Ch4 (очередь слотов).
 * BRR @ HSI 16 MHz: 115200 -> 0x008B, 460800 -> 0x0023.
 * NVIC: USART1=37 (ISER1), DMA2_Stream5=68, Stream7=70 (ISER2).
 *
 * День 17: детектор обгона кольца + настраиваемый приоритет RX.
 * UART RX имеет жёсткий внешний дедлайн (байт каждые 21.7 мкс @460800),
 * поэтому приоритет не должен быть ниже эластичных потоков.
 */

#define UART_DMA_BAUD_115200 0x02DBUL
#define UART_DMA_BAUD_460800 0x00B6UL

#define UART_DMA_RX_RING 4096U
#define UART_DMA_RX_MASK (UART_DMA_RX_RING - 1U)

#define UART_DMA_TX_SLOTS 8U
#define UART_DMA_TX_SLOT  320U

/* 0 low, 1 medium, 2 high, 3 very high. Лаба C: варьировать 0 <-> 3. */
#define UART_RX_PL 2U
#define UART_TX_PL 1U

void uart_dma_init(uint32_t brr);
void uart_dma_tx_init(void);

int  uart_dma_getc(void);
void uart_dma_putc(char c);
void uart_dma_puts(const char *s);

uint32_t uart_dma_head(void);
uint32_t uart_dma_tail(void);
uint32_t uart_dma_available(void);
uint8_t  uart_dma_at(uint32_t idx);
void     uart_dma_drop(uint32_t n);

/* День 17: согласованный счётчик записанного и детектор обгона. */
uint32_t uart_dma_written(void);
uint32_t uart_dma_check_lap(void);   /* возвращает число потерянных байт, 0 если нет */
uint32_t uart_dma_laps(void);
uint32_t uart_dma_lost(void);

uint8_t *uart_dma_tx_alloc(void);
void     uart_dma_tx_commit(uint16_t n);
uint32_t uart_dma_tx_free(void);

typedef struct {
    uint32_t rx_bytes;      /* байт удалено из кольца (разобрано или отброшено) */
    uint32_t rx_isr_calls;
    uint32_t dma_tc;
    uint32_t usart_idle;
    uint32_t usart_ore;
} uart_dma_stats_t;

typedef struct {
    uint32_t frames;
    uint32_t tc;
    uint32_t maxq;
    uint32_t stall;
} uart_tx_stats_t;

const uart_dma_stats_t *uart_dma_get_stats(void);
const uart_tx_stats_t  *uart_dma_tx_stats(void);

#endif /* UART_DMA_H */
