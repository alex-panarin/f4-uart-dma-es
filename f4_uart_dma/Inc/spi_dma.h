#ifndef SPI_DMA_H
#define SPI_DMA_H

#include <stdint.h>

/*
 * SPI1 master + DMA2 полный дуплекс.
 * Плата STM32_F4VE: CS=PB0, SCK=PB3, MISO=PB4, MOSI=PB5 (AF5).
 * RX = DMA2 Stream0 Ch3 (IRQ 56), TX = DMA2 Stream3 Ch3.
 * Stream5 для SPI1_TX не используем: занят USART1_RX.
 */

#define SPI_BR_DIV2 0U    /* 8 MHz @ PCLK2 = 16 MHz */
#define SPI_BR_DIV4 1U    /* 4 MHz */
#define SPI_BR_DIV8 2U    /* делитель 8: 84/8 = 10.5 МГц */
#define SPI_BR_DIV16 3U    /* делитель 16: 84/16 = 5.25 МГц */

void spi1_dma_init(uint32_t br);

void spi1_cs_low(void);
void spi1_cs_high(void);

/* Асинхронная транзакция. tx и rx обязаны быть длиной len.
 * release_cs=1 — поднять CS по завершении (конец транзакции),
 * release_cs=0 — оставить CS низким (составная транзакция). */
int  spi1_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t release_cs);

int      spi1_busy(void);
uint32_t spi1_done_count(void);
uint32_t spi1_err_count(void);

/* Раздельная атрибуция ошибок (W3-20). fe — информационный, не фатальный. */
void spi1_err_detail(uint32_t *ovr, uint32_t *te, uint32_t *dme,
                     uint32_t *fe, uint32_t *tmo);

#endif /* SPI_DMA_H */
