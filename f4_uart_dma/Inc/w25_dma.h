#ifndef W25_DMA_H
#define W25_DMA_H

#include <stdint.h>

/*
 * W25Qxx поверх SPI1+DMA. Фактический чип на плате: JEDEC EF 40 15 (32 Mbit).
 * Все.bulk-фазы идут через DMA; короткие команды — тоже DMA (единый путь).
 */

#define W25_MAX_XFER 1024U

void     w25_init(uint32_t spi_br);
uint32_t w25_jedec(void);
uint8_t  w25_rdsr(void);

int  w25_sector_erase(uint32_t addr);                                  /* 4K, ждёт WIP */
int  w25_page_program(uint32_t addr, const uint8_t *data, uint16_t len); /* <=256, без перехода страницы */
int  w25_read(uint32_t addr, uint8_t *dst, uint16_t len);              /* блокирующая обёртка */

/* Асинхронное чтение: заголовок отправляется синхронно, фаза данных — по DMA.
 * Возврат сразу; готовность по spi1_busy(). len <= W25_MAX_XFER. */
int  w25_read_async(uint32_t addr, uint8_t *dst, uint16_t len);

#endif /* W25_DMA_H */
