#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

/*
 * Тактовая система F407.
 *
 * HSI 16 МГц -> PLL (M=8, N=336, P=8, Q=7) -> SYSCLK 84 МГц
 *   AHB  = SYSCLK / 1 = 84 МГц
 *   APB1 = AHB / 2    = 42 МГц (USART2/3, таймеры 2–7)
 *   APB2 = AHB / 1    = 84 МГц (USART1, SPI1, таймеры 1, 8–11)
 * Flash: 2 WS при VDD=3.3V, prefetch + ART.
 *
 * Выбор 84 вместо 168: Flash latency 2 вместо 5 WS, запас по стабильности,
 * периферия остаётся в штатных диапазонах. Для 168 МГц достаточно изменить
 * P на 4 и WS на 5 — остальное не меняется.
 */

#define SYSCLK_HZ   84000000UL
#define HCLK_HZ     SYSCLK_HZ
#define PCLK1_HZ    (HCLK_HZ / 2U)
#define PCLK2_HZ    (HCLK_HZ)

void clock_init(void);

/* Для DWT: реальная частота CPU. На F4 CYCCNT идёт на HCLK, поэтому == SYSCLK. */
uint32_t clock_cpu_hz(void);

/* Проверка: возвращает фактический источник SYSCLK (0=HSI, 1=HSE, 2=PLL, 3=HSI). */
uint32_t clock_sysclk_source(void);

#endif /* CLOCK_H */
