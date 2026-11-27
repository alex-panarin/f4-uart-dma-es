#ifndef POWER_H
#define POWER_H

#include <stdint.h>

/*
 * Режимы пониженного потребления F407.
 *
 * Sleep: ядро стоит, периферия работает, пробуждение по любому IRQ.
 *        Таймаут задаётся SysTick (HCLK/8), чтобы не спать вечно.
 * Stop:  все такты стоят, SRAM/регистры живы, пробуждение по EXTI.
 *        ВНИМАНИЕ: RCC сбрасывается на HSI => обязателен clock_init().
 *        DWT CYCCNT в Stop не считает => время через сон теряется.
 *
 * Пробуждение: EXTI line 10 (PA10 = USART1_RX), спадающий фронт стартового бита.
 * Первый байт после пробуждения теряется — хост обязан слать преамбулу.
 */

void power_init(void);

/* Sleep на ms (<=1500) или до первого IRQ. Возвращает такты DWT за период. */
uint32_t power_sleep_ms(uint32_t ms);

/* Stop до EXTI-пробуждения. Восстанавливает такты.
 * Возвращает такты DWT, накопленные за пробуждение (сам сон не считается). */
uint32_t power_stop(void);

typedef struct {
    uint32_t sleep_entries;
    uint32_t stop_entries;
    uint32_t wake_exti;
    uint32_t clock_restores;
    uint32_t drain_timeouts;
    uint32_t last_wake_cycles;
    uint32_t rx_flushed;      /* байт отброшено после пробуждения */
} power_stats_t;

const power_stats_t *power_stats(void);

#endif /* POWER_H */
