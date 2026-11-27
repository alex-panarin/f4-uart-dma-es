/*
 * День 19: переход с HSI 16 МГц на PLL 84 МГц.
 *
 * Порядок переключения (обязательный):
 *   1. Включить HSI, дождаться HSIRDY
 *   2. Переключить SYSCLK на HSI (SW=HSI), дождаться SWS=HSI
 *   3. Выключить PLL (PLLRST=1, PLLON=0), дождаться PLLRDY=0
 *   4. Записать M, N, P, Q в PLLCFGR
 *   5. Установить Flash latency ДО включения PLL (иначе сбой при старте)
 *   6. Включить PLL, дождаться PLLRDY
 *   7. Переключить SYSCLK на PLL (SW=PLL), дождаться SWS=PLL
 *   8. Обновить делители AHB/APB1/APB2 в CFGR
 */

#include "clock.h"

#define RCC_BASE      0x40023800UL
#define RCC_CR        (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_PLLCFGR   (*(volatile uint32_t *)(RCC_BASE + 0x04UL))
#define RCC_CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x08UL))

#define FLASH_ACR     (*(volatile uint32_t *)0x40023C00UL)

/* CR bits */
#define CR_HSION      (1UL << 0)
#define CR_HSIRDY     (1UL << 1)
#define CR_PLLON      (1UL << 24)
#define CR_PLLRDY     (1UL << 25)

/* PLLCFGR bits */
#define PLLCFGR_PLLSRC_HSI  (0UL << 22)
#define PLLCFGR_PLLSRC_HSE  (1UL << 22)
#define PLLCFGR_M(m)        ((uint32_t)(m))
#define PLLCFGR_N(n)        ((uint32_t)(n) << 6)
#define PLLCFGR_P(p)        ((((uint32_t)(p) >> 1) - 1U) << 16)   /* p = 2,4,6,8 */
#define PLLCFGR_Q(q)        ((uint32_t)(q) << 24)

/* CFGR: SW (bits 0..1), SWS (bits 2..3), HPRE (4..7), PPRE1 (10..12), PPRE2 (13..15) */
#define CFGR_SW_HSI    0x0UL
#define CFGR_SW_PLL    0x2UL
#define CFGR_SWS_MASK  0xCUL
#define CFGR_SWS_HSI   0x0UL
#define CFGR_SWS_PLL   0x8UL

#define CFGR_HPRE_DIV1   (0x0UL << 4)
#define CFGR_PPRE1_DIV2  (0x4UL << 10)
#define CFGR_PPRE2_DIV1  (0x0UL << 13)

/* FLASH_ACR bits */
#define ACR_LATENCY(ws)  ((uint32_t)(ws))
#define ACR_PRFTEN       (1UL << 8)
#define ACR_ICEN         (1UL << 9)
#define ACR_DCEN         (1UL << 10)

void clock_init(void)
{
    /* 1. Включить внешний кварц 8 МГц и дождаться стабилизации. */
    RCC_CR |= (1UL << 16);                    /* HSEON */
    while (!(RCC_CR & (1UL << 17))) {         /* HSERDY */
    }

    /* 1а. Опционально: детектор отказа кварца (рекомендуется). */
    RCC_CR |= (1UL << 19);                    /* CSSON */

    /* 2. Включить HSI как резерв и переключить на него. */
    RCC_CR |= CR_HSION;
    while (!(RCC_CR & CR_HSIRDY)) {
    }
    RCC_CFGR = (RCC_CFGR & ~0x3UL) | CFGR_SW_HSI;
    while ((RCC_CFGR & CFGR_SWS_MASK) != CFGR_SWS_HSI) {
    }

    /* 3. Выключить PLL. */
    RCC_CR &= ~CR_PLLON;
    while (RCC_CR & CR_PLLRDY) {
    }

    /* 4. PLLCFGR: HSE → M=4 → N=168 → P=4 → 84 МГц, Q=7 → USB 48. */
    RCC_PLLCFGR = PLLCFGR_PLLSRC_HSE          /* ← теперь внешний кварц */
                | PLLCFGR_M(4U)
                | PLLCFGR_N(168U)
                | PLLCFGR_P(4U)
                | PLLCFGR_Q(7U);

    /* 5–8: без изменений (латентность, включение, переключение) */
    FLASH_ACR = ACR_PRFTEN | ACR_ICEN | ACR_DCEN | ACR_LATENCY(2U);

    RCC_CR |= CR_PLLON;
    while (!(RCC_CR & CR_PLLRDY)) {
    }

    RCC_CFGR = (RCC_CFGR & ~0x3UL) | CFGR_SW_PLL;
    while ((RCC_CFGR & CFGR_SWS_MASK) != CFGR_SWS_PLL) {
    }

    RCC_CFGR = (RCC_CFGR & ~(0xFUL << 4)) | CFGR_HPRE_DIV1;
    RCC_CFGR = (RCC_CFGR & ~(0x7UL << 10)) | CFGR_PPRE1_DIV2;
    RCC_CFGR = (RCC_CFGR & ~(0x7UL << 13)) | CFGR_PPRE2_DIV1;
}

uint32_t clock_cpu_hz(void)
{
    return SYSCLK_HZ;
}

uint32_t clock_sysclk_source(void)
{
    uint32_t sws = (RCC_CFGR & CFGR_SWS_MASK);
    if (sws == CFGR_SWS_PLL) return 2U;   /* PLL */
    if (sws == 0x4UL)        return 1U;   /* HSE */
    if (sws == 0x8UL)        return 3U;   /* PLL (альтернативный код) */
    return 0U;                             /* HSI */
}

