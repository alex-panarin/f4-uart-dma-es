#include "power.h"
#include "clock.h"
#include "uart_dma.h"
#include "spi_dma.h"

/* ----- PWR ----- */
#define PWR_BASE     0x40007000UL
#define PWR_CR       (*(volatile uint32_t *)(PWR_BASE + 0x00UL))
#define PWR_CSR      (*(volatile uint32_t *)(PWR_BASE + 0x04UL))

#define PWR_CR_LPDS  (1UL << 0)
#define PWR_CR_PDDS  (1UL << 1)
#define PWR_CR_CWUF  (1UL << 2)

/* ----- RCC ----- */
#define RCC_BASE     0x40023800UL
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x40UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

/* ----- SYSCFG / EXTI ----- */
#define SYSCFG_EXTICR3 (*(volatile uint32_t *)0x40013810UL)
#define EXTI_IMR       (*(volatile uint32_t *)0x40013C00UL)
#define EXTI_FTSR      (*(volatile uint32_t *)0x40013C0CUL)
#define EXTI_PR        (*(volatile uint32_t *)0x40013C14UL)

/* ----- Cortex-M4 системные ----- */
#define SCB_SCR      (*(volatile uint32_t *)0xE000ED10UL)
#define SCR_SLEEPDEEP (1UL << 2)

#define SYST_CSR     (*(volatile uint32_t *)0xE000E010UL)
#define SYST_RVR     (*(volatile uint32_t *)0xE000E014UL)
#define SYST_CVR     (*(volatile uint32_t *)0xE000E018UL)

#define NVIC_ISER1   (*(volatile uint32_t *)0xE000E104UL)
#define EXTI15_10_IRQN 40

#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004UL)

#define SYSTICK_HZ   (HCLK_HZ / 8U)     /* CLKSOURCE=0 => HCLK/8 */
#define DRAIN_MS     50U
#define MS_CYC(n)    ((uint32_t)(n) * (HCLK_HZ / 1000U))

static power_stats_t g_pw;
static volatile uint32_t g_exti_hits = 0U;

const power_stats_t *power_stats(void)
{
    return &g_pw;
}

void power_init(void)
{
    RCC_APB1ENR |= (1UL << 28);          /* PWR */
    RCC_APB2ENR |= (1UL << 14);          /* SYSCFG */
    __asm volatile ("dsb sy" ::: "memory");

    /* EXTI10 <- PA10 (значение 0 = порт A), поле EXTICR3[11:8] */
    SYSCFG_EXTICR3 = (SYSCFG_EXTICR3 & ~(0xFUL << 8)) | (0x0UL << 8);

    EXTI_FTSR |= (1UL << 10);            /* спадающий фронт: стартовый бит */
    EXTI_IMR  |= (1UL << 10);
    EXTI_PR    = (1UL << 10);

    NVIC_ISER1 = (1UL << (EXTI15_10_IRQN - 32U));
}

/* Дренаж: дождаться опустошения TX и завершения SPI, с границей по времени. */
static void drain(void)
{
    uint32_t t0 = DWT_CYCCNT;

    while (uart_dma_tx_free() != UART_DMA_TX_SLOTS) {
        if ((DWT_CYCCNT - t0) > MS_CYC(DRAIN_MS)) {
            g_pw.drain_timeouts++;
            break;
        }
    }
    while (spi1_busy()) {
        if ((DWT_CYCCNT - t0) > MS_CYC(DRAIN_MS)) {
            g_pw.drain_timeouts++;
            break;
        }
    }
}

static volatile uint32_t g_syst_done = 0U;

uint32_t power_sleep_ms(uint32_t ms)
{
    if (ms > 1500U) {
        ms = 1500U;
    }

    drain();
    g_pw.sleep_entries++;

    uint32_t load = ms * (SYSTICK_HZ / 1000U);
    if (load > 0U) {
        load -= 1U;
    }
    if (load > 0xFFFFFFUL) {
        load = 0xFFFFFFUL;
    }

    g_syst_done = 0U;
    SYST_CSR = 0UL;
    SYST_RVR = load;
    SYST_CVR = 0UL;
    SYST_CSR = (1UL << 0) | (1UL << 1);

    SCB_SCR &= ~SCR_SLEEPDEEP;

    uint32_t t0 = DWT_CYCCNT;

    /* WFI будит ЛЮБОЙ IRQ. Возвращаемся спать, пока SysTick не отработает. */
    do {
        __asm volatile ("dsb sy" ::: "memory");
        __asm volatile ("wfi");
        __asm volatile ("isb sy" ::: "memory");
    } while (!g_syst_done);

    uint32_t t1 = DWT_CYCCNT;
    SYST_CSR = 0UL;

    return (t1 - t0);
}


uint32_t power_stop(void)
{
    drain();
    g_pw.stop_entries++;

    uint32_t hits0 = g_exti_hits;

    PWR_CR |= PWR_CR_CWUF;               /* сброс флага пробуждения */
    PWR_CR &= ~PWR_CR_PDDS;              /* Stop, не Standby */
    PWR_CR |= PWR_CR_LPDS;               /* регулятор в маломощном режиме */

    EXTI_PR = (1UL << 10);               /* сброс отложенного запроса */

    SCB_SCR |= SCR_SLEEPDEEP;

    uint32_t t0 = DWT_CYCCNT;
    __asm volatile ("dsb sy" ::: "memory");
    __asm volatile ("wfi");
    __asm volatile ("isb sy" ::: "memory");
    uint32_t t1 = DWT_CYCCNT;            /* DWT в Stop не считает: это латентность */

    SCB_SCR &= ~SCR_SLEEPDEEP;
    PWR_CR &= ~PWR_CR_LPDS;

    g_pw.last_wake_cycles = (t1 - t0);
    if (g_exti_hits != hits0) {
        g_pw.wake_exti++;
    }

    /* Stop сбросил RCC на HSI: возвращаем 84 МГц и делители шин. */
    clock_init();
    g_pw.clock_restores++;

    /* Мусор на линии во время сна: отбрасываем непрочитанное. */
    uint32_t avail = uart_dma_available();
    if (avail > 0U) {
        uart_dma_drop(avail);
        g_pw.rx_flushed += avail;
    }

    return (t1 - t0);
}

/* ---------- ISR ---------- */

void SysTick_Handler(void)
{
    SYST_CSR = 0UL;
    g_syst_done = 1U;
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR & (1UL << 10)) {
        EXTI_PR = (1UL << 10);           /* сброс write-1-to-clear */
        g_exti_hits++;
    }
}
