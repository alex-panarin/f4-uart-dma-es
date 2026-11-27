#include "rtc.h"
#include "clock.h"

/* ----- RCC / PWR ----- */
#define RCC_BASE     0x40023800UL
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x40UL))
#define RCC_BDCR     (*(volatile uint32_t *)(RCC_BASE + 0x70UL))
#define RCC_CSR      (*(volatile uint32_t *)(RCC_BASE + 0x74UL))

#define PWR_CR       (*(volatile uint32_t *)0x40007000UL)
#define PWR_CR_DBP   (1UL << 8)

#define BDCR_LSEON   (1UL << 0)
#define BDCR_LSERDY  (1UL << 1)
#define BDCR_RTCSEL_MASK (0x3UL << 8)
#define BDCR_RTCSEL_LSE  (0x1UL << 8)
#define BDCR_RTCSEL_LSI  (0x2UL << 8)
#define BDCR_RTCEN   (1UL << 15)

#define CSR_LSION    (1UL << 0)
#define CSR_LSIRDY   (1UL << 1)

/* ----- RTC ----- */

#define RTC_TR       (*(volatile uint32_t *)(RTC_BASE + 0x00UL))
#define RTC_DR       (*(volatile uint32_t *)(RTC_BASE + 0x04UL))

#define RTC_ISR      (*(volatile uint32_t *)(RTC_BASE + 0x0CUL))
#define RTC_PRER     (*(volatile uint32_t *)(RTC_BASE + 0x10UL))
#define RTC_WPR      (*(volatile uint32_t *)(RTC_BASE + 0x24UL))
#define RTC_SSR      (*(volatile uint32_t *)(RTC_BASE + 0x28UL))
#define RTC_BKP0R    (*(volatile uint32_t *)(RTC_BASE + 0x50UL))
#define RTC_BKP1R    (*(volatile uint32_t *)(RTC_BASE + 0x54UL))

#define ISR_WUTWF    (1UL << 2)
#define ISR_RSF      (1UL << 5)
#define ISR_INITF    (1UL << 6)
#define ISR_INIT     (1UL << 7)
#define ISR_WUTF     (1UL << 10)

#define CR_WUCKSEL_MASK 0x7UL
#define CR_WUCKSEL_1HZ  0x4UL     /* ck_spre, период = WUT+1 секунд */
#define CR_WUTE      (1UL << 10)
#define CR_WUTIE     (1UL << 14)

/* ----- EXTI line 22 (RTC wakeup) ----- */
#define EXTI_IMR     (*(volatile uint32_t *)0x40013C00UL)
#define EXTI_RTSR    (*(volatile uint32_t *)0x40013C08UL)
#define EXTI_PR      (*(volatile uint32_t *)0x40013C14UL)

#define NVIC_ISER0   (*(volatile uint32_t *)0xE000E100UL)
#define RTC_WKUP_IRQN 3

#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004UL)
#define MS_CYC(n)    ((uint32_t)(n) * (HCLK_HZ / 1000U))

#define LSE_TIMEOUT_MS 2000U
#define LSI_TIMEOUT_MS 100U
#define RSF_TIMEOUT_MS 100U
#define INIT_TIMEOUT_MS 100U

#define BKUP_MAGIC   0x5A5A0022UL

static rtc_stats_t g_rs;
static uint32_t g_prs = 127U;      /* для пересчёта sub_ms */

uint32_t rtc_source(void) { return g_rs.source; }
const rtc_stats_t *rtc_stats(void) { return &g_rs; }

static void wpr_unlock(void)
{
    RTC_WPR = 0xCAU;
    RTC_WPR = 0x53U;
}

static void wpr_lock(void)
{
    RTC_WPR = 0xFFU;
}

static uint8_t bcd2bin(uint32_t v)
{
    return (uint8_t)(((v >> 4) & 0xFU) * 10U + (v & 0xFU));
}

static uint32_t bin2bcd(uint8_t v)
{
    return (uint32_t)(((v / 10U) << 4) | (v % 10U));
}

/* Синхронизация к границе секунды: иначе первый интервал короче до 1 с.
 * SSR — вычитающий счётчик; перезагрузка (cur > prev) = прошёл тик.
 * Читать SSR обязательно вместе с DR, иначе теневые регистры
 * останутся заблокированы и SSR не обновится. */
void rtc_wait_rsf(void)
{
    uint32_t prev = RTC_SSR;
    (void)RTC_DR;
    uint32_t ts = DWT_CYCCNT;
    while (1) {
        uint32_t cur = RTC_SSR;
        (void)RTC_DR;
        if (cur > prev) {
            break;
        }
        prev = cur;
        if ((DWT_CYCCNT - ts) > MS_CYC(1500U)) {
            break;
        }
    }
}

void rtc_init(void)
{
    RCC_APB1ENR |= (1UL << 28);            /* PWR */
    __asm volatile ("dsb sy" ::: "memory");
    PWR_CR |= PWR_CR_DBP;                  /* доступ к backup-домену */
    __asm volatile ("dsb sy" ::: "memory");

    /* Время уже идёт и магия на месте — не сбрасываем часы. */
    if ((RTC_BKP0R == BKUP_MAGIC) &&
        (RCC_BDCR & BDCR_RTCEN) &&
        (RCC_BDCR & BDCR_LSERDY)) {
        g_rs.source = RTC_SRC_LSE;
        g_rs.init_skipped++;
        RTC_BKP1R = RTC_BKP1R + 1U;
        g_rs.boots = RTC_BKP1R;
        wpr_unlock();
        RTC_CR |= (1UL << 5);              /* BYPSHAD: без теневой синхронизации */
        wpr_lock();
        return;
    }

    /* --- LSE с таймаутом --- */
    uint32_t t0 = DWT_CYCCNT;
    RCC_BDCR |= BDCR_LSEON;
    while (!(RCC_BDCR & BDCR_LSERDY)) {
        if ((DWT_CYCCNT - t0) > MS_CYC(LSE_TIMEOUT_MS)) {
            g_rs.lse_timeouts++;
            break;
        }
    }

    uint32_t sel;
    if (RCC_BDCR & BDCR_LSERDY)
    {
        g_rs.source = RTC_SRC_LSE;
        sel = BDCR_RTCSEL_LSE;
        g_prs = 127U;                      /* PRL=255, PRS=127 -> 1 Гц */
    }
    else
    {
        /* Fallback: LSI ~32 кГц, точность плохая — сообщаем флагом. */
        RCC_CSR |= CSR_LSION;
        t0 = DWT_CYCCNT;
        while (!(RCC_CSR & CSR_LSIRDY)) {
            if ((DWT_CYCCNT - t0) > MS_CYC(LSI_TIMEOUT_MS)) {
                break;
            }
        }
        if (!(RCC_CSR & CSR_LSIRDY)) {
            g_rs.source = RTC_SRC_NONE;
            return;
        }
        g_rs.source = RTC_SRC_LSI;
        sel = BDCR_RTCSEL_LSI;
        g_prs = 124U;                      /* PRL=255, PRS=124 -> ~1 Гц */
    }

    RCC_BDCR = (RCC_BDCR & ~BDCR_RTCSEL_MASK) | sel;
    RCC_BDCR |= BDCR_RTCEN;
    __asm volatile ("dsb sy" ::: "memory");

    /* --- инициализация RTC --- */
    wpr_unlock();

    RTC_ISR = ISR_INIT;
    t0 = DWT_CYCCNT;
    while (!(RTC_ISR & ISR_INITF)) {
        if ((DWT_CYCCNT - t0) > MS_CYC(INIT_TIMEOUT_MS)) {
            break;
        }
    }

    /* PRS всего 7 бит: 255 сюда НЕ помещается. PRL=255, PRS=g_prs. */
    RTC_PRER = ((uint32_t)255U) | ((uint32_t)g_prs << 16);

    /* 2000-01-01 00:00:00, суббота (WDU=6) */
    RTC_TR = 0UL;
    RTC_DR = (bin2bcd(0U) << 16) | (bin2bcd(1U) << 8) | bin2bcd(1U) | (6UL << 13);

    RTC_CR = (RTC_CR & ~CR_WUCKSEL_MASK) | (1UL << 5);

    RTC_ISR = 0UL;                         /* выход из init */
    wpr_lock();

    RTC_BKP0R = BKUP_MAGIC;
    RTC_BKP1R = RTC_BKP1R + 1U;
    g_rs.boots = RTC_BKP1R;
    g_rs.init_done++;

    rtc_wait_rsf();
}

int rtc_set(const rtc_time_t *t)
{
    if ((t->mm < 1U) || (t->mm > 12U) || (t->dd < 1U) || (t->dd > 31U) ||
        (t->hh > 23U) || (t->mi > 59U) || (t->ss > 59U)) {
        return -1;
    }

    wpr_unlock();
    RTC_ISR = ISR_INIT;

    uint32_t t0 = DWT_CYCCNT;
    while (!(RTC_ISR & ISR_INITF)) {
        if ((DWT_CYCCNT - t0) > MS_CYC(INIT_TIMEOUT_MS)) {
            RTC_ISR = 0UL;
            wpr_lock();
            return -1;
        }
    }

    RTC_TR = (bin2bcd(t->hh) << 16) | (bin2bcd(t->mi) << 8) | bin2bcd(t->ss);
    RTC_DR = (bin2bcd(t->yy) << 16) | (bin2bcd(t->mm) << 8) | bin2bcd(t->dd)
             | (6UL << 13);

    RTC_ISR = 0UL;
    wpr_lock();

    return 0;
}

void rtc_get(rtc_time_t *t)
{
    uint32_t tr1, dr1, tr2, dr2, ssr2;
    uint32_t guard = 0U;

    /* BYPSHAD=1: читаем напрямую, двойное чтение до совпадения TR/DR. */
    do {
        tr1  = RTC_TR;
        dr1  = RTC_DR;
        ssr2 = RTC_SSR;
        tr2  = RTC_TR;
        dr2  = RTC_DR;
        if (++guard > 8U) {
            break;
        }
    } while ((tr1 != tr2) || (dr1 != dr2));

    t->hh = bcd2bin((tr2 >> 16) & 0x3FU);
    t->mi = bcd2bin((tr2 >> 8) & 0x7FU);
    t->ss = bcd2bin(tr2 & 0x7FU);
    t->yy = bcd2bin((dr2 >> 16) & 0xFFU);
    t->mm = bcd2bin((dr2 >> 8) & 0x1FU);
    t->dd = bcd2bin(dr2 & 0x3FU);

    if (ssr2 > g_prs) {
        ssr2 = g_prs;
    }
    t->sub_ms = (uint16_t)(((g_prs - ssr2) * 1000U) / (g_prs + 1U));
}

static uint32_t is_leap(uint32_t y)
{
    return (((y % 4U) == 0U) && ((y % 100U) != 0U)) || ((y % 400U) == 0U);
}

uint32_t rtc_epoch(const rtc_time_t *t)
{
    static const uint16_t md[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    uint32_t y = 2000U + t->yy;
    uint32_t days = 0U;

    for (uint32_t k = 2000U; k < y; k++) {
        days += is_leap(k) ? 366U : 365U;
    }
    if ((t->mm >= 1U) && (t->mm <= 12U)) {
        days += md[t->mm - 1U];
        if ((t->mm > 2U) && is_leap(y)) {
            days += 1U;
        }
    }
    days += (t->dd > 0U) ? (t->dd - 1U) : 0U;

    return days * 86400U + (uint32_t)t->hh * 3600U + (uint32_t)t->mi * 60U + t->ss;
}

int rtc_wut_arm(uint32_t seconds)
{
    uint32_t wutr, sel, half;

    if ((seconds == 0U) || (seconds > 65536U)) {
        g_rs.wut_errors++;
        return -1;
    }

    /* Период = WUT+2 тактов ck_wut (загрузка на первом тике). */
    if (seconds <= 4U)
    {
    	half = (g_rs.source == RTC_SRC_LSE) ? 16384U : 16000U;
    	sel  = 3U;                          /* RTCCLK/2 */
    	wutr = seconds * half - 1U;
    }
    else
    {
    	sel  = 4U;                          /* ck_spre 1 Гц */
    	wutr = seconds - 1U;
    }

    wpr_unlock();
    RTC_CR &= ~(CR_WUTE | CR_WUTIE);

    uint32_t t0 = DWT_CYCCNT;
    while (!(RTC_ISR & ISR_WUTWF)) {
        if ((DWT_CYCCNT - t0) > MS_CYC(INIT_TIMEOUT_MS)) {
            g_rs.wut_errors++;
            wpr_lock();
            return -1;
        }
    }

    RTC_WUTR = wutr;
    RTC_CR = (RTC_CR & ~CR_WUCKSEL_MASK) | sel;
    RTC_ISR = (RTC_ISR & ~(ISR_WUTF | ISR_INIT));
    RTC_CR |= (CR_WUTE | CR_WUTIE);
    wpr_lock();

    EXTI_RTSR |= (1UL << 22);
    EXTI_IMR  |= (1UL << 22);
    EXTI_PR    = (1UL << 22);
    NVIC_ISER0 = (1UL << RTC_WKUP_IRQN);
    return 0;
}

void rtc_sync_tick(void)
{
    rtc_wait_rsf();
    uint32_t prev = RTC_SSR;
    (void)RTC_DR;
    uint32_t ts = DWT_CYCCNT;
    while (1) {
        uint32_t cur = RTC_SSR;
        (void)RTC_DR;
        if (cur > prev) {
            break;                      /* перезагрузка SSR = тик прошёл */
        }
        prev = cur;
        if ((DWT_CYCCNT - ts) > MS_CYC(1500U)) {
            break;
        }
    }
}

void rtc_wut_disarm(void)
{
    wpr_unlock();
    RTC_CR &= ~(CR_WUTE | CR_WUTIE);
    wpr_lock();
    RTC_ISR = (RTC_ISR & ~(ISR_WUTF | ISR_INIT));
    EXTI_IMR &= ~(1UL << 22);
}

/* ---------- ISR ---------- */

void RTC_WKUP_IRQHandler(void)
{
    RTC_ISR = (RTC_ISR & ~(ISR_WUTF | ISR_INIT));
    EXTI_PR = (1UL << 22);
    g_rs.wakeups++;
}
