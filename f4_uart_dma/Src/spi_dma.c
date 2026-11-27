/*
 * SPI1 + DMA2 полный дуплекс.
 *
 * W3-19: самовосстановление по времени в spi1_busy(), обработка OVR через
 *        ERRIE/SPI1_IRQHandler, ограниченные циклы EN/BSY.
 * W3-20: раздельные счётчики источников; FEIF в direct mode — информационный
 *        (транзакцию не рвём); снятие stale pending SPI-IRQ перед стартом.
 */

#include "spi_dma.h"

/* ----- RCC ----- */
#define RCC_BASE      0x40023800UL
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

/* ----- GPIOB ----- */
#define GPIOB_BASE    0x40020400UL
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))
#define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x0CUL))
#define GPIOB_BSRR    (*(volatile uint32_t *)(GPIOB_BASE + 0x18UL))
#define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + 0x20UL))

/* ----- SPI1 ----- */
#define SPI1_BASE     0x40013000UL
#define SPI1_CR1      (*(volatile uint32_t *)(SPI1_BASE + 0x00UL))
#define SPI1_CR2      (*(volatile uint32_t *)(SPI1_BASE + 0x04UL))
#define SPI1_SR       (*(volatile uint32_t *)(SPI1_BASE + 0x08UL))
#define SPI1_DR       (*(volatile uint32_t *)(SPI1_BASE + 0x0CUL))

#define SPI_CR1_MSTR  (1UL << 2)
#define SPI_CR1_BR(n) (((uint32_t)(n)) << 3)
#define SPI_CR1_SPE   (1UL << 6)
#define SPI_CR1_SSI   (1UL << 8)
#define SPI_CR1_SSM   (1UL << 9)

#define SPI_CR2_RXDMAEN (1UL << 0)
#define SPI_CR2_TXDMAEN (1UL << 1)
#define SPI_CR2_ERRIE   (1UL << 5)

#define SPI_SR_OVR    (1UL << 6)
#define SPI_SR_BSY    (1UL << 7)

/* ----- DMA2 ----- */
#define DMA2_BASE    0x40026400UL
#define DMA2_LISR    (*(volatile uint32_t *)(DMA2_BASE + 0x00UL))
#define DMA2_LIFCR   (*(volatile uint32_t *)(DMA2_BASE + 0x08UL))

#define DMA2_SxCR(x)   (*(volatile uint32_t *)(DMA2_BASE + 0x10UL + 0x18UL * (x)))
#define DMA2_SxNDTR(x) (*(volatile uint32_t *)(DMA2_BASE + 0x14UL + 0x18UL * (x)))
#define DMA2_SxPAR(x)  (*(volatile uint32_t *)(DMA2_BASE + 0x18UL + 0x18UL * (x)))
#define DMA2_SxM0AR(x) (*(volatile uint32_t *)(DMA2_BASE + 0x1CUL + 0x18UL * (x)))

#define DMA_CR_EN      (1UL << 0)
#define DMA_CR_TCIE    (1UL << 4)
#define DMA_CR_TEIE    (1UL << 2)
#define DMA_CR_DMEIE   (1UL << 1)
#define DMA_CR_DIR_M2P (1UL << 6)
#define DMA_CR_MINC    (1UL << 10)
#define DMA_CR_PSIZE8  (0UL << 11)
#define DMA_CR_MSIZE8  (0UL << 13)
#define DMA_CR_PL(n)   (((uint32_t)(n)) << 16)
#define DMA_CR_CHSEL(c) (((uint32_t)(c)) << 25)

/* Stream0 -> LISR слот 0 */
#define DMA0_TCIF  (1UL << 5)
#define DMA0_HTIF  (1UL << 4)
#define DMA0_TEIF  (1UL << 3)
#define DMA0_DMEIF (1UL << 2)
#define DMA0_FEIF  (1UL << 0)
#define DMA0_ALL   (DMA0_TCIF | DMA0_HTIF | DMA0_TEIF | DMA0_DMEIF | DMA0_FEIF)

/* Stream3 -> LISR слот 3 */
#define DMA3_TCIF  (1UL << 27)
#define DMA3_HTIF  (1UL << 26)
#define DMA3_TEIF  (1UL << 25)
#define DMA3_DMEIF (1UL << 24)
#define DMA3_FEIF  (1UL << 22)
#define DMA3_ALL   (DMA3_TCIF | DMA3_HTIF | DMA3_TEIF | DMA3_DMEIF | DMA3_FEIF)

#define SPI_RX_STREAM 0
#define SPI_TX_STREAM 3
#define SPI_CHANNEL   3

/* ----- NVIC ----- */
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104UL)
#define NVIC_ICPR1 (*(volatile uint32_t *)0xE000E284UL)

#define DMA2_STREAM0_IRQN 56
#define SPI1_IRQN         35

/* ----- Временные границы (DWT) ----- */
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define CPU_HZ     16000000UL
#define MS(n)      ((uint32_t)(n) * (CPU_HZ / 1000UL))

#define T_XFER_MAX  MS(50)    /* 1024 Б @2 МГц = 4.1 мс, запас 12x */
#define T_EN_CLR    MS(1)
#define T_BSY       MS(1)

/* ----- Состояние ----- */
static volatile uint32_t g_busy = 0U;
static volatile uint8_t  g_release = 1U;
static volatile uint32_t g_done = 0U;
static volatile uint32_t g_t0 = 0U;

static volatile uint32_t g_ovr = 0U;
static volatile uint32_t g_te  = 0U;
static volatile uint32_t g_dme = 0U;
static volatile uint32_t g_fe  = 0U;   /* информационный */
static volatile uint32_t g_tmo = 0U;

void spi1_cs_low(void)  { GPIOB_BSRR = (1UL << (0U + 16U)); }
void spi1_cs_high(void) { GPIOB_BSRR = (1UL << 0U); }

uint32_t spi1_done_count(void) { return g_done; }
uint32_t spi1_err_count(void)  { return g_ovr + g_te + g_dme + g_tmo; }

void spi1_err_detail(uint32_t *ovr, uint32_t *te, uint32_t *dme,
                     uint32_t *fe, uint32_t *tmo)
{
    *ovr = g_ovr;
    *te  = g_te;
    *dme = g_dme;
    *fe  = g_fe;
    *tmo = g_tmo;
}

/* Аварийное завершение: остановить потоки, сбросить OVR, отпустить CS. */
static void spi_abort(void)
{
    if (!g_busy) {
        return;
    }

    DMA2_SxCR(SPI_RX_STREAM) = 0UL;
    DMA2_SxCR(SPI_TX_STREAM) = 0UL;

    uint32_t t = DWT_CYCCNT;
    while ((DMA2_SxCR(SPI_RX_STREAM) | DMA2_SxCR(SPI_TX_STREAM)) & DMA_CR_EN) {
        if ((DWT_CYCCNT - t) > T_EN_CLR) {
            break;
        }
    }

    DMA2_LIFCR = DMA0_ALL | DMA3_ALL;
    (void)SPI1_DR;
    (void)SPI1_SR;

    spi1_cs_high();
    g_busy = 0U;
}

void spi1_dma_init(uint32_t br)
{
    RCC_AHB1ENR |= (1UL << 1) | (1UL << 22);   /* GPIOB, DMA2 */
    RCC_APB2ENR |= (1UL << 12);                /* SPI1 */
    __asm volatile ("dsb sy" ::: "memory");

    /* PB0 = CS: выход, pull-up, неактивен */
    GPIOB_MODER = (GPIOB_MODER & ~3UL) | 1UL;
    GPIOB_PUPDR = (GPIOB_PUPDR & ~3UL) | 1UL;
    spi1_cs_high();

    /* PB3/4/5 = AF5 */
    GPIOB_MODER   = (GPIOB_MODER & ~(0x3FUL << 6)) | (0x2AUL << 6);
    GPIOB_OSPEEDR |= (0x3FUL << 6);
    GPIOB_AFRL    = (GPIOB_AFRL & ~0x00FFF000UL) | 0x00555000UL;

    SPI1_CR1 = 0UL;
    SPI1_CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_ERRIE;
    SPI1_CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
             | SPI_CR1_BR(br) | SPI_CR1_SPE;

    (void)SPI1_DR;
    (void)SPI1_SR;

    NVIC_ISER1 = (1UL << (DMA2_STREAM0_IRQN - 32U))
               | (1UL << (SPI1_IRQN - 32U));
}

int spi1_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint8_t release_cs)
{
    if ((len == 0U) || (tx == 0) || (rx == 0)) {
        return -1;
    }

    /* Ожидание предыдущей транзакции ограничено самовосстановлением */
    while (spi1_busy()) {
    }

    g_release = release_cs;
    g_t0 = DWT_CYCCNT;
    g_busy = 1U;

    DMA2_SxCR(SPI_RX_STREAM) = 0UL;
    DMA2_SxCR(SPI_TX_STREAM) = 0UL;

    uint32_t t = DWT_CYCCNT;
    while ((DMA2_SxCR(SPI_RX_STREAM) | DMA2_SxCR(SPI_TX_STREAM)) & DMA_CR_EN) {
        if ((DWT_CYCCNT - t) > T_EN_CLR) {
            break;
        }
    }
    DMA2_LIFCR = DMA0_ALL | DMA3_ALL;

    (void)SPI1_DR;                            /* сброс OVR: DR, затем SR */
    (void)SPI1_SR;
    NVIC_ICPR1 = (1UL << (SPI1_IRQN - 32U));  /* W3-20: снять stale pending */

    /* RX включаем ПЕРВЫМ */
    DMA2_SxPAR(SPI_RX_STREAM)  = (uint32_t)&SPI1_DR;
    DMA2_SxM0AR(SPI_RX_STREAM) = (uint32_t)rx;
    DMA2_SxNDTR(SPI_RX_STREAM) = len;
    DMA2_SxCR(SPI_RX_STREAM) = DMA_CR_MINC | DMA_CR_PSIZE8 | DMA_CR_MSIZE8
                                | DMA_CR_PL(3) | DMA_CR_CHSEL(SPI_CHANNEL)
                                | DMA_CR_TCIE | DMA_CR_TEIE | DMA_CR_DMEIE;
    DMA2_SxCR(SPI_RX_STREAM) |= DMA_CR_EN;

    DMA2_SxPAR(SPI_TX_STREAM)  = (uint32_t)&SPI1_DR;
    DMA2_SxM0AR(SPI_TX_STREAM) = (uint32_t)tx;
    DMA2_SxNDTR(SPI_TX_STREAM) = len;
    DMA2_SxCR(SPI_TX_STREAM) = DMA_CR_DIR_M2P | DMA_CR_MINC
                                | DMA_CR_PSIZE8 | DMA_CR_MSIZE8
                                | DMA_CR_PL(2) | DMA_CR_CHSEL(SPI_CHANNEL);
    DMA2_SxCR(SPI_TX_STREAM) |= DMA_CR_EN;

    return 0;
}

int spi1_busy(void)
{
    if (g_busy && ((DWT_CYCCNT - g_t0) > T_XFER_MAX)) {
        g_tmo++;
        spi_abort();
        return 0;
    }
    return (int)g_busy;
}

/* ---------- ISR ---------- */

void DMA2_Stream0_IRQHandler(void)
{
    uint32_t lisr = DMA2_LISR;

    DMA2_LIFCR = (lisr & DMA0_ALL);

    /* FEIF в direct mode — информационный: транзакцию не рвём */
    if (lisr & DMA0_FEIF) {
        g_fe++;
    }

    if (lisr & (DMA0_TEIF | DMA0_DMEIF)) {
        if (lisr & DMA0_TEIF)  { g_te++; }
        if (lisr & DMA0_DMEIF) { g_dme++; }
        spi_abort();
        return;
    }

    if (lisr & DMA0_TCIF) {
        DMA2_SxCR(SPI_TX_STREAM) = 0UL;

        uint32_t t = DWT_CYCCNT;
        while (SPI1_SR & SPI_SR_BSY) {
            if ((DWT_CYCCNT - t) > T_BSY) {
                break;
            }
        }

        if (g_release) {
            spi1_cs_high();
        }
        g_busy = 0U;
        g_done++;
    }
}

void SPI1_IRQHandler(void)
{
    uint32_t sr = SPI1_SR;

    if (sr & SPI_SR_OVR) {
        (void)SPI1_DR;
        (void)SPI1_SR;
        g_ovr++;
        spi_abort();
    }
}
