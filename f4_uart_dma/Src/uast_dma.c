#include "uart_dma.h"

/* ----- RCC ----- */
#define RCC_BASE      0x40023800UL
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

/* ----- GPIOA ----- */
#define GPIOA_BASE    0x40020000UL
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))
#define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + 0x08UL))
#define GPIOA_AFRH    (*(volatile uint32_t *)(GPIOA_BASE + 0x24UL))

/* ----- USART1 ----- */
#define USART1_BASE   0x40011000UL
#define USART1_SR     (*(volatile uint32_t *)(USART1_BASE + 0x00UL))
#define USART1_DR     (*(volatile uint32_t *)(USART1_BASE + 0x04UL))
#define USART1_BRR    (*(volatile uint32_t *)(USART1_BASE + 0x08UL))
#define USART1_CR1    (*(volatile uint32_t *)(USART1_BASE + 0x0CUL))
#define USART1_CR3    (*(volatile uint32_t *)(USART1_BASE + 0x14UL))

#define USART_SR_TXE  (1UL << 7)
#define USART_SR_IDLE (1UL << 4)
#define USART_SR_ORE  (1UL << 3)

#define USART_CR1_RE     (1UL << 2)
#define USART_CR1_TE     (1UL << 3)
#define USART_CR1_IDLEIE (1UL << 4)
#define USART_CR1_UE     (1UL << 13)
#define USART_CR3_DMAR   (1UL << 6)
#define USART_CR3_DMAT   (1UL << 7)

/* ----- DMA2 ----- */
#define DMA2_BASE    0x40026400UL
#define DMA2_HISR    (*(volatile uint32_t *)(DMA2_BASE + 0x04UL))
#define DMA2_HIFCR   (*(volatile uint32_t *)(DMA2_BASE + 0x0CUL))

#define DMA2_SxCR(x)   (*(volatile uint32_t *)(DMA2_BASE + 0x10UL + 0x18UL * (x)))
#define DMA2_SxNDTR(x) (*(volatile uint32_t *)(DMA2_BASE + 0x14UL + 0x18UL * (x)))
#define DMA2_SxPAR(x)  (*(volatile uint32_t *)(DMA2_BASE + 0x18UL + 0x18UL * (x)))
#define DMA2_SxM0AR(x) (*(volatile uint32_t *)(DMA2_BASE + 0x1CUL + 0x18UL * (x)))

#define DMA_CR_EN      (1UL << 0)
#define DMA_CR_TCIE    (1UL << 4)
#define DMA_CR_DIR_M2P (1UL << 6)
#define DMA_CR_CIRC    (1UL << 8)
#define DMA_CR_MINC    (1UL << 10)
#define DMA_CR_PSIZE8  (0UL << 11)
#define DMA_CR_MSIZE8  (0UL << 13)
#define DMA_CR_PL(n)   (((uint32_t)(n)) << 16)
#define DMA_CR_CHSEL(c) (((uint32_t)(c)) << 25)

#define DMA5_TCIF  (1UL << 11)
#define DMA5_HTIF  (1UL << 10)
#define DMA5_TEIF  (1UL << 9)
#define DMA5_DMEIF (1UL << 8)
#define DMA5_FEIF  (1UL << 6)
#define DMA5_ALL   (DMA5_TCIF | DMA5_HTIF | DMA5_TEIF | DMA5_DMEIF | DMA5_FEIF)

#define DMA7_TCIF  (1UL << 27)
#define DMA7_HTIF  (1UL << 26)
#define DMA7_TEIF  (1UL << 25)
#define DMA7_DMEIF (1UL << 24)
#define DMA7_FEIF  (1UL << 22)
#define DMA7_ALL   (DMA7_TCIF | DMA7_HTIF | DMA7_TEIF | DMA7_DMEIF | DMA7_FEIF)

#define DMA5_STREAM 5
#define DMA5_CHANNEL 4
#define DMA7_STREAM 7
#define DMA7_CHANNEL 4

#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104UL)
#define NVIC_ISER2 (*(volatile uint32_t *)0xE000E108UL)
#define USART1_IRQN       37
#define DMA2_STREAM5_IRQN 68
#define DMA2_STREAM7_IRQN 70

/* ----- Состояние ----- */
static volatile uint8_t  rx_ring[UART_DMA_RX_RING];
static volatile uint32_t rx_tail = 0U;
static uart_dma_stats_t  g_stats;

static volatile uint32_t g_laps = 0U;
static volatile uint32_t g_lost = 0U;

typedef struct { const uint8_t *p; uint16_t n; } tx_desc_t;
static uint8_t           tx_buf[UART_DMA_TX_SLOTS][UART_DMA_TX_SLOT];
static tx_desc_t         txq[UART_DMA_TX_SLOTS];
static volatile uint32_t tx_head = 0U;
static volatile uint32_t tx_tail = 0U;
static volatile uint32_t tx_busy = 0U;
static uart_tx_stats_t   tx_st;

/* ---------- RX ---------- */

uint32_t uart_dma_head(void)
{
    uint32_t ndtr = DMA2_SxNDTR(DMA5_STREAM);
    return (UART_DMA_RX_RING - ndtr) & UART_DMA_RX_MASK;
}

uint32_t uart_dma_tail(void) { return rx_tail; }

uint32_t uart_dma_available(void)
{
    return (uart_dma_head() - rx_tail) & UART_DMA_RX_MASK;
}

uint8_t uart_dma_at(uint32_t idx)
{
    return rx_ring[idx & UART_DMA_RX_MASK];
}

void uart_dma_drop(uint32_t n)
{
    rx_tail = (rx_tail + n) & UART_DMA_RX_MASK;
    g_stats.rx_bytes += n;
}

/* Согласованное «сколько записал DMA».
 * W3-25: если TCIF взведён, но ISR ещё не инкрементировал dma_tc,
 * оборот учитываем вручную — иначе w занижен на RING. */
uint32_t uart_dma_written(void)
{
    uint32_t tc, pend, ndtr;

    do {
        tc   = g_stats.dma_tc;
        pend = (DMA2_HISR & DMA5_TCIF) ? 1U : 0U;
        ndtr = DMA2_SxNDTR(DMA5_STREAM);
    } while (tc != g_stats.dma_tc);

    return (tc + pend) * UART_DMA_RX_RING
           + ((UART_DMA_RX_RING - ndtr) & UART_DMA_RX_MASK);
}

uint32_t uart_dma_check_lap(void)
{
    uint32_t w = uart_dma_written();

    /* W3-25: ЗНАКОВОЕ сравнение. Заниженный снимок даёт отрицательный used,
     * который в uint32 превращался в ~4e9 и порождал ложный обгон. */
    int32_t used = (int32_t)(w - g_stats.rx_bytes);

    if (used > (int32_t)UART_DMA_RX_RING) {
        g_laps++;
        g_lost += (uint32_t)used;
        rx_tail = w & UART_DMA_RX_MASK;
        g_stats.rx_bytes = w;
        return (uint32_t)used;
    }
    return 0U;
}

uint32_t uart_dma_laps(void) { return g_laps; }
uint32_t uart_dma_lost(void) { return g_lost; }

void uart_dma_init(uint32_t brr)
{
    RCC_AHB1ENR |= (1UL << 0) | (1UL << 22);
    RCC_APB2ENR |= (1UL << 4);
    __asm volatile ("dsb sy" ::: "memory");

    GPIOA_MODER   = (GPIOA_MODER & ~(0xFUL << 18)) | (0xAUL << 18);
    GPIOA_OSPEEDR |= (0xFUL << 18);
    GPIOA_AFRH    = (GPIOA_AFRH & ~(0xFFUL << 4)) | (0x77UL << 4);

    USART1_BRR = brr;
    USART1_CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    USART1_CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE;
    USART1_CR1 |= USART_CR1_UE;

    DMA2_SxCR(DMA5_STREAM) = 0UL;
    while (DMA2_SxCR(DMA5_STREAM) & DMA_CR_EN) {
    }
    DMA2_HIFCR = DMA5_ALL;

    for (uint32_t i = 0; i < UART_DMA_RX_RING; i++) {
        rx_ring[i] = 0U;
    }
    rx_tail = 0U;

    DMA2_SxPAR(DMA5_STREAM)  = (uint32_t)&USART1_DR;
    DMA2_SxM0AR(DMA5_STREAM) = (uint32_t)rx_ring;
    DMA2_SxNDTR(DMA5_STREAM) = UART_DMA_RX_RING;
    DMA2_SxCR(DMA5_STREAM) = DMA_CR_CIRC | DMA_CR_MINC
                              | DMA_CR_PSIZE8 | DMA_CR_MSIZE8
                              | DMA_CR_PL(UART_RX_PL)
                              | DMA_CR_CHSEL(DMA5_CHANNEL)
                              | DMA_CR_TCIE;
    DMA2_SxCR(DMA5_STREAM) |= DMA_CR_EN;

    NVIC_ISER1 = (1UL << (USART1_IRQN - 32U));
    NVIC_ISER2 = (1UL << (DMA2_STREAM5_IRQN - 64U));
}

int uart_dma_getc(void)
{
    if (uart_dma_available() == 0U) {
        return -1;
    }
    uint8_t b = uart_dma_at(rx_tail);
    uart_dma_drop(1U);
    return (int)b;
}

void uart_dma_putc(char c)
{
    while (!(USART1_SR & USART_SR_TXE)) {
    }
    USART1_DR = (uint32_t)(uint8_t)c;
}

void uart_dma_puts(const char *s)
{
    while (*s) {
        uart_dma_putc(*s++);
    }
}

const uart_dma_stats_t *uart_dma_get_stats(void) { return &g_stats; }

/* ---------- TX ---------- */

static void tx_start(void)
{
    uint32_t i = tx_tail & (UART_DMA_TX_SLOTS - 1U);
    tx_busy = 1U;

    DMA2_SxCR(DMA7_STREAM) = 0UL;
    while (DMA2_SxCR(DMA7_STREAM) & DMA_CR_EN) {
    }
    DMA2_HIFCR = DMA7_ALL;

    DMA2_SxPAR(DMA7_STREAM)  = (uint32_t)&USART1_DR;
    DMA2_SxM0AR(DMA7_STREAM) = (uint32_t)txq[i].p;
    DMA2_SxNDTR(DMA7_STREAM) = txq[i].n;
    DMA2_SxCR(DMA7_STREAM) = DMA_CR_DIR_M2P | DMA_CR_MINC
                              | DMA_CR_PSIZE8 | DMA_CR_MSIZE8
                              | DMA_CR_PL(UART_TX_PL)
                              | DMA_CR_CHSEL(DMA7_CHANNEL)
                              | DMA_CR_TCIE;
    DMA2_SxCR(DMA7_STREAM) |= DMA_CR_EN;
}

void uart_dma_tx_init(void)
{
    tx_head = 0U; tx_tail = 0U; tx_busy = 0U;
    tx_st.frames = 0U; tx_st.tc = 0U; tx_st.maxq = 0U; tx_st.stall = 0U;

    DMA2_SxCR(DMA7_STREAM) = 0UL;
    while (DMA2_SxCR(DMA7_STREAM) & DMA_CR_EN) {
    }
    DMA2_HIFCR = DMA7_ALL;
    NVIC_ISER2 = (1UL << (DMA2_STREAM7_IRQN - 64U));
}

uint32_t uart_dma_tx_free(void)
{
    return UART_DMA_TX_SLOTS - (tx_head - tx_tail);
}

uint8_t *uart_dma_tx_alloc(void)
{
    if (uart_dma_tx_free() == 0U) {
        tx_st.stall++;
        return (uint8_t *)0;
    }
    return tx_buf[tx_head & (UART_DMA_TX_SLOTS - 1U)];
}

void uart_dma_tx_commit(uint16_t n)
{
    uint32_t prim;
    __asm volatile ("mrs %0, primask" : "=r" (prim));
    __asm volatile ("cpsid i" ::: "memory");

    uint32_t i = tx_head & (UART_DMA_TX_SLOTS - 1U);
    txq[i].p = tx_buf[i];
    txq[i].n = n;
    tx_head++;

    uint32_t q = tx_head - tx_tail;
    if (q > tx_st.maxq) {
        tx_st.maxq = q;
    }
    if (!tx_busy) {
        tx_start();
    }

    __asm volatile ("msr primask, %0" :: "r" (prim) : "memory");
}

const uart_tx_stats_t *uart_dma_tx_stats(void) { return &tx_st; }

/* ---------- ISR ---------- */

void DMA2_Stream5_IRQHandler(void)
{
    uint32_t hisr = DMA2_HISR;
    DMA2_HIFCR = (hisr & DMA5_ALL);
    g_stats.rx_isr_calls++;
    if (hisr & DMA5_TCIF)
    {
        g_stats.dma_tc++;
    }
}


void DMA2_Stream7_IRQHandler(void)
{
    uint32_t hisr = DMA2_HISR;
    DMA2_HIFCR = (hisr & DMA7_ALL);
    if (hisr & DMA7_TCIF) {
        tx_st.frames++;
        tx_st.tc++;
        tx_tail++;
        tx_busy = 0U;
        if (tx_tail != tx_head) {
            tx_start();
        }
    }
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1_SR;
    if (sr & USART_SR_IDLE) {
        (void)USART1_DR;
        g_stats.usart_idle++;
        if (sr & USART_SR_ORE) {
            g_stats.usart_ore++;
        }
    }
}
