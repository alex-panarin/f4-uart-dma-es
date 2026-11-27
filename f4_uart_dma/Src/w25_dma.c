#include "w25_dma.h"
#include "spi_dma.h"

#define CMD_JEDEC 0x9FU
#define CMD_RDSR  0x05U
#define CMD_WREN  0x06U
#define CMD_READ  0x03U
#define CMD_PP    0x02U
#define CMD_SE    0x20U

#define SR_WIP    0x01U
#define PAGE      256U

#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define CPU_HZ     16000000UL

#define T_PP_MS    50U
#define T_SE_MS    500U
#define T_IDLE_MS  500U

static uint8_t dummy_tx[W25_MAX_XFER];
static uint8_t dummy_rx[W25_MAX_XFER];

static void wait_idle(void)
{
    while (spi1_busy()) {
    }
}

void w25_init(uint32_t spi_br)
{
    spi1_dma_init(spi_br);
    for (uint32_t i = 0; i < W25_MAX_XFER; i++) {
        dummy_tx[i] = 0xFFU;
    }
    spi1_cs_high();
}

uint32_t w25_jedec(void)
{
    uint8_t tx[4] = { CMD_JEDEC, 0xFFU, 0xFFU, 0xFFU };
    uint8_t rx[4];

    spi1_cs_low();
    spi1_xfer(tx, rx, 4U, 1U);
    wait_idle();

    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
}

uint8_t w25_rdsr(void)
{
    uint8_t tx[2] = { CMD_RDSR, 0xFFU };
    uint8_t rx[2];

    spi1_cs_low();
    spi1_xfer(tx, rx, 2U, 1U);
    wait_idle();

    return rx[1];
}

static void w25_wren(void)
{
    uint8_t tx[1] = { CMD_WREN };

    spi1_cs_low();
    spi1_xfer(tx, dummy_rx, 1U, 1U);
    wait_idle();
}

/*
 * W3-17: ожидание ограничено ВРЕМЕНЕМ, а не числом итераций.
 * Подряд идущие 0xFF = шина не отвечает (MISO подтянут), а не Busy.
 * Возврат: 0 — готово, -1 — таймаут, -2 — шина не отвечает.
 */
static int wait_wip(uint32_t timeout_ms)
{
    uint32_t t0 = DWT_CYCCNT;
    uint32_t limit = timeout_ms * (CPU_HZ / 1000UL);
    uint32_t ff_run = 0U;

    do {
        uint8_t sr = w25_rdsr();

        if ((sr & SR_WIP) == 0U) {
            return 0;
        }
        if (sr == 0xFFU) {
            if (++ff_run >= 8U) {
                return -2;
            }
        } else {
            ff_run = 0U;
        }
    } while ((DWT_CYCCNT - t0) < limit);

    return -1;
}

static void put_addr(uint8_t *p, uint32_t addr)
{
    p[0] = (uint8_t)(addr >> 16);
    p[1] = (uint8_t)(addr >> 8);
    p[2] = (uint8_t)addr;
}

int w25_sector_erase(uint32_t addr)
{
    uint8_t tx[4];
    tx[0] = CMD_SE;
    put_addr(&tx[1], addr);

    if (wait_wip(T_IDLE_MS) != 0) {
        return -1;
    }
    w25_wren();

    spi1_cs_low();
    spi1_xfer(tx, dummy_rx, 4U, 1U);
    wait_idle();

    return wait_wip(T_SE_MS);
}

int w25_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t tx[4];

    if ((len == 0U) || (len > PAGE) || (((addr % PAGE) + len) > PAGE)) {
        return -1;
    }
    if (wait_wip(T_IDLE_MS) != 0) {
        return -1;
    }
    w25_wren();

    tx[0] = CMD_PP;
    put_addr(&tx[1], addr);

    spi1_cs_low();
    spi1_xfer(tx, dummy_rx, 4U, 0U);      /* CS держим */
    wait_idle();
    spi1_xfer(data, dummy_rx, len, 1U);
    wait_idle();

    return wait_wip(T_PP_MS);
}

int w25_read_async(uint32_t addr, uint8_t *dst, uint16_t len)
{
    uint8_t tx[4];

    if ((len == 0U) || (len > W25_MAX_XFER)) {
        return -1;
    }
    wait_idle();

    tx[0] = CMD_READ;
    put_addr(&tx[1], addr);

    spi1_cs_low();
    spi1_xfer(tx, dummy_rx, 4U, 0U);
    wait_idle();

    return spi1_xfer(dummy_tx, dst, len, 1U);
}

int w25_read(uint32_t addr, uint8_t *dst, uint16_t len)
{
    int rc = w25_read_async(addr, dst, len);
    wait_idle();
    return rc;
}
