#ifndef RTC_H
#define RTC_H

#include <stdint.h>

/*
 * RTC в backup-домене. Источник LSE 32.768 кГц, fallback LSI при отказе.
 * Время идёт в Stop/Standby и переживает NRST. При наличии батарейки на VBAT
 * переживает и снятие питания.
 *
 * Обязательные шаги после Stop: clock_init() (День 21) + rtc_wait_rsf().
 */

#define RTC_SRC_NONE 0U
#define RTC_SRC_LSE  1U
#define RTC_SRC_LSI  2U

#define RTC_BASE     0x40002800UL
#define RTC_CR       (*(volatile uint32_t *)(RTC_BASE + 0x08UL))
#define RTC_WUTR     (*(volatile uint32_t *)(RTC_BASE + 0x14UL))

typedef struct {
    uint8_t  yy;      /* 0..99, от 2000 */
    uint8_t  mm;      /* 1..12 */
    uint8_t  dd;      /* 1..31 */
    uint8_t  hh;      /* 0..23 */
    uint8_t  mi;      /* 0..59 */
    uint8_t  ss;      /* 0..59 */
    uint16_t sub_ms;  /* 0..999 */
} rtc_time_t;

typedef struct {
    uint32_t source;        /* RTC_SRC_* */
    uint32_t lse_timeouts;
    uint32_t init_done;
    uint32_t init_skipped;  /* время уже шло — не сбрасывали */
    uint32_t rsf_timeouts;
    uint32_t wakeups;
    uint32_t boots;         /* из backup-регистра */
    uint32_t wut_errors;
} rtc_stats_t;

void     rtc_init(void);
int      rtc_set(const rtc_time_t *t);
void     rtc_get(rtc_time_t *t);
uint32_t rtc_epoch(const rtc_time_t *t);   /* секунд от 2000-01-01 00:00:00 */
void     rtc_wait_rsf(void);
void	 rtc_sync_tick(void);

int  rtc_wut_arm(uint32_t seconds);        /* 1..65536 */
void rtc_wut_disarm(void);

uint32_t rtc_source(void);
const rtc_stats_t *rtc_stats(void);

#endif /* RTC_H */
