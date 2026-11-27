#!/usr/bin/env python3
"""Стенд RTC: RTC на LSE.

Режимы:
  stats     - источник RTC, счётчики инициализации и пробуждений
  sync      - установить время от хоста
  get       - прочитать время и сравнить с хостом
  stopwake  - Stop на N секунд по RTC-пробуждению: RTC идёт, DWT стоит
  drift     - дрейф RTC относительно хоста за N секунд

Примеры:
  python rtc_bench.py -p COM5 stats
  python rtc_bench.py -p COM5 sync
  python rtc_bench.py -p COM5 get
  python rtc_bench.py -p COM5 stopwake --secs 5
  python rtc_bench.py -p COM5 drift --secs 60
"""

import argparse
import struct
import time
from datetime import datetime

import serial

SYNC = 0xA5
CMD_RTC_SET, CMD_RTC_GET, CMD_RTC_STOPWAKE, CMD_RTC_STATS = 0x2A, 0x2B, 0x2C, 0x2D
EPOCH_2000 = 946684800          # unix time для 2000-01-01 00:00:00 UTC


def _tbl():
    t = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        t.append(c)
    return t


_T = _tbl()


def crc16(d):
    c = 0xFFFF
    for b in d:
        c = ((c << 8) & 0xFFFF) ^ _T[((c >> 8) ^ b) & 0xFF]
    return c


def frame(cmd, payload=b''):
    body = struct.pack('<H', len(payload)) + bytes([cmd]) + payload
    return bytes([SYNC]) + body + struct.pack('<H', crc16(body))


def call(s, cmd, payload=b'', timeout=5.0):
    s.reset_input_buffer()
    s.write(frame(cmd, payload))
    s.flush()
    buf = bytearray()
    end = time.time() + timeout
    while time.time() < end:
        b = s.read(s.in_waiting or 1)
        if b:
            buf.extend(b)
        i = buf.find(bytes([SYNC]))
        if i >= 0 and len(buf) >= i + 6:
            ln = struct.unpack('<H', buf[i + 1:i + 3])[0]
            if len(buf) >= i + 6 + ln:
                body = bytes(buf[i + 1:i + 4 + ln])
                rx = struct.unpack('<H', buf[i + 4 + ln:i + 6 + ln])[0]
                if crc16(body) == rx:
                    return buf[i + 3], bytes(buf[i + 4:i + 4 + ln])
                del buf[:i + 6 + ln]
    return None


SRC = {0: 'NONE', 1: 'LSE 32.768 kHz', 2: 'LSI ~32 kHz (неточный)'}


def read_time(s):
    r = call(s, CMD_RTC_GET)
    if not r or len(r[1]) < 16:
        return None
    yy, mm, dd, hh, mi, ss, sub, src, _, _, _, epoch = struct.unpack('<6BHB3BI', r[1][:16])
    return dict(yy=yy, mm=mm, dd=dd, hh=hh, mi=mi, ss=ss, sub=sub, src=src, epoch=epoch)


def mode_stats(s):
    r = call(s, CMD_RTC_STATS)
    if not r or len(r[1]) < 32:
        print('RTC_STATS: нет ответа')
        return
    src, lse_to, ini, skip, rsf, wake, boots, werr = struct.unpack('<8I', r[1])
    print(f'RTC source     : {SRC.get(src, src)}')
    print(f'lse_timeouts   : {lse_to}')
    print(f'init_done      : {ini}   init_skipped: {skip}')
    print(f'rsf_timeouts   : {rsf}')
    print(f'wakeups        : {wake}')
    print(f'boots (backup) : {boots}')
    print(f'wut_errors     : {werr}')


def mode_sync(s):
    now = datetime.now()
    payload = bytes([now.year - 2000, now.month, now.day, now.hour, now.minute, now.second])
    r = call(s, CMD_RTC_SET, payload)
    if not r or r[1][0] != 0:
        print('RTC_SET: отказ')
        return
    print(f'установлено: {now.strftime("%Y-%m-%d %H:%M:%S")}')
    t = read_time(s)
    if t:
        print(f'прочитано:   20{t["yy"]:02d}-{t["mm"]:02d}-{t["dd"]:02d} '
              f'{t["hh"]:02d}:{t["mi"]:02d}:{t["ss"]:02d}.{t["sub"]:03d}')


def mode_get(s):
    t0 = time.time()
    t = read_time(s)
    if not t:
        print('RTC_GET: нет ответа')
        return
    host = datetime.now()
    print(f'RTC : 20{t["yy"]:02d}-{t["mm"]:02d}-{t["dd"]:02d} '
          f'{t["hh"]:02d}:{t["mi"]:02d}:{t["ss"]:02d}.{t["sub"]:03d}  src={SRC.get(t["src"])}')
    print(f'HOST: {host.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}')
    rtc_unix = EPOCH_2000 + t['epoch']
    print(f'разница RTC-HOST (с учётом TZ не выравнивалась): {rtc_unix - int(t0)} с')


def mode_stopwake(s, secs):
    print(f'Stop на {secs} с по RTC-пробуждению...')
    t0 = time.time()
    r = call(s, CMD_RTC_STOPWAKE, struct.pack('<I', secs), timeout=secs + 10.0)
    wall = time.time() - t0
    if not r or len(r[1]) < 16:
        print('STOPWAKE: нет ответа')
        return
    e0, e1, cyc, src, wutr, cr, s1, s2, s3, s4 = struct.unpack('<10I', r[1][:40])
    overhead = (s1 + s2 + s3 + s4) / 1000.0
    print(f'  RTC: {e1 - e0} с   настенное: {wall:.3f} с   сон≈{wall - overhead:.3f} с')
    print(f'  этапы, мс: RSF+e0={s1}  arm={s2}  stop(без сна)={s3}  disarm+RSF={s4}')
    print(f'  WUTR={wutr}  WUCKSEL={cr & 7}  активное={overhead:.3f} с')
    #print(f'  => {"OK" if ok else "РАСХОЖДЕНИЕ"}')


def mode_drift(s, secs):
    a = read_time(s)
    h0 = time.time()
    if not a:
        print('нет ответа')
        return
    print(f'ждём {secs} с...')
    time.sleep(secs)
    b = read_time(s)
    h1 = time.time()
    if not b:
        print('нет ответа')
        return
    ta = a['epoch'] + a['sub'] / 1000.0
    tb = b['epoch'] + b['sub'] / 1000.0
    d_rtc = tb - ta
    d_host = h1 - h0
    err = d_rtc - d_host
    ppm = err / d_host * 1e6 if d_host > 0 else 0
    print(f'RTC прошло:  {d_rtc:.3f} с')
    print(f'HOST прошло: {d_host:.3f} с')
    print(f'ошибка:      {err:+.3f} с  ({ppm:+.0f} ppm)')
    print(f'разрешение субсекунд: {1000.0/128:.2f} мс => предел оценки '
          f'±{0.0078/d_host*1e6:.0f} ppm на этом интервале')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=460800)
    ap.add_argument('mode', choices=['stats', 'sync', 'get', 'stopwake', 'drift'])
    ap.add_argument('--secs', type=int, default=5)
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.05)
    if a.mode == 'stats':
        mode_stats(s)
    elif a.mode == 'sync':
        mode_sync(s)
    elif a.mode == 'get':
        mode_get(s)
    elif a.mode == 'stopwake':
        mode_stopwake(s, a.secs)
    else:
        mode_drift(s, a.secs)
    s.close()


if __name__ == '__main__':
    main()