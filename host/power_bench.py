#!/usr/bin/env python3
"""Стенд Sleep and Stop: режимы пониженного потребления.

Режимы:
  sleep  - Sleep на N мс, сравнение DWT и настенного времени
  stop   - Stop + пробуждение по EXTI (UART-активность), проверка восстановления тактов
  stats  - счётчики режимов
  verify - PING: подтверждение 84 МГц после пробуждения

Примеры:
  python power_bench.py -p COM5 sleep --ms 500
  python power_bench.py -p COM5 stop  --ms 500
  python power_bench.py -p COM5 stats
"""

import argparse
import struct
import time

import serial

SYNC = 0xA5
CMD_PING, CMD_SLEEP, CMD_STOP, CMD_PWRSTATS = 0x20, 0x27, 0x28, 0x29


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


def read_frame(s, timeout=3.0):
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


def send(s, cmd, payload=b''):
    s.write(frame(cmd, payload))
    s.flush()


def mode_sleep(s, ms):
    s.reset_input_buffer()
    t0 = time.time()
    send(s, CMD_SLEEP, struct.pack('<I', ms))
    r = read_frame(s, timeout=ms / 1000.0 + 3.0)
    wall_ms = (time.time() - t0) * 1000
    if not r or len(r[1]) < 8:
        print('SLEEP: нет ответа')
        return
    cycles, ack_ms = struct.unpack('<II', r[1][:8])
    dwt_ms = cycles / 84000.0
    print(f'SLEEP запрошено {ms} мс, подтверждено {ack_ms} мс')
    print(f'  настенное время: {wall_ms:.1f} мс')
    print(f'  DWT насчитал:    {cycles} тактов = {dwt_ms:.2f} мс @84МГц')
    ratio = dwt_ms / wall_ms if wall_ms > 0 else 0
    print(f'  DWT/настенное:   {ratio:.3f}')
    if ratio > 0.8:
        print('  => DWT СЧИТАЕТ во время Sleep')
    else:
        print('  => DWT НЕ считает во время Sleep')


def mode_stop(s, ms):
    s.reset_input_buffer()
    t0 = time.time()
    send(s, CMD_STOP)

    time.sleep(ms / 1000.0)              # устройство спит, тактов нет

    # Преамбула-пробуждение: фронты на PA10 будят EXTI.
    s.write(b'\x00\x00\x00\x00')
    s.flush()
    time.sleep(0.05)                     # восстановление тактов

    r = read_frame(s, timeout=3.0)
    wall_ms = (time.time() - t0) * 1000
    if not r or len(r[1]) < 8:
        print('STOP: нет ответа после пробуждения')
        return
    cycles, src = struct.unpack('<II', r[1][:8])
    dwt_ms = cycles / 16000.0     # после Stop ядро на HSI 16 МГц до clock_init()
    names = {0: 'HSI', 1: 'HSE', 2: 'PLL'}
    print(f'STOP спали ~{ms} мс')
    print(f'  настенное время:   {wall_ms:.1f} мс')
    print(f'  DWT насчитал:      {cycles} тактов = {dwt_ms:.3f} мс')
    print(f'  источник тактов:   {names.get(src, src)}')
    print(f'  разрыв оси:        {wall_ms - dwt_ms:.1f} мс потеряно для DWT')
    print(f'  DWT (пробуждение): {cycles} тактов = {cycles / 16.0:.2f} мкс @HSI16')
    if src == 2:
        print('  => такты восстановлены (PLL), clock_init() сработал')
    else:
        print('  => FAIL: такты не восстановлены')


def mode_verify(s):
    send(s, CMD_PING, struct.pack('<I', 10))
    r = read_frame(s, timeout=3.0)
    if not r or len(r[1]) < 8:
        print('PING: нет ответа')
        return
    t0, t1 = struct.unpack('<II', r[1][:8])
    mhz = ((t1 - t0) & 0xFFFFFFFF) / 10000.0
    print(f'PING 10 мс: {mhz:.3f} МГц')


def mode_stats(s):
    send(s, CMD_PWRSTATS)
    r = read_frame(s, timeout=3.0)
    if not r or len(r[1]) < 28:
        print('PWRSTATS: нет ответа')
        return
    sl, st, we, cr, dt, lw, rf = struct.unpack('<7I', r[1])
    print(f'PWR: sleep={sl} stop={st} wake_exti={we} clock_restores={cr}')
    print(f'     drain_timeouts={dt} last_wake={lw} cycles ({lw / 84.0:.1f} мкс) rx_flushed={rf}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=460800)
    ap.add_argument('mode', choices=['sleep', 'stop', 'stats', 'verify'])
    ap.add_argument('--ms', type=int, default=500)
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.05)

    if a.mode == 'sleep':
        mode_sleep(s, a.ms)
    elif a.mode == 'stop':
        mode_stop(s, a.ms)
    elif a.mode == 'verify':
        mode_verify(s)
    else:
        mode_stats(s)

    s.close()


if __name__ == '__main__':
    main()