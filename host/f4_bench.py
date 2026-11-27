#!/usr/bin/env python3
"""Стенд F4: арбитраж шин и детектор обгона.

Режимы:
  base   - UART-конвейер без фоновой нагрузки
  load   - запустить фоновое чтение SPI, затем UART-конвейер
  stall  - STALL(ms) + залп кадров -> обгон кольца
  stats  - RXSTATS / TXSTATS / SPISTATS

Примеры:
  python f4_bench.py -p COM5 base  --n 400 --window 32
  python f4_bench.py -p COM5 load  --n 400 --window 32 --len 262144
  python f4_bench.py -p COM5 stall --ms 200 --n 200
  python f4_bench.py -p COM5 stats
  python f4_bench.py -p COM5 -b 460800 ping
  python f4_bench.py -p COM5 -b 460800 ping --ms 10
  python f4_bench.py -p COM5 -b 460800 clock
  python f4_bench.py -p COM5 -b 460800 sysinfo
"""

import argparse
import random
import struct
import threading
import time

import serial

SYNC = 0xA5
ECHO, STATS, TXSTATS, RXSTATS, STALL, SPISTATS, LOAD, SYSINFO, PING, CLOCK_INFO = 1, 2, 3, 4, 5, 0x14, 0x15, 0x16, 0x20, 0x21


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


class Reader(threading.Thread):
    def __init__(self, s):
        super().__init__(daemon=True)
        self.s = s
        self.frames = []
        self.lock = threading.Lock()
        self.stop = False

    def run(self):
        buf = bytearray()
        while not self.stop:
            n = self.s.in_waiting
            chunk = self.s.read(n if n else 1)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                i = buf.find(bytes([SYNC]))
                if i < 0:
                    buf.clear()
                    break
                if i:
                    del buf[:i]
                if len(buf) < 4:
                    break
                ln = struct.unpack('<H', buf[1:3])[0]
                tot = 6 + ln
                if len(buf) < tot:
                    break
                body = bytes(buf[1:4 + ln])
                rx = struct.unpack('<H', buf[4 + ln:6 + ln])[0]
                if crc16(body) == rx:
                    with self.lock:
                        self.frames.append((buf[3], bytes(buf[4:4 + ln])))
                del buf[:tot]

    def count(self):
        with self.lock:
            return len(self.frames)

    def snap(self):
        with self.lock:
            return list(self.frames)


def call(s, cmd, payload=b'', timeout=3.0):
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

def mode_ping(s, ms):
    r = call(s, PING, struct.pack('<I', ms))
    if not r or len(r[1]) < 8:
        print('нет ответа на PING')
        return
    t0, t1 = struct.unpack('<II', r[1][:8])
    dt = (t1 - t0) & 0xFFFFFFFF

    print(f'задержка:           {ms} мс')
    print(f'тактов за задержку: {dt}')

    if ms == 0:
        print(f'overhead кадра: {dt} тактов')
        return

    measured_hz = dt / (ms / 1000.0)
    mhz = measured_hz / 1e6
    print(f'измеренная частота: {mhz:.3f} МГц')

    if 83.5 <= mhz <= 84.5:
        print('OK: частота в пределах 84 МГц +/- 0.5%')
    elif 15.5 <= mhz <= 16.5:
        print('FAIL: работает на HSI, PLL не захватил')
    else:
        print(f'WARN: частота вне ожиданий ({mhz:.1f} МГц)')


def mode_clock(s):
    r = call(s, CLOCK_INFO)
    if not r or len(r[1]) < 4:
        print('нет ответа на CLOCK_INFO')
        return
    src = struct.unpack('<I', r[1][:4])[0]
    names = {0: 'HSI (внутренний)', 1: 'HSE (внешний кварц)', 2: 'PLL'}
    print(f'источник тактов: {names.get(src, f"неизвестно ({src})")}')
    if src == 2:
        print('OK: система работает от PLL')
    elif src == 0:
        print('WARN: PLL не включён или не захватил частоту')

def show_stats(s):
    r = call(s, RXSTATS)
    if r and len(r[1]) == 24:
        w, c, laps, lost, head, tail = struct.unpack('<6I', r[1])
        print(f'RX : written={w} consumed={c} laps={laps} lost={lost} head={head} tail={tail}')
    r = call(s, TXSTATS)
    if r and len(r[1]) == 16:
        f, tc, mq, st = struct.unpack('<4I', r[1])
        print(f'TX : frames={f} tc={tc} maxq={mq} stall={st}')
    r = call(s, SPISTATS)
    if r and len(r[1]) == 28:
        d, e, ovr, te, dme, fe, tmo = struct.unpack('<7I', r[1])
        print(f'SPI: done={d} err={e} | ovr={ovr} te={te} dme={dme} fe={fe} tmo={tmo}')


def run_pipe(s, n, window, maxlen, seed=42):
    rnd = random.Random(seed)
    sent = []
    for i in range(n):
        ln = rnd.randint(4, maxlen)
        sent.append(struct.pack('<I', i) + bytes(rnd.randrange(256) for _ in range(ln - 4)))
    frames = [frame(ECHO, p) for p in sent]

    rd = Reader(s)
    rd.start()
    t0 = time.time()
    idx = 0
    last_cnt = 0
    last_rx = t0

    while True:
        cnt = rd.count()
        if cnt >= n:
            break
        if cnt != last_cnt:
            last_cnt = cnt
            last_rx = time.time()
        if (time.time() - last_rx) > 1.5:      # поток иссяк — не ждём дедлайна
            break
        if (time.time() - t0) > 30.0:
            break
        k = min(window - (idx - cnt), n - idx)
        if k > 0:
            for _ in range(k):
                s.write(frames[idx])
                idx += 1
        else:
            time.sleep(0)

    s.flush()
    time.sleep(0.2)
    rd.stop = True
    rd.join(timeout=1.0)

    got = rd.snap()
    dt = max(last_rx - t0, 1e-6)               # время до ПОСЛЕДНЕГО ответа
    ok = sum(1 for k, (c, p) in enumerate(got) if c == 0x81 and k < n and p == sent[k])
    wire = 2 * sum(6 + len(p) for p in sent)

    print(f'PIPE: recv={len(got)}/{n} positional_ok={ok} '
          f'time={dt:.2f}s fps={len(got) / dt:.1f} rate={wire / dt / 1024:.1f} KiB/s'
          f'{"  [ПОТЕРИ]" if len(got) < n else ""}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=460800)
    ap.add_argument('mode', choices=['load', 'stall', 'stats', 'sysinfo', 'ping', 'clock', 'pipe'])
    ap.add_argument('--n', type=int, default=400)
    ap.add_argument('--window', type=int, default=32)
    ap.add_argument('--maxlen', type=int, default=300)
    ap.add_argument('--len', type=int, default=262144)
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x100000)
    ap.add_argument('--ms', type=int, default=200)
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.05)
    if a.mode == 'sysinfo':
        r = call(s, SYSINFO)
        if r and len(r[1]) == 80:
            (ver, uw, uc, ulaps, ulost, uore, uidle,
             txf, txmq, txst,
             spid, spiovr, spite, spidme, spife, spitmo,
             fok, fcrc, fskip, fmax) = struct.unpack('<20I', r[1])
            print(f'PROJECT  : v{ver:#010x}')
            print(f'UART RX  : written={uw} consumed={uc} laps={ulaps} lost={ulost} ore={uore} idle={uidle}')
            print(f'UART TX  : frames={txf} maxq={txmq} stall={txst}')
            print(f'SPI      : done={spid} ovr={spiovr} te={spite} dme={spidme} fe={spife} tmo={spitmo}')
            print(f'FRAME    : ok={fok} crc_err={fcrc} skipped={fskip} maxlen={fmax}')
            print('INVARIANT written==consumed:', 'OK' if uc == uw else 'FAIL')
    elif a.mode == 'ping':
        mode_ping(s, a.ms)
    elif a.mode == 'clock':
        mode_clock(s)
    elif a.mode == 'stats':
        show_stats(s)
    elif a.mode == 'pipe':
        run_pipe(s, a.n, a.window, a.maxlen)
        show_stats(s)
    elif a.mode == 'load':
        call(s, LOAD, struct.pack('<II', a.addr, a.len))
        run_pipe(s, a.n, a.window, a.maxlen)
        show_stats(s)
    else:
        rd = Reader(s)
        rd.start()
        s.write(frame(STALL, struct.pack('<I', a.ms)))
        rnd = random.Random(1)
        sent = [struct.pack('<I', i) + bytes(rnd.randrange(256) for _ in range(150))
                for i in range(a.n)]
        for p in sent:
            s.write(frame(ECHO, p))
        s.flush()
        time.sleep(1.5)
        rd.stop = True
        rd.join(timeout=1.0)
        got = rd.snap()
        echo = [p for c, p in got if c == 0x81]
        print(f'STALL {a.ms} ms: sent={a.n} echo_recv={len(echo)}')
        show_stats(s)

    s.close()


if __name__ == '__main__':
    main()