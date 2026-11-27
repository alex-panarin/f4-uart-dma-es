#!/usr/bin/env python3
"""Стенд Flash: W25Qxx через SPI+DMA, надёжная пакетная запись с ретраями.

Режимы:
  id       - JEDEC ID
  prepare  - надёжная запись: сессия + окно + FSYNC-битмап + ретраи
  bench    - чтение+CRC на устройстве (blocking и overlap)
  fstatus  - счётчики записи флеша

Примеры:
  python flash_bench.py -p COM5 id
  python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8
  python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8 --simulate-loss 5
  python flash_bench.py -p COM5 bench   --addr 0x100000 --size 32768
"""

import argparse
import struct
import threading
import time

import serial

SYNC = 0xA5
CMD_JEDEC, CMD_FERASE, CMD_FWRITE, CMD_FBENCH, CMD_FSTATUS = 0x10, 0x11, 0x12, 0x13, 0x23
CMD_FSESSION, CMD_FPAGE, CMD_FSYNC = 0x24, 0x25, 0x26


def _tbl():
    t = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        t.append(c)
    return t


_T = _tbl()


def crc16(data):
    c = 0xFFFF
    for b in data:
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


def transact(s, cmd, payload=b'', timeout=3.0):
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


def pattern(size, seed=7):
    out = bytearray()
    x = seed & 0xFF
    for _ in range(size):
        x = (x * 31 + 7) & 0xFF
        out.append(x)
    return bytes(out)


def do_prepare(s, addr, size, window, do_erase=True, max_attempts=3, simulate_loss=0):
    data = pattern(size)
    npages = (size + 255) // 256
    bitmap_bytes = (npages + 7) // 8
    t_all = time.time()

    # --- сессия: базовый адрес, число страниц, стирание ---
    t0 = time.time()
    r = transact(s, CMD_FSESSION,
                 struct.pack('<IHB', addr, npages, 1 if do_erase else 0),
                 timeout=15.0)
    if not r or r[1][0] != 0:
        print('FSESSION fail')
        return False
    setup_ms = (time.time() - t0) * 1000
    print(f'SESSION: base={addr:#x} pages={npages} erase={do_erase} setup={setup_ms:.0f} ms')

    # --- кадры всех страниц заранее ---
    frames = []
    for q in range(npages):
        off = q * 256
        frames.append(frame(CMD_FPAGE, struct.pack('<H', q) + data[off:off + 256]))

    def get_bitmap():
        rr = transact(s, CMD_FSYNC, b'', timeout=3.0)
        if rr and len(rr[1]) >= 2 + bitmap_bytes:
            return rr[1][2:2 + bitmap_bytes]
        return None

    def is_done(bm, q):
        return (bm[q >> 3] >> (q & 7)) & 1

    def send_batch(order):
        rd = Reader(s)
        rd.start()
        idx = 0
        last_cnt = 0
        last_rx = time.time()
        while idx < len(order):
            cnt = rd.count()
            if cnt != last_cnt:
                last_cnt = cnt
                last_rx = time.time()
            if time.time() - last_rx > 2.0:
                break
            inflight = idx - cnt
            k = min(window - inflight, len(order) - idx)
            if k > 0:
                for _ in range(k):
                    s.write(frames[order[idx]])
                    idx += 1
                s.flush()
            else:
                time.sleep(0)
        s.flush()
        time.sleep(0.1)
        rd.stop = True
        rd.join(timeout=1.0)
        return idx

    # --- цикл попыток: послать незаписанные -> срезать битмап -> повторить ---
    pending = list(range(npages))
    for attempt in range(1, max_attempts + 1):
        if not pending:
            break
        batch = pending
        if attempt == 1 and simulate_loss > 0:
            batch = pending[:-simulate_loss]   # имитация потери запросов
        sent = send_batch(batch)
        time.sleep(0.05)
        bm = get_bitmap()
        if bm is None:
            print(f'  attempt {attempt}: FSYNC fail')
            continue
        pending = [q for q in range(npages) if not is_done(bm, q)]
        print(f'  attempt {attempt}: sent={sent} done={npages - len(pending)}/{npages} pending={len(pending)}')

    bm = get_bitmap()
    if bm is None:
        print('final FSYNC fail')
        return False
    done = sum(1 for q in range(npages) if is_done(bm, q))
    total_ms = (time.time() - t_all) * 1000
    print(f'WRITE: {done}/{npages} confirmed, TOTAL {total_ms:.0f} ms, host crc16={crc16(data):#06x}')
    return done == npages


def do_bench(s, addr, size):
    data = pattern(size)
    want = crc16(data)
    print(f'host crc16 = {want:#06x}, size={size}')
    for mode, name in ((0, 'blocking'), (1, 'overlap ')):
        r = transact(s, CMD_FBENCH, struct.pack('<IIB', addr, size, mode), timeout=10.0)
        got, us, bps = struct.unpack('<HII', r[1])
        print(f'{name}: crc={got:#06x} {"OK" if got == want else "MISMATCH"} '
              f'time={us / 1000:.2f} ms rate={bps / 1024:.1f} KiB/s')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=460800)
    ap.add_argument('mode', choices=['id', 'prepare', 'bench', 'fstatus'])
    ap.add_argument('--addr', type=lambda x: int(x, 0), default=0x100000)
    ap.add_argument('--size', type=int, default=32768)
    ap.add_argument('--window', type=int, default=8)
    ap.add_argument('--no-erase', action='store_true')
    ap.add_argument('--max-attempts', type=int, default=3)
    ap.add_argument('--simulate-loss', type=int, default=0)
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.1)

    if a.mode == 'id':
        r = transact(s, CMD_JEDEC)
        print(f'JEDEC = {struct.unpack("<I", r[1])[0]:#010x}')
    elif a.mode == 'prepare':
        ok = do_prepare(s, a.addr, a.size, a.window,
                        do_erase=not a.no_erase,
                        max_attempts=a.max_attempts,
                        simulate_loss=a.simulate_loss)
        if ok:
            print('--- verify (read-back) ---')
            do_bench(s, a.addr, a.size)
        else:
            print('PREPARE FAILED')
    elif a.mode == 'bench':
        do_bench(s, a.addr, a.size)
    else:
        r = transact(s, CMD_FSTATUS)
        if r and len(r[1]) >= 8:
            ok, fail = struct.unpack('<II', r[1][:8])
            print(f'FLASH: fw_ok={ok} fw_fail={fail}')

    s.close()


if __name__ == '__main__':
    main()