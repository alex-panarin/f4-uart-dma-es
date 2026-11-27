# Оптимизация скорости и надежности работы при программировании внешней и внутренней Flash

1. Пакетное программирование флеша w25Q32 на F407 spi
2. надёжный транспорт на стороне устройства

## Ключевые решения

- каждая страница получает порядковый номер в запросе;
- устройство отвечает номером последней успешно записанной страницы;
- хост при расхождении повторно шлёт страницы, начиная с потерянной;
- идемпотентность гарантирована (повторная запись той же страницы безопасна).

## Идемпотентность

Повторная запись той же страницы теми же данными безопасна: программирование флеша только сбрасывает биты, а данные для данной страницы неизменны. Значит, повтор всегда корректен.

## Битмап

- источник истины. Ответы FPAGE нужны только для управления окном (чтобы не переполнить кольцо). Истинное состояние — в битмапе FSYNC. Это закрывает оба класса потерь:
потерян ответ → страница записана, битмап это подтвердит → повтор не нужен;
потерян запрос → страница не записана, битмап покажет 0 → хост пошлёт её снова.

## Ретрай по битмапу

Каждая попытка: послать все незаписанные страницы окном → запросить битмап → вычислить недостающие → повторить. Не нужно угадывать, что потерялось, — устройство само сообщает.

## Тестовые прогоны

python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8
SESSION: base=0x100000 pages=128 erase=True setup=226 ms
  attempt 1: sent=128 done=128/128 pending=0
WRITE: 128/128 confirmed, TOTAL 1170 ms, host crc16=0xafa3
--- verify (read-back) ---
host crc16 = 0xafa3, size=32768
blocking: crc=0xafa3 OK time=29.21 ms rate=1095.4 KiB/s
overlap : crc=0xafa3 OK time=25.43 ms rate=1258.3 KiB/s

python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8 --simulate-loss 5
SESSION: base=0x100000 pages=128 erase=True setup=226 ms
  attempt 1: sent=123 done=123/128 pending=5
  attempt 2: sent=5 done=128/128 pending=0
WRITE: 128/128 confirmed, TOTAL 1346 ms, host crc16=0xafa3
--- verify (read-back) ---
host crc16 = 0xafa3, size=32768
blocking: crc=0xafa3 OK time=29.21 ms rate=1095.4 KiB/s
overlap : crc=0xafa3 OK time=25.43 ms rate=1258.3 KiB/s

python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8
SESSION: base=0x100000 pages=128 erase=True setup=228 ms
  attempt 1: sent=128 done=127/128 pending=1
  attempt 2: sent=1 done=128/128 pending=0
WRITE: 128/128 confirmed, TOTAL 1331 ms, host crc16=0xafa3
--- verify (read-back) ---
host crc16 = 0xafa3, size=32768
blocking: crc=0xafa3 OK time=29.21 ms rate=1095.4 KiB/s
overlap : crc=0xafa3 OK time=25.43 ms rate=1258.3 KiB/s

python flash_bench.py -p COM5 prepare --addr 0x100000 --size 32768 --window 8
SESSION: base=0x100000 pages=128 erase=True setup=225 ms
  attempt 1: sent=128 done=128/128 pending=0
WRITE: 128/128 confirmed, TOTAL 1201 ms, host crc16=0xafa3
--- verify (read-back) ---
host crc16 = 0xafa3, size=32768
blocking: crc=0xafa3 OK time=29.21 ms rate=1095.4 KiB/s
overlap : crc=0xafa3 OK time=25.43 ms rate=1258.3 KiB/s
