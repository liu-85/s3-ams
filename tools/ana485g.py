# -*- coding: utf-8 -*-
"""提取我方 Dxx 应答（0xE0 类）的 use_flag / pressure / meters(float) / 料位"""
import re, struct

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[.*?\]\s+([0-9A-Fa-f]+)")

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    try:
        b = bytes.fromhex(m.group(5))
    except ValueError:
        continue
    frames.append((m.group(1), int(m.group(3)), m.group(2), b))

print("=== 我方 Dxx 应答字段时序（只取 len>=45 且 b[1] 在 C0~C7）===")
print("  时间        t      use_flag pressure  meters(m)   料位")
prev = None
cnt = 0
for t, ms, d, b in frames:
    if d != "TX" or len(b) < 45:
        continue
    if not (0xC0 <= b[1] <= 0xC7):
        continue
    use_flag = b[19]
    pressure = b[25] | (b[26] << 8)
    meters = struct.unpack('<f', b[21:25])[0]
    ch = b[20]
    key = (use_flag, pressure, round(meters, 4), ch)
    if key == prev:
        continue
    prev = key
    print("  [%s] t=%6d  use=0x%02X  p=0x%04X  meters=%10.4f  料位=%02X"
          % (t, ms, use_flag, pressure, meters, ch))
    cnt += 1
    if cnt > 80:
        print("  ...")
        break
