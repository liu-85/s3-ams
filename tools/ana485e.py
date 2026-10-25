# -*- coding: utf-8 -*-
"""分析 485 日志：0xC5 控制帧的状态码时序 + 我方应答 pressure 反推内部状态"""
import re, sys, collections

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[.*?\]\s+([0-9A-Fa-f]+)")

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    t, d, ms, ln, hexs = m.group(1), m.group(2), int(m.group(3)), int(m.group(4)), m.group(5)
    try:
        b = bytes.fromhex(hexs)
    except ValueError:
        continue
    frames.append((t, ms, d, ln, b))

print("总帧数:", len(frames))
rx = [f for f in frames if f[2] == "RX"]
tx = [f for f in frames if f[2] == "TX"]
print("RX(打印机→AMS):", len(rx), " TX(AMS→打印机):", len(tx))

# ---- 1. 0xC5 控制帧的 (statu, motion, read) 分布 ----
print("\n=== RX 0xC5 控制帧分布 (statu, motion, read) ===")
cnt = collections.Counter()
for t, ms, d, ln, b in rx:
    if len(b) >= 11 and b[1] == 0xC5:
        cnt[(b[6], b[7], b[9])] += 1
for (st, mo, rd), v in sorted(cnt.items(), key=lambda x: -x[1]):
    print("  statu=%02X motion=%02X read=%02X   x%d" % (st, mo, rd, v))

# ---- 2. 0xC5 帧的完整时间线（压缩连续相同帧）----
print("\n=== 0xC5 状态变化时间线（相同状态只记首次与末次）===")
seq = []
for t, ms, d, ln, b in frames:
    if len(b) >= 11 and b[1] == 0xC5:
        key = (d, b[5], b[6], b[7], b[9])
        if not seq or seq[-1][0] != key:
            seq.append([key, t, ms, 1])
        else:
            seq[-1][3] += 1
            seq[-1][1] = t
            seq[-1][2] = ms
for key, t, ms, n in seq:
    d, ams, st, mo, rd = key
    print("  [%s] %s t=%d ams=%02X statu=%02X motion=%02X read=%02X  x%d"
          % (t, d, ms, ams, st, mo, rd, n))

# ---- 3. 我方应答 pressure 反推内部 motion ----
# pressure 在 Dxx_res+25 (2字节) / Cxx_res+13
PMAP = {0x4700: "send_out(送料)", 0x2B00: "on_use/idle(待机)", 0xFFFF: "无耗材"}

print("\n=== 我方应答 pressure 时间线 ===")
prev = None
for t, ms, d, ln, b in tx:
    if len(b) < 30:
        continue
    # 找 pressure：Dxx 应答 len 通常 30+，pressure 在 offset 25
    if b[1] in (0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7):
        p = b[25] | (b[26] << 8)
        if p in (0x4700, 0x2B00, 0xFFFF):
            if p != prev:
                print("  [%s] t=%d pressure=0x%04X  → %s" % (t, ms, p, PMAP.get(p, "?")))
                prev = p

# ---- 4. 每次 09/A5 之后到下一个状态帧的间隔 ----
print("\n=== before_on_use(09/A5) 之后的状态帧（含间隔）===")
last_a5 = None
n = 0
for t, ms, d, ln, b in frames:
    if len(b) < 11 or b[1] != 0xC5:
        continue
    st, mo = b[6], b[7]
    if st == 0x09 and mo == 0xA5:
        last_a5 = ms
        print("  [%s] t=%d  ← 09/A5 before_on_use" % (t, ms))
        n += 1
        if n > 12:
            break
    elif last_a5 is not None and ms - last_a5 < 15000 and (st, mo) != (0x09, 0xA5):
        print("  [%s] t=%d  (+%dms)  statu=%02X motion=%02X read=%02X" % (t, ms, ms - last_a5, st, mo, b[9]))
        n += 1
        if n > 12:
            break
