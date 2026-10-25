# -*- coding: utf-8 -*-
"""只看 ams0 的动作类状态帧（0x03/0x07/0x09/0x0F），输出干净时间线"""
import re

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

MEAN = {
    (0x03, 0x00): "★送料 send_out",
    (0x09, 0xA5): "★before_on_use",
    (0x07, 0x7F): "★on_use 打印中",
    (0x07, 0x00): "★stop_on_use",
    (0x09, 0x3F): "★before_pull_back",
    (0x0F, 0xFF): "☆0F/FF",
    (0x0F, 0x00): "☆0F/00",
}

print("=== ams=00 动作帧时间线（去重连续相同 key）===")
prev = None
t0 = None
for t, ms, d, b in frames:
    if d != "RX" or len(b) < 11 or b[1] != 0xC5 or b[5] != 0x00:
        continue
    key = (b[6], b[7], b[9])
    label = MEAN.get((key[0], key[1]))
    if label is None:
        continue
    if key == prev:
        continue
    prev = key
    if t0 is None:
        t0 = ms
    print("  [%s] t=%6d (+%5.1fs)  statu=%02X motion=%02X read=%02X   %s"
          % (t, ms, (ms - t0) / 1000.0, key[0], key[1], key[2], label))
