# -*- coding: utf-8 -*-
"""打印某时间窗内所有 485 帧（RX/TX 交错），看完整对话"""
import re, sys

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[.*?\]\s+([0-9A-Fa-f]+)")

t_from = sys.argv[1] if len(sys.argv) > 1 else "15:06:08"
t_to   = sys.argv[2] if len(sys.argv) > 2 else "15:06:12"

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    try:
        b = bytes.fromhex(m.group(5))
    except ValueError:
        continue
    frames.append((m.group(1), int(m.group(3)), m.group(2), int(m.group(4)), b))

print("=== %s ~ %s 全部帧 ===" % (t_from, t_to))
for t, ms, d, ln, b in frames:
    if not (t_from <= t <= t_to):
        continue
    # 只关心 0xC5 类（短包头 3D Cx）
    if len(b) < 4 or b[0] != 0x3D:
        continue
    if not (0xC0 <= b[1] <= 0xE7):
        continue
    typ = b[4] if len(b) > 4 else 0
    # 短包 0xC5 = 运动帧
    if b[1] == 0xC5 and len(b) >= 11:
        st, mo, rd = b[6], b[7], b[9]
        tag = ""
        if st == 0x03: tag = "送料"
        elif st == 0x09 and mo == 0xA5: tag = "before_on_use"
        elif st == 0x09 and mo == 0x3F: tag = "before_pull_back"
        elif st == 0x07 and mo == 0x7F: tag = "on_use"
        elif st == 0x07 and mo == 0x00: tag = "stop_on_use"
        elif st == 0x0F and mo == 0xFF: tag = "★0F/FF"
        elif st == 0x0F and mo == 0x00: tag = "★0F/00"
        elif st == 0x01: tag = "空闲"
        print("  [%s] %s t=%6d len=%3d  type=%02X ams=%02X statu=%02X motion=%02X read=%02X  %s"
              % (t, d, ms, ln, typ, b[5], st, mo, rd, tag))
    elif b[1] & 0xF8 == 0xC0 and len(b) >= 30:
        # 我方应答
        use_flag = b[19]
        pressure = b[25] | (b[26] << 8)
        print("  [%s] %s t=%6d len=%3d  ← 应答 use=0x%02X pressure=0x%04X 料位=%02X"
              % (t, d, ms, ln, use_flag, pressure, b[20]))
