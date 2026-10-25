import re

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[[^\]]*\]\s+([0-9A-Fa-f]+)")

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    frames.append((int(m.group(3)), m.group(2), bytes.fromhex(m.group(5))))

print("=== AMS0 的 13 字节 0xC5/0x04 帧时间线（t=100000..130000，压缩连续重复）===")
prev = None
run = 0
lastt = 0
for ms, d, b in frames:
    if not (100000 <= ms <= 130000):
        continue
    if not (len(b) == 13 and b[1] == 0xC5 and b[4] == 0x04 and b[5] == 0x00):
        continue
    key = (b[6], b[7], b[8], b[9])
    if key == prev:
        run += 1
        lastt = ms
        continue
    if prev is not None:
        print("  t=%d..%d  statu=%02X motion=%02X b8=%02X read=%02X  x%d" % (startt, lastt, prev[0], prev[1], prev[2], prev[3], run))
    prev = key
    run = 1
    startt = ms
    lastt = ms
if prev is not None:
    print("  t=%d..%d  statu=%02X motion=%02X b8=%02X read=%02X  x%d" % (startt, lastt, prev[0], prev[1], prev[2], prev[3], run))
