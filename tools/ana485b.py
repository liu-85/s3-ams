import re, collections

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[[^\]]*\]\s+([0-9A-Fa-f]+)")

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    frames.append((m.group(1), int(m.group(3)), m.group(2), bytes.fromhex(m.group(5))))

rx = [f for f in frames if f[2] == "RX"]

print("=== RX 0xC5 按 buf[4] 分类 ===")
c = collections.Counter()
for t, ms, d, b in rx:
    if len(b) > 5 and b[1] == 0xC5:
        c[(b[4], len(b))] += 1
for k, v in sorted(c.items()):
    print("  buf[4]=%02X len=%d  x%d" % (k[0], k[1], v))

print("\n=== buf[4]==0x03 的样例（最多 15 条）===")
n = 0
for t, ms, d, b in frames:
    if len(b) > 5 and b[1] == 0xC5 and b[4] == 0x03:
        print("  [%s] %s t=%d len=%d %s" % (t, d, ms, len(b), b.hex().upper()))
        n += 1
        if n >= 15:
            break
print("  buf[4]=0x03 总数:", sum(1 for t, ms, d, b in frames if len(b) > 5 and b[1] == 0xC5 and b[4] == 0x03))

print("\n=== buf[4]==0x04 且 len==13 的样例（最多 15 条）===")
n = 0
for t, ms, d, b in frames:
    if len(b) == 13 and b[1] == 0xC5 and b[4] == 0x04:
        print("  [%s] %s t=%d %s" % (t, d, ms, b.hex().upper()))
        n += 1
        if n >= 15:
            break

print("\n=== 时间窗 t=109000..112000 全部 RX/TX（看交错顺序）===")
for t, ms, d, b in frames:
    if 109000 <= ms <= 112000:
        print("  t=%d %s %s" % (ms, d, b.hex().upper()))
