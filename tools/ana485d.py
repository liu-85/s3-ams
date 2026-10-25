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
print("frames=%d RX=%d TX=%d span=%d..%d" % (len(frames), len(rx), len(frames) - len(rx), frames[0][1], frames[-1][1]))

print("\n=== RX 0xC5 按 (buf[4], len) 分类 ===")
c = collections.Counter()
for t, ms, d, b in rx:
    if len(b) > 5 and b[1] == 0xC5:
        c[(b[4], len(b))] += 1
for k, v in sorted(c.items()):
    print("  buf[4]=%02X len=%2d  x%d" % (k[0], k[1], v))

print("\n=== 12 字节 0x03 短包 payload 分布 ===")
c2 = collections.Counter()
for t, ms, d, b in frames:
    if len(b) == 12 and b[1] == 0xC5 and b[4] == 0x03:
        c2[tuple(b[5:10])] += 1
for k, v in c2.most_common(20):
    print("  ams=%02X b6=%02X b7=%02X b8=%02X b9=%02X  x%d" % (k[0], k[1], k[2], k[3], k[4], v))

print("\n=== 13 字节 0x04 包 (ams,statu,motion,b8,read) 分布 ===")
c3 = collections.Counter()
for t, ms, d, b in frames:
    if len(b) == 13 and b[1] == 0xC5 and b[4] == 0x04:
        c3[(b[5], b[6], b[7], b[8], b[9])] += 1
for k, v in c3.most_common(20):
    print("  ams=%02X statu=%02X motion=%02X b8=%02X read=%02X  x%d" % (k[0], k[1], k[2], k[3], k[4], v))

print("\n=== 状态行 ===")
sp = re.compile(r"状态:\s*电机=(\S+)\s+离合=(\S+)\s+电流=(\d+)mA\s+峰值=(\d+)mA\s+act485=(\d)")
n = 0
for line in open(path, encoding="utf-8", errors="replace"):
    if sp.search(line):
        print("  " + line.strip()[:200])
        n += 1
        if n > 60:
            print("  ...")
            break
print("  状态行数:", n)

print("\n=== 13B 0x04 / ams0 状态时间线（压缩重复）===")
prev = None
run = 0
startt = lastt = 0
for t, ms, d, b in frames:
    if not (len(b) == 13 and b[1] == 0xC5 and b[4] == 0x04 and b[5] == 0x00):
        continue
    key = (b[6], b[7], b[8], b[9])
    if key == prev:
        run += 1
        lastt = ms
        continue
    if prev is not None:
        print("  t=%7d..%-7d statu=%02X motion=%02X b8=%02X read=%02X x%d" % (startt, lastt, prev[0], prev[1], prev[2], prev[3], run))
    prev = key
    run = 1
    startt = lastt = ms
if prev is not None:
    print("  t=%7d..%-7d statu=%02X motion=%02X b8=%02X read=%02X x%d" % (startt, lastt, prev[0], prev[1], prev[2], prev[3], run))
