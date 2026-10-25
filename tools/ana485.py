import re, collections

path = "tools/新建 文本文档.txt"
pat = re.compile(r"\[(\d\d:\d\d:\d\d)\]\s+(RX|TX)\s+t=(\d+)\s+len=(\d+)\s+\[[^\]]*\]\s+([0-9A-Fa-f]+)")

frames = []
for line in open(path, encoding="utf-8", errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    t, d, ms, ln, hexs = m.group(1), m.group(2), int(m.group(3)), int(m.group(4)), m.group(5)
    frames.append((t, ms, d, ln, bytes.fromhex(hexs)))

print("total frames:", len(frames))
rx = [f for f in frames if f[2] == "RX"]
print("RX:", len(rx), "TX:", len(frames) - len(rx))

cnt = collections.Counter()
for t, ms, d, ln, b in rx:
    if len(b) >= 11 and b[1] == 0xC5:
        cnt[(b[5], b[6], b[7], b[9])] += 1

print("\n=== RX 0xC5 (ams, statu, motion, read) 计数 ===")
for k, v in cnt.most_common():
    print("  ams=%02X statu=%02X motion=%02X read=%02X  x%d" % (k[0], k[1], k[2], k[3], v))

print("\n=== statu_flags==0x03 的帧 ===")
n = 0
for t, ms, d, ln, b in frames:
    if len(b) >= 11 and b[1] == 0xC5 and b[6] == 0x03:
        print("  [%s] %s t=%d ams=%02X statu=%02X motion=%02X read=%02X" % (t, d, ms, b[5], b[6], b[7], b[9]))
        n += 1
        if n > 40:
            print("  ...(截断)")
            break
print("  累计 statu=0x03 帧数:", sum(1 for t, ms, d, ln, b in frames if len(b) >= 11 and b[1] == 0xC5 and b[6] == 0x03))

print("\n=== 各 statu_flags 首次出现 ===")
first = {}
for t, ms, d, ln, b in rx:
    if len(b) >= 11 and b[1] == 0xC5:
        if b[6] not in first:
            first[b[6]] = (t, ms, b[5], b[7], b[9])
for k in sorted(first):
    v = first[k]
    print("  statu=%02X first [%s] t=%d ams=%02X motion=%02X read=%02X" % (k, v[0], v[1], v[2], v[3], v[4]))

print("\n=== 状态行（电机/离合/电流）统计 ===")
sp = re.compile(r"状态:\s*电机=(\S+)\s+离合=(\d)\s+电流=(\d+)mA\s+峰值=(\d+)mA\s+act485=(\d)")
st = collections.Counter()
for line in open(path, encoding="utf-8", errors="replace"):
    m = sp.search(line)
    if m:
        st[(m.group(1), m.group(2), m.group(5))] += 1
for k, v in st.most_common():
    print("  电机=%s 离合=%s act485=%s  x%d" % (k[0], k[1], k[2], v))
