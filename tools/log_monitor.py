#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
log_monitor.py —— s3-ams 设备日志/状态监控（PC 侧，挂后台跑）

用法：
    python tools/log_monitor.py                     # 默认 192.168.2.100
    python tools/log_monitor.py --ip 192.168.2.100 --interval 1
    python tools/log_monitor.py --status-only       # 只看状态不看日志
    python tools/log_monitor.py --log-only          # 只看日志不出状态行

做两件事：
  1. 轮询 /log，把设备端日志环形缓冲里**新追加**的条目带本地时间戳打出来
     （重叠检测：旧串的最长后缀如果是新串的前缀，剩下的就是新内容；
      整段对不上——比如设备重启——就整串当新内容并标记分隔线）
  2. 轮询 /status，出一行紧凑状态：电机方向 / 占空比 / 电流 / 峰值 /
     堵转 latch / 跳闸次数 / 吸合离合 / act485 开关。关键字段变化时
     额外打一行醒目提示（进料测试就看这个）。

输出同时写控制台和 logs/monitor-YYYYmmdd-HHMMSS.log，Ctrl+C 停止。
断网自动退避重连，不会退出。
"""

import argparse
import json
import sys
import time
import urllib.request
import urllib.error
from datetime import datetime
from pathlib import Path

# 需要盯的关键字（命中就加 [!] 标记）
ALERT_KEYWORDS = ("堵转", "跳闸", "失败", "出错", "未通过", "错误", "★",
                  "超时", "冲突", "stall")


def now_str():
    return datetime.now().strftime("%H:%M:%S")


def fetch(base, path, timeout=3.0):
    """GET 一个 JSON 接口；失败返回 None（不抛异常，调用方决定退避）"""
    url = base.rstrip("/") + path
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8", "replace"))
    except (urllib.error.URLError, OSError, ValueError):
        return None


def new_tail(old, new):
    """从 new 里抠出相对 old 新追加的部分。old=None 时整串都是新的。
    （旧固件 /log 返回单串的退化路径；新固件返回行数组+seq，走下面那条）"""
    if old is None:
        return new, False
    if new == old:
        return "", False
    # 旧串最长后缀 == 新串前缀 → 后缀之后即新内容
    max_k = min(len(old), len(new))
    for k in range(max_k, 0, -1):
        if new.startswith(old[len(old) - k:]):
            return new[k:], False
    # 完全对不上：设备重启或缓冲被冲掉
    return new, True


def log_entries(d, prev_text, prev_seq):
    """把 /log 的返回值规整成 [(seq, line), ...] 增量。
    新固件：{"log":[...], "seq":[...]} → 按 seq 去重
    旧固件：{"log":"单串"} → 最长后缀重叠检测
    返回 (entries, prev_text, prev_seq)"""
    raw = d.get("log", "")
    if isinstance(raw, list):
        seqs = d.get("seq") or []
        out = []
        for i, line in enumerate(raw):
            s = seqs[i] if i < len(seqs) else -1
            if s >= 0 and s <= prev_seq:
                continue
            out.append((s, line))
        new_prev_seq = seqs[-1] if seqs else prev_seq
        if out and seqs and seqs[-1] < prev_seq:
            # 行号倒退 = 设备重启
            out = [(s, l) for s, l in zip(seqs, raw)]
            new_prev_seq = seqs[-1] if seqs else -1
            return [(-1, "---- 设备重启，日志缓冲重置 ----")] + out, prev_text, new_prev_seq
        return out, prev_text, new_prev_seq
    # 旧固件：单串
    tail, reset = new_tail(prev_text, raw)
    entries = []
    if reset and tail:
        entries.append((-1, "---- 设备日志缓冲不连续（设备可能重启） ----"))
    if tail and tail.strip():
        entries.append((-1, tail.strip()))
    return entries, raw, prev_seq


def fmt_status(d):
    """从 /status 抠出测试进料需要的字段，返回 (一行文本, 变化字典)"""
    hw = d.get("hardware", {}) or {}
    cur = d.get("current", {}) or {}
    ams = d.get("ams", {}) or {}

    md = hw.get("motor_direction", 0)
    dir_txt = {1: "进料", -1: "退料", 0: "停止"}.get(md, str(md))
    engaged = hw.get("engaged", []) or []
    ch_txt = (",".join(str(c) for c in engaged) if engaged else "-")

    ma = cur.get("ma", -1)
    mv = cur.get("mv", -1)
    peak_ma = cur.get("peak_ma", -1)
    tripped = bool(cur.get("tripped"))
    trips = cur.get("trips", 0)
    ready = cur.get("ready")
    stall_ma = cur.get("stall_ma", 0)

    ma_txt = f"{ma}mA" if isinstance(ma, (int, float)) and ma >= 0 else "--"
    peak_txt = f"{peak_ma}mA" if isinstance(peak_ma, (int, float)) and peak_ma >= 0 else "--"
    trip_txt = f"跳闸x{trips}" + ("[已LATCH]" if tripped else "")

    line = (f"电机:{dir_txt:<2} 离合:{ch_txt:<3} 电流:{ma_txt:<7} "
            f"峰值1s:{peak_txt:<7} 阈值:{stall_ma}mA {trip_txt} "
            f"act485:{d.get('act485', 0)} busy:{1 if ams.get('busy') else 0}")
    if ready is False:
        line += " [采样不可用]"
    if tripped:
        line = "[!] " + line
    return line, {"dir": md, "ma": ma, "tripped": tripped, "trips": trips,
                  "engaged": tuple(engaged)}


def main():
    ap = argparse.ArgumentParser(description="s3-ams 设备日志/状态监控")
    ap.add_argument("--ip", default="192.168.2.100", help="设备 IP")
    ap.add_argument("--interval", type=float, default=1.0, help="轮询间隔秒")
    ap.add_argument("--status-only", action="store_true", help="不出设备日志")
    ap.add_argument("--log-only", action="store_true", help="不出状态行")
    ap.add_argument("--out", default=None, help="日志文件（默认 logs/monitor-时间戳.log）")
    args = ap.parse_args()

    base = f"http://{args.ip}"

    out_dir = Path(__file__).resolve().parent.parent / "logs"
    out_dir.mkdir(exist_ok=True)
    if args.out:
        out_path = Path(args.out)
    else:
        out_path = out_dir / f"monitor-{datetime.now().strftime('%Y%m%d-%H%M%S')}.log"

    f = out_path.open("a", encoding="utf-8")

    def emit(text):
        print(text, flush=True)
        f.write(text + "\n")
        f.flush()

    emit(f"# {now_str()} 监控启动 → {base}（间隔 {args.interval}s）")
    emit(f"# 日志文件: {out_path}")
    emit(f"# Ctrl+C 停止；断网自动重连")

    prev_log = None      # 上一次 /log 的整串（旧固件路径）
    prev_seq = -1        # 上一次见到的最大行号（新固件路径）
    prev_key = None      # 上一次状态关键字段
    fails = 0
    first_status = True

    try:
        while True:
            t0 = time.time()

            # ---- /log：新追加的条目 ----
            if not args.status_only:
                d = fetch(base, "/log")
                if d is None:
                    fails += 1
                    if fails in (1, 5, 20) or fails % 60 == 0:
                        emit(f"{now_str()} [!] /log 请求失败（连续 {fails} 次），重试中…")
                else:
                    if fails:
                        emit(f"{now_str()} 连接恢复（此前连续失败 {fails} 次）")
                        fails = 0
                    entries, prev_log, prev_seq = log_entries(
                        d, prev_log, prev_seq)
                    for _seq, line in entries:
                        if not line:
                            continue
                        mark = "[!]" if any(k in line for k in ALERT_KEYWORDS) else "   "
                        emit(f"{now_str()} {mark} {line}")

            # ---- /status：紧凑状态行 ----
            if not args.log_only:
                d = fetch(base, "/status")
                if d is None:
                    fails += 1
                    if fails in (1, 5, 20) or fails % 60 == 0:
                        emit(f"{now_str()} [!] /status 请求失败（连续 {fails} 次），重试中…")
                else:
                    if fails:
                        emit(f"{now_str()} 连接恢复（此前连续失败 {fails} 次）")
                        fails = 0
                    line, key = fmt_status(d)
                    emit(f"{now_str()} {line}")
                    if first_status:
                        first_status = False
                        emit(f"{now_str()} --- 状态基线已建立，之后只在关键字段变化时提示 ---")
                    elif prev_key is not None:
                        # 电机起停
                        if key["dir"] != prev_key["dir"]:
                            act = {1: "▶ 电机启动（进料）", -1: "◀ 电机启动（退料）",
                                   0: "■ 电机停止"}.get(key["dir"], "? 电机状态变化")
                            emit(f"{now_str()} === {act} ===")
                        # 堵转跳闸 / latch
                        if key["tripped"] and not prev_key["tripped"]:
                            emit(f"{now_str()} [!] === 堵转保护跳闸！电机已停、离合已断 ===")
                        if key["trips"] != prev_key["trips"]:
                            emit(f"{now_str()} [!] 跳闸次数 {prev_key['trips']} → {key['trips']}")
                        # 离合切换
                        if key["engaged"] != prev_key["engaged"]:
                            emit(f"{now_str()} --- 离合 {prev_key['engaged']} → {key['engaged']} ---")
                        # 电流突变（±300mA 才报，避免每行都刷）
                        if (isinstance(key["ma"], (int, float)) and isinstance(prev_key["ma"], (int, float))
                                and abs(key["ma"] - prev_key["ma"]) >= 300):
                            emit(f"{now_str()} ~~~ 电流 {prev_key['ma']}mA → {key['ma']}mA ~~~")
                    prev_key = key

            # ---- 睡到下一个周期（扣掉耗时）----
            dt = time.time() - t0
            if dt < args.interval:
                time.sleep(args.interval - dt)
            elif fails:
                time.sleep(min(2.0, 0.2 * min(fails, 10)))  # 失败时退避
    except KeyboardInterrupt:
        emit(f"{now_str()} 监控停止（Ctrl+C）")
    finally:
        f.close()


if __name__ == "__main__":
    main()
