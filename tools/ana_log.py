#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ana_log.py —— 把 mon485.bat 抓下来的日志压成"人看的一页"

用法：
    ana_log.bat                                 # 双击即可，自动取最新日志
    python tools\\ana_log.py                     # 同上（命令行版）
    python tools\\ana_log.py tools\\logs\\mon485_20261025_172223.log
    python tools\\ana_log.py 最新.log 15:06:09 15:06:12   # 只看这个时间窗

输出四段：
    1) 概况        时间跨度 / RX TX 帧数 / 丢帧数
    2) 状态机时间线 只列打印机发来的 Dxx 帧（送料 / 准备使用 / 准备退料 / 使用中 / 暂停），
                    连续相同的合并成一行，看"打印机什么时候改主意"最直观
    3) ★电机事件   电机方向 / 离合 / 动作 / 打印阶段 的每一次变化
    4) 异常        丢帧告警、错误类日志

为什么需要：原始日志一秒上百行，肉眼根本追不上状态机跳变。
"""

import re
import sys
import os
import glob

LINE = re.compile(r'^\[(\d\d:\d\d:\d\d\.\d\d\d)\]\s+(.*)$')
RX   = re.compile(r'^(RX|TX)\s+#(\d+)\s+t=(\d+)\s+len=(\d+)\s+(\[3D\])?\s*([0-9A-Fa-f]*)\s*(.*)$')
# 解码尾巴形如：<<准备使用 ams0 read=0xFF
ST   = re.compile(r'<<(.+?)\s+ams(\d+)\s+read=0x([0-9A-Fa-f]{2})')


def pick_log(argv):
    if len(argv) >= 1 and os.path.isfile(argv[0]):
        return argv[0], argv[1:]
    here = os.path.dirname(os.path.abspath(__file__))
    cand = sorted(glob.glob(os.path.join(here, 'logs', '*.log')))
    if not cand:
        sys.exit('找不到日志：%s\\logs\\*.log 是空的，先跑一次 mon485.bat' % here)
    return cand[-1], argv


def main():
    path, rest = pick_log(sys.argv[1:])
    t_from = rest[0] if len(rest) > 0 else None
    t_to   = rest[1] if len(rest) > 1 else None

    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        raw = f.read().splitlines()

    first_t = last_t = None
    n_rx = n_tx = 0
    lost = 0
    timeline = []      # (time, seq, t, dec)  只收有解码的 RX
    motors = []        # (time, text)
    bad = []           # (time, text)
    last_by_ams = {}   # ams -> (状态文本, read)  按通道去重
    n_state = 0

    def in_win(t):
        if t_from and t < t_from:
            return False
        if t_to and t > t_to:
            return False
        return True

    for ln in raw:
        m = LINE.match(ln)
        if not m:
            continue
        t, body = m.group(1), m.group(2)

        if body.startswith('⚠ 丢帧'):
            mm = re.search(r'丢帧\s+(\d+)\s+条', body)
            if mm:
                lost += int(mm.group(1))
            if in_win(t):
                bad.append((t, body))
            continue

        if body.startswith('★电机'):
            if in_win(t):
                motors.append((t, body[3:].strip()))
            continue

        r = RX.match(body)
        if r:
            if first_t is None:
                first_t = t
            last_t = t
            if r.group(1) == 'RX':
                n_rx += 1
            else:
                n_tx += 1
            dec = (r.group(7) or '').strip()
            if r.group(1) == 'RX' and dec:
                sm = ST.search(dec)
                if not sm:
                    continue
                state, ams, rd = sm.group(1), int(sm.group(2)), sm.group(3)
                # 打印机 4 路轮询，同一状态每 40ms 来一遍；按通道各记一份上次状态，
                # 只有"这个通道真的换了状态"才进时间线，否则日志会被刷爆。
                if last_by_ams.get(ams) == (state, rd):
                    continue
                last_by_ams[ams] = (state, rd)
                n_state += 1
                timeline.append((t, r.group(2), r.group(3), state, ams, rd))
            continue

        if body.startswith('状态:') and in_win(t):
            # 状态行只在内容变化时才会被打印，这里只收"堵转 LATCH"这类关键项
            if '堵转LATCH' in body:
                bad.append((t, body))
            continue

    print('=' * 78)
    print('日志：%s' % path)
    if first_t:
        print('时间：%s ~ %s' % (first_t, last_t))
    print('帧数：RX %d  TX %d  丢帧 %d 条  状态变化 %d 次' % (n_rx, n_tx, lost, n_state))
    if t_from or t_to:
        print('筛选：%s ~ %s' % (t_from or '开头', t_to or '结尾'))
    print('=' * 78)

    print('\n---- 2) 打印机状态机时间线（RX Dxx，按通道去重）----')
    if not timeline:
        print('  （没有带解码的 Dxx 帧；可能全程都是 Cxx 读帧，或设备空闲）')
    for t, seq, tt, state, ams, rd in timeline:
        print('  %s  #%-7s t=%-8s ams%d  %-12s read=0x%s' % (t, seq, tt, ams, state, rd))

    print('\n---- 3) ★电机动作事件 ----')
    if not motors:
        print('  （全程电机没动过）')
    for t, txt in motors:
        print('  %s  %s' % (t, txt))

    print('\n---- 4) 异常 ----')
    if not bad:
        print('  （无丢帧、无堵转）')
    for t, txt in bad[:80]:
        print('  %s  %s' % (t, txt))
    if len(bad) > 80:
        print('  …… 还有 %d 条' % (len(bad) - 80))

    print()


if __name__ == '__main__':
    main()
