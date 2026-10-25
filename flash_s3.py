#!/usr/bin/env python3
"""
s3-ams 一键刷写脚本（固化正确偏移，避免再刷错分区）。

用法：
    # 默认用本地 .pio/build/s3ams 的产物
    python flash_s3.py COM6

    # 或指定固件目录（如 GitHub 下载下来的 zip 解压后的产物）
    python flash_s3.py COM6 --bootloader bl.bin --partitions pt.bin --firmware fw.bin

偏移约定（与 partitions_16mb.csv 的 16MB 双 OTA 布局严格一致）：
    bootloader   @ 0x0
    partitions   @ 0x8000
    firmware (app, ota_0) @ 0x20000   ← 关键！不是 0x10000

刷写顺序：
    1. 读 esp-asm-s3 同款三段（bootloader / 分区表 / 应用），
       应用必须落在 ota_0=0x20000，与分区表对齐。
    2. 不擦除 NVS（0x9000）——保留 WiFi 配置。
    3. 硬复位后抓 45s 串口日志，校验 "Loaded app from partition at offset 0x20000"。

依赖：pip install esptool pyserial
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_PORT = "COM6"
DEFAULT_ENV_BUILD = Path(__file__).parent / ".pio" / "build" / "s3ams"


def default_artifacts():
    bl = DEFAULT_ENV_BUILD / "bootloader.bin"
    pt = DEFAULT_ENV_BUILD / "partitions.bin"
    fw = DEFAULT_ENV_BUILD / "firmware.bin"
    if not (bl.exists() and pt.exists() and fw.exists()):
        sys.exit(f"本地产物缺失：\n  {bl}\n  {pt}\n  {fw}\n请先 pio run -e s3ams 或下载 GitHub 产物用 --bootloader/--partitions/--firmware 指定")
    return bl, pt, fw


def find_esptool():
    candidates = [
        r"C:\Users\lkfcs\AppData\Local\Programs\Python\Python310\Scripts\esptool.exe",
        r"C:\Users\lkfcs\AppData\Local\Programs\Python\Python310\Scripts\esptool",
        "esptool",
    ]
    for c in candidates:
        if os.path.exists(c) or c == "esptool":
            return c
    sys.exit("找不到 esptool.exe，请用 pip install esptool 或修改脚本")


def _esptool_cmd(port, *args):
    """拼 esptool 命令行（兼容 exe 与 python -m esptool 两种安装形态）"""
    esptool = find_esptool()
    if esptool != "esptool":
        return [esptool, "--port", port, "--chip", "esp32s3", "--baud", "460800", *args]
    return [sys.executable, "-m", "esptool", "--port", port, "--chip", "esp32s3",
            "--baud", "460800", *args]


def erase_otadata(port):
    """★ 2026-10-25 血的教训：必须擦 otadata（0xF000 / 0x2000）

    网页 OTA 成功后，otadata 会被改成指向 ota_1。此后串口刷机只写
    ota_0@0x20000 —— bootloader 按 otadata 仍然从 ota_1 启动，
    也就是**新固件刷进去了但根本没跑**。症状：
      - 串口日志里没有 "Loaded app from partition at offset 0x20000"
      - 明明改了代码、编译刷机都成功，行为却一点没变
      - /status 的 app_version 还是上一版
    擦掉 otadata（全 0xFF）后，bootloader 回退到第一个有效 app = ota_0。
    """
    print("=== 擦除 otadata（0xF000/0x2000）：强制下次从 ota_0 启动 ===")
    cmd = _esptool_cmd(port, "erase_region", "0xF000", "0x2000")
    r = subprocess.run(cmd, capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0 or "not found" in out.lower():
        print("⚠ otadata 擦除失败（设备可能仍从 ota_1 启动）：")
        print(out[-400:])
        return False
    print("  otadata 已擦除")
    return True


def write_flash(port, bl, pt, fw):
    cmd = _esptool_cmd(port, "write_flash",
                       "0x0", str(bl),
                       "0x8000", str(pt),
                       "0x20000", str(fw))
    print("=== 三段刷写：boot@0x0 + part@0x8000 + app@0x20000（保留 NVS）===")
    print("  bootloader : ", bl, f"({bl.stat().st_size}B)")
    print("  partitions : ", pt, f"({pt.stat().st_size}B)")
    print("  firmware   : ", fw, f"({fw.stat().st_size}B)")
    print("CMD:", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout)
    if r.returncode != 0:
        print("STDERR:", r.stderr)
        sys.exit("esptool 刷写失败")
    if "Hash of data verified" not in r.stdout:
        print("⚠ 未看到 'Hash of data verified'，请人工确认")


def watch_boot(port, dur=45):
    try:
        import serial
    except ImportError:
        print("（未装 pyserial，跳过 boot 日志校验：pip install pyserial）")
        return
    print(f"=== 抓 boot 日志（{dur}s）===")
    ser = serial.Serial(port, 115200, timeout=0.1, exclusive=True)
    time.sleep(0.2)
    ser.dtr = True; ser.rts = True
    time.sleep(0.05); ser.dtr = False
    time.sleep(0.05); ser.rts = False
    ser.reset_input_buffer()
    s = ""
    end = time.time() + dur
    while time.time() < end:
        c = ser.read(4096)
        if c:
            s += c.decode("utf-8", "replace")
    ser.close()

    print("SHA-256 失败？", "SHA-256 comparison failed" in s)
    print("进 app？", "Loaded app from partition at offset 0x20000" in s)
    print("分区表 ota_0？", "00020000" in s)
    keep = [l for l in s.split("\n") if l.strip() and any(k in l for k in [
        "SHA", "Loaded app", "partition", "Invalid", "No bootable",
        "core_dump", "task_wdt", "ESP-AMS", "ams", "AP", "STA", "boot:"
    ])]
    print("\n=== 关键日志 ===")
    for l in keep[:30]:
        print("  ", l)
    if "Loaded app from partition at offset 0x20000" in s and "SHA-256 comparison failed" not in s:
        print("\n✅ 刷机成功：固件已落到 ota_0=0x20000，设备正常 boot")
    else:
        print("\n❌ 未确认进 app，请检查上方日志")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT, help="串口，如 COM6 / /dev/ttyUSB0")
    ap.add_argument("--bootloader", help="bootloader.bin 路径")
    ap.add_argument("--partitions", help="partitions.bin 路径")
    ap.add_argument("--firmware", help="firmware.bin 路径")
    ap.add_argument("--no-watch", action="store_true", help="不抓 boot 日志")
    args = ap.parse_args()

    if args.bootloader and args.partitions and args.firmware:
        bl, pt, fw = Path(args.bootloader), Path(args.partitions), Path(args.firmware)
    else:
        bl, pt, fw = default_artifacts()

    write_flash(args.port, bl, pt, fw)
    erase_otadata(args.port)   # 必须在写完之后擦：否则 OTA 残留会把启动拽回 ota_1
    if not args.no_watch:
        time.sleep(1)
        watch_boot(args.port)


if __name__ == "__main__":
    main()
