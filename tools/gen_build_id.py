"""
gen_build_id.py —— PlatformIO 预脚本：每次编译生成一个"链接时刻"戳

为什么需要它
------------
网页 OTA 判断"升级到底成功没有"，原来的判据是 **app_version 有没有变**。
但 `__DATE__ / __TIME__` 是**编译**时刻 —— 只改了别的文件时，
web_server.cpp 不会被重新编译，build_time 就还是旧的。

⇒ 于是「同版本号重新刷一次固件」（改了代码但忘了改版本号）
   页面上永远判不出成功，一直停在"正在等待设备重启并核对版本…"，
   看着像"没启动成功"，其实固件早就写进去、早就跑起来了。

而 ESP-IDF app descriptor 里的 `time`/`date` 也不行：
本工程用 Arduino 预编译的 IDF 库，那两个字段是**库包**的编译时间
（实测恒为 "Feb 12 2025"），不随我们的链接变化。

所以最可靠的办法：**每次编译生成一个新的 build id 头文件**。
头文件内容一变，PlatformIO 的依赖追踪就会重新编译 web_server.cpp，
新 id 被烧进固件 —— 刷进去就一定和上一版不同。

用法（platformio.ini）：
    extra_scripts = pre:tools/gen_build_id.py
"""

import datetime
import os

Import("env")  # noqa: F821  (PlatformIO 注入)

PROJECT_DIR = env["PROJECT_DIR"]  # noqa: F821
SRC_DIR = os.path.join(PROJECT_DIR, "src")
OUT = os.path.join(SRC_DIR, "build_id.h")

now = datetime.datetime.now()
# 格式：20261101-210530（秒级精度，同一秒内重复编译才会相同）
build_id = now.strftime("%Y%m%d-%H%M%S")

content = (
    "/* 由 tools/gen_build_id.py 自动生成，**不要手改**。\n"
    " * 每次编译都会刷新，用来给网页 OTA 判断\"这次刷的到底是不是新固件\"。\n"
    " * 生成时刻：%s\n"
    " */\n"
    "#ifndef S3AMS_BUILD_ID_H\n"
    "#define S3AMS_BUILD_ID_H\n"
    "#define S3AMS_BUILD_ID \"%s\"\n"
    "#define S3AMS_BUILD_DATE \"%s\"\n"
    "#define S3AMS_BUILD_TIME \"%s\"\n"
    "#endif\n"
) % (now.strftime("%Y-%m-%d %H:%M:%S"), build_id,
     now.strftime("%Y-%m-%d"), now.strftime("%H:%M:%S"))

# 内容没变就别改文件，避免无谓的重编译
old = None
if os.path.exists(OUT):
    try:
        with open(OUT, "r", encoding="utf-8") as f:
            old = f.read()
    except OSError:
        old = None

if old != content:
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(content)
    print("gen_build_id: %s -> %s" % (OUT, build_id))
else:
    print("gen_build_id: 沿用 %s" % build_id)
