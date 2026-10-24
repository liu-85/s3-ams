#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 web/ 目录下的静态文件（index.html, app.js, style.css）转换为 C 头文件。
生成的 web_files.h 包含每个文件的字符串常量，供 web_server.cpp 直接 serve。
"""

import os
import sys

WEB_DIR = os.path.join(os.path.dirname(__file__), "..", "web")
OUT_FILE = os.path.join(os.path.dirname(__file__), "..", "src", "web_files.h")

FILES = [
    ("index.html", "WEB_INDEX_HTML"),
    ("style.css", "WEB_STYLE_CSS"),
    ("app.js", "WEB_APP_JS"),
]


def esc_cpp_string(text):
    """转义 C 字符串中的特殊字符。"""
    out = []
    for ch in text:
        if ch == '\\':
            out.append('\\\\')
        elif ch == '"':
            out.append('\\"')
        elif ch == '\n':
            out.append('\\n"\n"')
        elif ch == '\r':
            pass  # 丢弃 \r
        elif ch == '\t':
            out.append('\\t')
        else:
            out.append(ch)
    return ''.join(out)


def main():
    lines = []
    lines.append("/* 本文件由 tools/generate_web_files.py 自动生成，请勿手动修改 */")
    lines.append("#pragma once")
    lines.append("#include <Arduino.h>")
    lines.append("")

    for fname, var in FILES:
        fpath = os.path.join(WEB_DIR, fname)
        if not os.path.exists(fpath):
            print(f"跳过缺失文件: {fpath}")
            continue
        with open(fpath, "r", encoding="utf-8") as f:
            content = f.read()
        lines.append(f"// 来源: web/{fname}  ({len(content)} 字节)")
        lines.append(f'static const char *{var} =')
        lines.append('  "' + esc_cpp_string(content) + '";')
        lines.append("")

    with open(OUT_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"已生成: {OUT_FILE}")


if __name__ == "__main__":
    main()
