#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线校验 /status 的 JSON 拼串有没有语法错（v1.1.32 加）。

## 为什么需要这个工具

`handle_status()` 是用几十条 `json += "..." + String(x) + "..."` 手拼出来的，
括号/逗号全靠人眼保证。v1.1.31 就在 ams 里少写了一个前导逗号、多写了一个
尾逗号：

    ..."link":{...}"nvs":{...},}      ← 非法

后果特别阴险：**设备完全正常**（WiFi、485、打印都好），但前端
`JSON.parse` 抛异常 → 网页所有状态位显示 "-"、OTA 页永远判不出
"设备已重启"，现场看起来极像"固件刷死了"。

## 它怎么做的

不编译、不跑固件，直接把 `src/web_server.cpp` 里 `handle_status()` 的
字符串字面量按出现顺序拼起来，把 `String(...)` / `snprintf(...)` /
`log_*()` 替换成 `0`，再交给 `json.loads()` 校验。

两个已知的"提取器噪声"，工具会自动归一化，不算错：
  * 循环里的条件逗号（`if (i) json += ",";`）摊平后会出现 `[,` / `,]`
  * `switch` 分支里的状态名（`dtxt = "waiting";`）不是 JSON，按变量名
    以 `txt` 结尾整句剔除

## 用法

    python tools/check_status_json.py            # 校验，OK 时返回 0
    python tools/check_status_json.py -v         # 出错时打印上下文

建议**每次改 handle_status() 之后跑一次**，比刷机验证快得多。
"""

import io
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src", "web_server.cpp")
END_MARK = 'server.send(200, "application/json; charset=utf-8", json);'

# 这些调用整体丢弃（里面的字符串不是 JSON 片段）
SKIP_CALLS = ("snprintf", "log_i", "log_w", "log_e", "log_d", "log_v")


def extract(body):
    """把 handle_status() 的 json 拼串还原成一条 JSON 文本（值统一填 0）。"""
    out = []
    i, n = 0, len(body)

    def match_call(pos):
        k = pos
        while k < n and body[k] in " \t":
            k += 1
        if k >= n or body[k] != "(":
            return -1
        depth, j = 0, k
        while j < n:
            c = body[j]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    return j
            elif c == '"':
                j += 1
                while j < n and body[j] != '"':
                    if body[j] == "\\":
                        j += 1
                    j += 1
            j += 1
        return -1

    while i < n:
        c = body[i]
        if c == "/" and body.startswith("//", i):
            j = body.find("\n", i)
            i = n if j < 0 else j
            continue
        if c == "/" and body.startswith("/*", i):
            j = body.find("*/", i + 2)
            i = n if j < 0 else j + 2
            continue
        if c == '"':
            j, buf = i + 1, []
            while j < n:
                ch = body[j]
                if ch == "\\":
                    nx = body[j + 1]
                    buf.append({"n": "\n", "t": "\t", '"': '"',
                                "\\": "\\", "r": "\r"}.get(nx, "\\" + nx))
                    j += 2
                    continue
                if ch == '"':
                    break
                buf.append(ch)
                j += 1
            out.append("".join(buf))
            i = j + 1
            continue

        handled = False
        for name in ("String",) + SKIP_CALLS:
            if body.startswith(name, i) and (i == 0 or not
                                             (body[i - 1].isalnum() or body[i - 1] == "_")):
                j = match_call(i + len(name))
                if j > 0:
                    if name == "String":
                        out.append("0")
                    i, handled = j + 1, True
                break
        if handled:
            continue
        i += 1
    return "".join(out)


def structural_check(s):
    """与固件里 json_struct_ok() 等价的检查（对 if/else 分支不敏感）。

    返回 (ok, 说明)。
    检查三件事：
      ① 花括号/方括号配对，引号是否闭合
      ② 逗号后面紧跟 } 或 ]（尾逗号，JSON 不允许）
      ③ 右括号不会多于左括号
    """
    depth = 0
    in_str = False
    esc = False
    last_sig = ""
    for c in s:
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
                last_sig = c
            continue
        if c == '"':
            in_str = True
            last_sig = c
            continue
        if c in " \t\r\n":
            continue
        if c in "}]":
            if last_sig == ",":
                return False, "出现尾逗号 `,%s`（JSON 不允许）" % c
            depth -= 1
            if depth < 0:
                return False, "多出来一个 `%s`" % c
        elif c in "{[":
            depth += 1
        last_sig = c
    if in_str:
        return False, "字符串引号没有闭合"
    if depth != 0:
        return False, "括号没配平（还差 %d 个右括号）" % depth
    return True, ""


def main():
    verbose = "-v" in sys.argv

    src = io.open(SRC, encoding="utf-8").read()
    start = src.index("static void handle_status(void)")
    end = src.index(END_MARK, start)
    body = src[start:end]

    # switch 分支里的状态名（dtxt / phase / const char *phase ...）不是 JSON。
    # 规则：`标识符 = "字面量"` 整句剔除，但 `json = "..."` 要保留（那是 JSON 开头）。
    def _drop_assign(m):
        return m.group(0) if m.group(1) == "json" else ""

    body = re.sub(r'(?<![+\w])([A-Za-z_]\w*)\s*=\s*"[^"]*"', _drop_assign, body)

    txt = extract(body)
    # 循环里的条件逗号摊平后留下的 [,  /  ,]
    norm = txt.replace("[,", "[").replace(",]", "]")

    ok, why = structural_check(norm)
    if not ok:
        print("[FAIL] /status 的 JSON 结构损坏：%s" % why)
        print("       设备本身不会崩，但网页会显示全 \"-\"、OTA 页判不出重启。")
        if verbose:
            print("       还原后的文本（前 600 字符）:")
            print("       " + norm[:600])
        return 1

    # 结构没问题，再试完整解析。失败多半是 if/else 分支被摊平造成的
    # 假阳性（同一路径只会走一支），所以只当警告，不当失败。
    try:
        json.loads(norm)
    except Exception as e:                                   # noqa: BLE001
        print("[WARN] 结构 OK，但完整解析失败：%s" % e)
        print("       多半是 if/else 分支被摊平导致的假阳性；")
        print("       请人工确认下面这段是不是同一分支的两条互斥路径：")
        if verbose:
            pos = getattr(e, "pos", 0)
            print("       ...%s..." % norm[max(0, pos - 160):pos + 160])
        return 0

    print("[ OK ] /status 的 JSON 合法（%d 字符，完整解析通过）" % len(norm))
    return 0


if __name__ == "__main__":
    sys.exit(main())
