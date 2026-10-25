# s3-ams v1.1.8 发布说明（刷机 otadata 大坑 + 日志中文截断修复）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**上一版**：v1.1.7（栈 32KB 已修好，但设备跑的根本不是它 —— 见下）

## 坑 1（最重要）：串口刷机"刷了没效果" —— otadata 没擦

**现象**：`flash_s3.py` 刷机全程 Hash 校验通过，但设备行为一点没变，
`/status` 的 `app_version` 还是上一版，日志里也看不到新加的打印。

**根因**：分区表里 `otadata` 在 **0xF000 / 8KB**，而刷机脚本只写
`boot@0x0 + partitions@0x8000 + app@0x20000`（ota_0）。
**网页 OTA 成功后，otadata 会被改成指向 ota_1**；此后 bootloader
按 otadata 从 **ota_1** 启动 —— 新固件刷进了 ota_0，但**从来没被运行**。

**铁证**：串口日志里没有 `Loaded app from partition at offset 0x20000`
（`flash_s3.py` 一直在报"分区表 ota_0？ False"，之前被误认为无关紧要）。

**修复（`flash_s3.py`）**：写完固件后追加
`esptool erase_region 0xF000 0x2000`，擦掉 otadata → bootloader 回退到
第一个有效 app = ota_0。**以后每次串口刷都会自动擦，不用再手动处理。**

> 这也解释了为什么 v1.1.5 / v1.1.6 的修复"看起来没生效"：
> 设备实际一直在跑 ota_1 里的旧镜像。

## 坑 2：日志中文被截断成半个汉字 → 日志面板空白

`/log` 返回的 JSON 解析报错：
```
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xe6 ... invalid continuation byte
```
**根因**：`web_log()` 用 `strncpy(..., 160-1)` 截断，中文日志常在 159
字节处被切成半个汉字（如 `0xE6` 后面没有 continuation 字节），
JSON 里出现非法 UTF-8 → 前端 `JSON.parse` 直接抛错。

**修复**：
- 截断时回退到完整 UTF-8 字符边界（截断点若落在 `10xxxxxx` 上，
  一路退到该字符的 lead 字节，整个字符丢弃）
- JSON 转义时把控制字符（<0x20）换成空格，避免同类问题

## 本次包含的完整修复链（v1.1.5 → v1.1.8）

| 版本 | 修复 |
|---|---|
| v1.1.5 | OTA 期间退订任务看门狗（上传几十秒喂不了狗 → TWDT panic） |
| v1.1.6 | handle_log 的 7.7KB 栈数组改 static；OTA 期间暂停日志轮询 |
| v1.1.7 | loopTask 栈 8KB→32KB（Stack canary panic）；TWDT 配置真正生效 |
| v1.1.8 | **flash_s3.py 擦 otadata**（刷机不生效的总根源）；日志中文截断 |

## 实测验证（v1.1.8 已刷入并验证）

```
主循环任务栈: 32768 字节（32 KB）
任务看门狗: 10000ms 超时，只监控主循环任务
/log 连抓 3 次：http=200 1108B 50~150ms，JSON 解析成功 17 行
/status: version=1.1.8-s3ams  stack_free=29968（32KB 栈仅用 2.8KB）
uptime 持续增长，无 panic
```

## 产物

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.8\bootloader.bin --partitions release\v1.1.8\partitions.bin --firmware release\v1.1.8\firmware.bin
```

（现在也可以直接走网页 OTA 了 —— v1.1.5~v1.1.8 已把 OTA 的坑全部填平）
