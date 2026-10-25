# s3-ams v1.1.6 发布说明（/log 栈溢出崩溃修复 —— OTA "升级失败"的真凶）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**上一版**：v1.1.5（TWDT 修复生效，但 OTA 仍报"升级失败"，/log 一抓设备就重启）

## 根因：handle_log 的 7.7KB 栈数组把 8KB 的 loopTask 栈撑爆

v1.1.2 起 `handle_log()` 在**栈上**开了 `char lines[48][160]` ≈ **7.7KB**。
Arduino 的 loopTask 栈只有 **8KB**，再叠加 WebServer
`handleClient → _handleRequest → handle_log` 调用链，直接**栈溢出 panic**。

**症状极具误导性（三个老问题一个根因）**：
1. **/log 一抓设备就重启** —— 返回空/超时。曾长期被误判为
   "esp_log hook 没生效"（v1.1.5 已修 hook，但崩溃照旧）
2. **OTA "升级失败/上传中断"** —— 网页 OTA 上传要几十秒，前端
   `setInterval(pollLog, 2000)` 每 2 秒抓一次 /log，**上传期间第一次
   轮询就崩** → 设备重启 → OTA 连接断 → 前端报"升级失败"。
   v1.1.5 修了 TWDT 也没用，因为这个崩得比看门狗还快
3. 任何轮询 /log 的监控脚本（mon485.ps1 等）都会把设备搞重启

**实锤过程**：只抓 /status 设备稳定运行 45s+；一抓 /log 就无响应，
随后 /status 显示 uptime 归零（重启过）。

## 修复

**固件（`src/web_server.cpp` handle_log）**：
- 三个栈数组改 `static`（handle_log 只在 loopTask 单线程调用，零风险）
- String json 改为一次 `reserve()` 到位，避免 48 次 `+=` 反复 realloc
- 排查其余 handler：/status 用堆 String、/bus485raw 用 malloc，
  均无大栈分配，无同类风险

**前端（`web/app.js`，双保险）**：
- `otaUpload()` 一开始就 `logPaused = true` —— 上传期间暂停日志轮询，
  不再和上传抢这台单客户端的 Web 服务器
- 失败/中断时恢复 `logPaused = false`，让设备日志面板把固件记录的
  失败原因显示出来（成功路径保持暂停等重启）

## 产物（串口刷）

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.6\bootloader.bin --partitions release\v1.1.6\partitions.bin --firmware release\v1.1.6\firmware.bin --no-watch
```

刷完验证：
1. 网页「设备日志」立刻有内容，**反复刷新不再重启设备**
2. 网页「系统升级」传 firmware.bin → 应显示"OTA 成功，设备将重启"
   （v1.1.5 修 TWDT + v1.1.6 修栈溢出，两层都通了）

| 文件 | 大小 | MD5 |
|---|---|---|
| bootloader.bin | 20160 | e53ee3cb94d6343b691b42d3f9dc66a4 |
| partitions.bin | 3072 | 4bb153dff163ba42cd36e1255c03b5bc |
| firmware.bin | 1188672 | 8809a963f9b50a67871489985d06bedb |
