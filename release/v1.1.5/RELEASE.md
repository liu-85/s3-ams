# s3-ams v1.1.5 发布说明（OTA 上传中断修复 + 网页设备日志真正生效）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**上一版**：v1.1.4（OTA 500 已修，但出现新的"上传中断"；/log 全空）

## 修复 1：网页 OTA"上传中断"（真正根因：任务看门狗 panic）

**现象**：v1.1.4 上传固件，进度条走着就断，浏览器提示"上传中断"；
期间设备自行重启（实测 uptime 从 124s 掉到 9s）。

**根因**：`main.cpp` 用 `esp_task_wdt_add(NULL)` 把**主循环任务**订阅进了
10 秒任务看门狗（TWDT，`trigger_panic=true`）。multipart 上传时，
`server.handleClient()` 会在**一次调用内**把整个 1.18MB 收完
（边收边写 flash，几十秒不返回）——主循环卡在里面喂不了狗 →
第 10 秒 TWDT **panic 重启** → 浏览器连接被 reset → "上传中断"。
（v1.1.4 修的 500 只是让 upload 事件能送达，没解决这个。）

**修复（`src/web_server.cpp`）**：
- `UPLOAD_FILE_START`：`esp_task_wdt_delete(NULL)` 把主循环临时退出看门狗
- `UPLOAD_FILE_END` / `UPLOAD_FILE_ABORTED`：`esp_task_wdt_add(NULL)` 恢复订阅
- 每写完一个数据块（1436B）兜底 `esp_task_wdt_reset()`——万一退订失败也死不了
- `web_server_handle()` 加保险丝：20s 没有上传进度就恢复订阅，
  防客户端半途断开后系统长期裸奔
- 成功后响应改为 JSON（`{"ok":true,"info":...}`）——前端 `xhr.onload`
  解析的是 JSON，v1.1.4 发纯文本会被误判成"上传失败（HTTP 200）"
- 发送成功响应后 `delay(500)` → `delay(1000)`，弱信号下留足 flush 时间

## 修复 2：网页「设备日志」面板永远是空的

**根因**：一直以为挂上 `esp_log_set_vprintf` 钩子就能截到 `log_i`。
实际 Arduino 的 `log_i` **不走 esp_log**：
`log_i → log_printf → ets_printf`（ROM 函数，直接走串口 putc1 钩子），
`esp_log_set_vprintf` 只能截 IDF 的 `ESP_LOGx` 通道。所以钩子装了个寂寞。

**修复（新增 `src/ams_log.h`，所有 .cpp 第一个 include）**：
- 对 `log_i/log_w/log_e/log_n` 做宏劫持：**串口输出原样不动**
  （还是原来的 `ARDUHAL_LOG_FORMAT + log_printf`），额外抄送一份
  纯消息文本进网页日志环（`web_logf → web_log`）
- IDF 组件日志（WiFi 等）仍由 `esp_log_set_vprintf` 兜底，两条通道互补

## 改进：连续相同消息自动折叠

日志环加"连续相同消息合并"：与上一条完全相同的消息不占新行、
不推进 seq（前端按 seq 增量渲染，面板不再被刷屏），行尾显示
` ×N` 次数。例：启动横幅两行 `====` 会显示成一条 `==== ×2`。

## 关于"WEB 打开慢"

实测（curl，RSSI −73dBm）：小文件/大文件均偶发 1.7~2.1s 卡顿，
其余 0.15~0.4s；主循环空闲无阻塞（离合已在后台任务）。
形态符合**弱信号 TCP 重传（RTO≈1s）**，不是固件卡死。
建议：电脑/路由器离板子近一些，或改善路由覆盖。
固件侧已确认 `esp_wifi_set_ps(WIFI_PS_NONE)` 生效（不开 WiFi 省电）。

## 产物（本次必须走串口刷）

设备上 1.1.4 的网页 OTA 正是本版要修的问题，先用 COM6 串口刷：

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.5\bootloader.bin --partitions release\v1.1.5\partitions.bin --firmware release\v1.1.5\firmware.bin --no-watch
```

刷完验证：
1. 网页「设备日志」应立即有启动日志（`s3-ams - ESP32-S3 AMS 固件` 等）
2. 网页「系统升级」再传一次同一个 firmware.bin → 应显示
   "OTA 成功，设备将重启"，**不再上传中断**（之后升级就走网页）

| 文件 | 大小 | MD5 |
|---|---|---|
| bootloader.bin | 20160 | e53ee3cb94d6343b691b42d3f9dc66a4 |
| partitions.bin | 3072 | 4bb153dff163ba42cd36e1255c03b5bc |
| firmware.bin | 1188112 | b4fb8d8949cf76bc8a56f3470e1afc8e |
