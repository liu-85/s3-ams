# 版本记录

固件版本号在 `src/web_server.cpp` 的 `APP_VERSION`，形如 `1.1.12-s3ams`。
每个版本的三件套（`bootloader.bin` / `partitions.bin` / `firmware.bin`）和详细
RELEASE.md 在 `release/vX.Y.Z/`。

---

## v1.1.12 —— 挤出失败 / 退料偶发不动：三个真实根因

**2026-10-25** · `9f9c037`

| # | 根因 | 修复 |
|---|---|---|
| 1 | `read=0xFF` 时**无条件**清空全部通道 + `now_filament_num=0xFF`，而打印机在"使用中"周期性发 `0x0F/FF` | 只有 `statu==0x00`（复位/取消）才清空 |
| 2 | `0x0F` 才是"使用中"，固件只认 `0x07`（日志里 `0x07/0x7F` 零帧） | `0x07` 与 `0x0F` 都接受 |
| 3 | `read=0xFF` 固定回 `pressure=0xFFFF`（无耗材） | `statu=0x03/0x0F` 时用当前料位回答 |
| 4 | `before_on_use` 期间电机停死，单电机+离合结构下拉不动 | **跟随送料泵**：送 250ms / 停 350ms |

附带：退料优先级最高（跳过缓冲逻辑，避免覆盖挂起的离合请求）、`statu=0x01`
停止广播不再清掉打印中通道、`/status` 新增 `phase` / `follow` 诊断字段。

**结果**：A1 通道进料全流程通过。详见 [docs/TEST-2026-10-25-A1-FEED.md](docs/TEST-2026-10-25-A1-FEED.md)

---

## v1.1.11 —— 计米单位错 1000 倍

**2026-10-25** · `d45631a`

`accumulate_feed_meters()` 把毫秒当秒用：

```cpp
// 旧：dt 是 ms，FILAMENT_PUSH_M_PER_SEC 是 m/s → 放大 1000 倍
// 实测：送料 3.3 秒报 141 米，第二次 818 米
float meters = ((float)dt / 1000.0f) * FILAMENT_PUSH_M_PER_SEC;
if (dt > 500) return;   // 主循环卡顿后不要一次补一大段
```

打印机按 `meters` 折算料盘余量，看到 141 米就认为"早已到位"，3.3 秒就发 `09/A5`
让挤出机拉料 → 拉不到 → 报挤出失败。

新增：`/status` 的 `feed.meters`、`POST /meters_reset` + 前端「清零计米」按钮
（旧固件的错误值已写进 flash，不重置打印机认为料已用光）。

> 这是真 bug，但**不是本次现象的主因**（修完现象没变）。

---

## v1.1.10 —— 第二次退料电机不动 + OTA"报失败但实际成功"

**2026-10-25** · `8ef3fe7`

- **退料**：`s_last_need_pull_back[]` 只在触发时置 `true`，**全项目没有地方清回 `false`**
  （本意让 `last_action_set()` 清，但那个函数定义了从未被调用）→ 第二次进
  `need_pull_back` 时条件不成立 → 整块退料驱动被跳过。改为离开状态时清 latch。
- **OTA**：上传完成后设备重启掐断连接，浏览器误判 `onerror`。前端改为按
  `app_version` 版本号比对判断成功，不看 HTTP 状态。

---

## v1.1.9 —— 送料电机不转：离合状态回读恒为 0

**2026-10-25** · `b88c244`

上层靠 `clutch_rb`（离合状态回读）判断是否吸合完成，回读恒 0 → 一直认为没吸合 →
反复重试 / 不给电机。修复回读逻辑。

---

## v1.1.8 —— 串口刷机不生效的总根源

**2026-10-25** · `631ccac`

- **`flash_s3.py` 现在擦除 otadata（0xF000）**。不擦的话 OTA 选中的还是旧分区，
  刷完重启版本号不变——这是"刷了 1.1.7 还是 1.1.6"的总根源。
- 日志中文按字节截断，切在 UTF-8 多字节中间 → 乱码/崩溃。改为 UTF-8 安全截断。

---

## v1.1.7 —— loopTask 栈 8KB → 32KB

**2026-10-25** · `c55fbe9`

`/log` 请求仍然 panic：`Stack canary watchpoint triggered (loopTask)`，实测用量 8.8KB。

不是某个函数有大数组，而是 **LTO 把 `_handleRequest` + `send` + String 拼装整条链
全内联**，几十个小帧累积吃掉 8.8KB。

用同名强符号覆盖 `getArduinoLoopTaskStackSize()` 返回 32768。
（不能在 `platformio.ini` 加 `-DCONFIG_ARDUINO_LOOP_STACK_SIZE`——`sdkconfig.h`
在命令行宏之后 include，会覆盖回去。）

---

## v1.1.6 —— `/log` 栈溢出崩溃（OTA"升级失败"真凶）

**2026-10-25** · `c580ee4`

`handle_log()` 里 7.7KB 的栈数组 → panic。改堆分配；上传期间暂停日志轮询。

---

## v1.1.5 —— OTA 上传中断（TWDT panic）+ 网页设备日志真正生效

**2026-10-25** · `3cda9d3`

上传写 flash 期间主循环被长时间占用 → TWDT 10s 超时 panic → 连接断。
上传期间退订看门狗，完成后恢复。

---

## v1.1.4 —— 网页 OTA HTTP 500 + 微动全通道滤波

**2026-10-25** · `9ade1f9`

- 3 参数 `server.on()` 的上传回调为 `null`，收不到事件 → 改正确签名
- 微动滤波原来只做了通道 1，其余通道仍乱跳 → 全通道统一

---

## v1.1.3 —— 送料时电机没电没转

**2026-10-25** · `4842baa`

**根因**：`AMCU_clutch_pump()` 咬合完成后调 `motor_set_dir(motor_get_dir())`，
而电机从未启动过 → `motor_get_dir()` 返回 STOP → 实际执行了 `motor_stop()`。

修复：`AMCU_drive_channel()` 记住 `s_pending_dir` / `s_pending_speed`，
pump 按记忆值 `motor_set_dir_speed()` 启动。

同时新增：`/bus485raw` 原始报文环形缓冲（32 帧）、`tools/mon485.bat` 抓包脚本、
`tools/motor_test.bat` 电机三档判定。

---

## v1.1.2 —— 网页设备日志面板为空

**2026-10-25** · `6432907`

`web_log()` 没有任何调用方（日志只进串口），且旧 `/log` 返回单串、
前端 `renderLog` 会 TypeError（面板双重坏）。

修复：用 `esp_log_set_vprintf` hook 截 `log_i/w/e` 进网页缓冲（串口输出不变），
`setup()` 最开头装 hook。缓冲改行式环形 48 行 × 160 字符 + seq，`/log` 返回
`{"log":[...],"seq":[...],"mem_free":N}`。

---

## v1.1.1 —— Web OTA 上传总是中断

**2026-10-25** · `99b2fb1`

**与网络无关**。Arduino-ESP32 的 `WebServer` 对非 multipart 的 POST
（`application/octet-stream`）会把整个 body 一次性读进 RAM；S3 不开 PSRAM 时堆只有
几百 KB，1.18MB 固件读到一半 realloc 失败 → `_parseRequest` 返回 false → 掐连接 →
浏览器 `onerror`「上传中断」。

修复：前端改 `FormData` multipart（不要手设 Content-Type）；
`handle_ota_upload` 按 Content-Length 开分区、逐块校验、失败 abort、全程日志。

---

## v1.1.0 —— 电流采样 / 485 开关 / 微动门限

**2026-10-25** · `4f008d4`

- `POST /act485_set` 原来走 `handle_ok` 占位，根本没写 `ams_enable` → 真写 + `/status` 回显
- `MOTOR_SPEED_485_FEED` 90 → 100（duty=255，电机启动阈值留裕量）
- 微动悬空 ~371mV 噪声落在 START 区误触发 → 新增 `BOARD_SENSOR_MV_NOISE = 350mV` 门限
- 新增 `src/current_sense.{h,cpp}`：AT8236 ISEN(GPIO3) 10ms 采样、IIR、1s 峰值窗、
  堵转保护（`r_mohm` / `stall_ma` / `stall_ms` / `blank_ms`，NVS `s3ams-cur`）。
  默认 `stall_ma=0` 只显示不保护，阈值需实测标定。
- 新增手动点动：`amcu_jog_start/pump`（非阻塞），`/jog_set` 存参数、`/hardware_test` 执行

---

## v1.0.x —— 合并与硬件对齐（早期）

- `adeceef` s3-ams：合并 Tianshu 协议 + esp32s3-485 硬件（ESP32-S3）
- `b339282` GitHub Actions 在线编译
- `01c0244` 修复 Web UI 卡顿 + 无限送料
- `4d1af78` **对齐可用配置：16MB flash，不开 PSRAM**（标了 PSRAM/QIO 会 ROM 校验失败 + 循环重启）
- `ffebf4c` 一键刷写脚本：bootloader@0x0 + partitions@0x8000 + app@0x20000，保留 NVS
- WiFi 配网系列：AP 扫描 → 关 AP 释放射频 → STA 连接（修 `NO_AP_FOUND`）；
  `wifi_connect` JSON 偏移从 +7/+11 修正为 **name+8 / password+12**

---

## 工具脚本

| 日期 | 变更 |
|---|---|
| 2026-10-25 | `tools/mon485` 重写为抓帧优先：丢帧 13% → 0.7%，日志落盘，每帧带 `#seq` + 丢帧告警，★电机动作事件，帧解码只认 `type=0x04` |
| 2026-10-25 | 新增 `tools/ana_log.py` / `ana_log.bat`：日志压成一页（概况 / 状态机时间线 / 电机事件 / 异常） |
| 2026-10-25 | `tools/motor_test.bat`：电机三档判定 |
| 2026-10-25 | `tools/log_monitor.py`：轻量 `/log` + `/status` 监控 |
