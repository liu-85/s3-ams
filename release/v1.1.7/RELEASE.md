# s3-ams v1.1.7 发布说明（loopTask 栈 8KB→32KB —— /log 栈溢出彻底修复）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**上一版**：v1.1.6（把 handle_log 数组改 static，但 /log 仍然 panic）

## 现场（v1.1.6 刷完的串口日志）

```
[2742][I][main.cpp:55] setup(): 系统初始化完成，进入主循环
Guru Meditation Error: Core  1 panic'ed (Unhandled debug exception).
Debug exception reason: Stack canary watchpoint triggered (loopTask)
Backtrace: ... 0x42007c8e (handle_log) ... 0x4200df3a (_handleRequest)
           ... 0x42006e25 (loop)
```

`rst:0xc (RTC_SW_CPU_RST)` 每 ~3 秒一轮 —— **启动进主循环就崩，无限重启**。

## 根因：不是某个大数组，是整条调用链累积 8.8KB

- v1.1.6 已把 handle_log 的 `char lines[48][160]`（7.7KB）改成 static，
  反汇编确认 `handle_log` 自身栈帧只有 **112 字节**（`entry a1, 112`），
  .bss 里也确实有 `handle_log::lines`（7680 字节）
- 但 panic 现场 **SP = 0x3fcaeae0，已越过栈底约 652 字节**，
  实测栈用量 ≈ **8.8KB > loopTask 的 8KB**
- 消耗者不是单个函数（全固件扫描：没有任何函数栈帧 >1KB），
  而是 **LTO 把 `_handleRequest + send + String 拼装` 整条链全内联**，
  几十个小帧累积吃掉 8.8KB —— 只要有一个 `/log` 请求就必然爆栈
- 这也解释了为什么 v1.1.5 只在抓 /log 时崩（不抓就稳定 45s+）

## 修复

**1. 主循环栈 8KB → 32KB（`src/main.cpp`）**
```cpp
size_t getArduinoLoopTaskStackSize(void) { return 32768; }
```
Arduino core 里这个函数是 `__attribute__((weak))`，用同名强符号覆盖即可
（链接器 map 已验证符号来自 `src/main.cpp.o`，core 的 8KB 版被丢弃）。
S3 有 320KB RAM，多 24KB 无压力（RAM 23.4%）。

> 为什么不在 platformio.ini 加 `-DCONFIG_ARDUINO_LOOP_STACK_SIZE`：
> `sdkconfig.h` 在命令行宏之后 include，会把值重新覆盖回 8192。

**2. 顺手修：任务看门狗配置一直没生效**
```
E (1050) task_wdt: esp_task_wdt_init(515): TWDT already initialized
```
Arduino 核心启动时已按默认（**5 秒** + 监控 idle 任务）初始化过 TWDT，
我们的 `esp_task_wdt_init` 失败 → 想要的 10s/不监控 idle 配置根本没上。
现在先 `esp_task_wdt_deinit()` 再 init，配置真正生效。

**3. 新增自诊断字段 `/status → stack_free`**
`uxTaskGetStackHighWaterMark(NULL)` 返回主循环任务历史最小剩余栈。
稳定运行后应 >20KB；若掉到 2KB 以下说明还要加栈。

## 产物（串口刷）

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.7\bootloader.bin --partitions release\v1.1.7\partitions.bin --firmware release\v1.1.7\firmware.bin --no-watch
```

刷完验证：
1. 串口应打印 `主循环任务栈: 32768 字节（32 KB）`
2. 网页「设备日志」反复刷新不再重启；`/status` 里 `stack_free` 应 >20000
3. 网页「系统升级」传 firmware.bin → "OTA 成功，设备将重启"

| 文件 | 大小 | MD5 |
|---|---|---|
| bootloader.bin | 20160 | e53ee3cb94d6343b691b42d3f9dc66a4 |
| partitions.bin | 3072 | 4bb153dff163ba42cd36e1255c03b5bc |
| firmware.bin | 1190208 | d9ca6066cc7627b24954c0c32f2dbde5 |
