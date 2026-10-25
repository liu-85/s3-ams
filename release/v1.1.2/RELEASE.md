# s3-ams v1.1.2 发布说明（设备日志面板修复）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO 6.1.18, env=s3ams）

## 修复：网页"设备日志"面板永远是空的

**根因**：`web_log()` 函数定义了，但**固件里没有任何地方调用它** ——
所有日志都走 `log_i/log_w/log_e`（只进串口），所以 `/log` 永远返回空串，
网页日志面板也是空白（且旧 `/log` 返回单串格式，前端 `renderLog` 拿到会
`lines.join` TypeError，属于双重坏）。

**修复**：
- `esp_log_set_vprintf()` 挂 hook：所有 `log_i/log_w/log_e` 输出截一份进
  网页日志缓冲（串口输出完全不变）；setup() 最开头安装，不漏启动日志
- 日志缓冲从"单字符环"改为**行式环形缓冲**（48 行 × 160 字符，
  带单调递增 seq），多任务写入用自旋锁保护
- `/log` 改为返回 `{"log":[...], "seq":[...], "mem_free":N}`，
  与前端 `renderLog` 的 seq 增量协议对齐（只追加新行，不整屏闪）

## 附带：PC 侧日志监控脚本

`tools/log_monitor.py`（纯标准库，无需装包）：

```bash
python tools/log_monitor.py --ip 192.168.2.100 --interval 1
```

- 轮询 `/log`：新日志带本地时间戳实时打出（新旧固件格式都认：
  行数组+seq 优先，单串自动退后缀重叠检测）
- 轮询 `/status`：每秒一行紧凑状态——电机方向/离合/电流/1s 峰值/
  堵转 latch/跳闸次数/act485；**电机起停、堵转跳闸、离合切换、
  电流突变 ±300mA 时额外出醒目行**
- 输出同时写 `logs/monitor-时间戳.log`，断网自动退避重连
- `--status-only` / `--log-only` 可单看一路

## 产物（一键刷写）

```bash
python flash_s3.py COM6 \
  --bootloader release/v1.1.2/bootloader.bin \
  --partitions release/v1.1.2/partitions.bin \
  --firmware   release/v1.1.2/firmware.bin
```

刷完后网页"设备日志"立即可用，PC 监控脚本也会自动切到行格式输出。
