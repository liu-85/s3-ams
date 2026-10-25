# PC 端工具

都在 `tools/` 下，全部是**只读监控**或**显式动作**，不会偷偷改设备状态。
需要 Python 的脚本用系统 Python 3（`python --version` 能跑就行，只用标准库）。

---

## 一、`mon485.bat` —— 485 抓包（最常用）

```bat
tools\mon485.bat                        :: 默认 192.168.2.100 / 50ms
tools\mon485.bat 192.168.2.100 50
tools\mon485.bat 192.168.2.100 20       :: 看到丢帧告警时用这个
```

### 颜色

| 颜色 | 内容 |
|---|---|
| 青 | `RX` 打印机 → 本机 |
| 绿 | `TX` 本机 → 打印机 |
| 灰 | 状态行 / WEB 日志 |
| **黄** | **★电机动作事件**：电机方向 / 离合 / 动作 / 打印阶段 / 占空比 任一变化 |
| 红 | 错误日志、`⚠ 丢帧` 告警 |

### 输出长这样

```
[17:23:38.343] RX #52380 t=583871 len=13 [3D] 3DC50DF10400010003FF001281  <<停止/空闲 ams1 read=0xFF
[17:23:38.344] TX #52381 t=583872 len=60 [3D] 3DC03C0204017501110F0F0FFF...
[17:23:38.869] 状态: 电机=停止/100% 离合=0x00 电流=0mA 峰值=0mA act485=1 | 料位0 动作=waiting 在线=1 打印=0 堵转=0 点动=0 回读=1 cmd=10 计米=1.310m 阶段=none 跟随=0 微动=S00/377mV
[15:06:10.221] ★电机 电机 停止 -> 进料 | 离合 0x00 -> 0x01 | 动作 waiting -> need_send_out | 阶段 none -> before_on_use
```

### 日志文件

所有输出同时写 `tools\logs\mon485_yyyyMMdd_HHmmss.log`（`logs/` 已在 `.gitignore`）。
**控制台滚掉的一行不少**，Ctrl+C 停之后用记事本 / VS Code 打开。

### 关于丢帧

设备端 485 环形缓冲只有 **32 帧**，设备忙时 60~90 帧/秒，只够存 400ms。
脚本按"抓帧优先"设计：主循环几乎只拉 `/bus485raw`，`/log` 和 `/status` 降频到 400ms 一次。

每帧带 `#seq`。**seq 不连续会打红字**：

```
[17:23:42.221] ⚠ 丢帧 9 条（seq 52716 -> 52726）
```

看到这个就把间隔改小（`mon485.bat 192.168.2.100 20`），或关掉别的页面/脚本。

实测：默认 50ms 时丢帧率约 **0.7%**，且集中在启动瞬间。

### 状态行字段

| 字段 | 含义 |
|---|---|
| `电机=进料/100%` | 方向 / 占空比 |
| `离合=0x01` | 8 路离合掩码，bit0 = 通道 1（A1） |
| `电流 / 峰值` | ISEN 实时值 / 1 秒峰值 |
| `料位N` | 当前料位（0 起） |
| `动作` | `need_send_out` / `need_pull_back` / `waiting` |
| `打印` | 是否处于打印态 |
| `回读` | 离合状态回读（1 = 与请求一致） |
| `计米` | 本通道已用米数 |
| **`阶段`** | **`none` / `before_on_use` / `on_use` / `stop_on_use`** ← 诊断关键 |
| **`跟随`** | 跟随送料泵当前是否在"送"相位 |
| `微动=S00/377mV` | `S<停止位><开始位>` + 原始电压 |

---

## 二、`ana_log.bat` —— 把日志压成一页

```bat
tools\ana_log.bat                              :: 自动取最新日志
tools\ana_log.bat tools\logs\xxx.log
tools\ana_log.bat tools\logs\xxx.log 15:06:09 15:06:12   :: 只看这个时间窗
```

输出四段：

1. **概况** —— 时间跨度 / RX TX 帧数 / 丢帧数 / 状态变化次数
2. **状态机时间线** —— 只列打印机发来的 Dxx 帧，**按 ams 通道分别去重**
   （4 路轮询每 40ms 一轮，不去重会刷爆）
3. **★电机事件** —— 每一次电机/离合/阶段变化
4. **异常** —— 丢帧告警、堵转 LATCH

原始日志一秒上百行，肉眼追不上状态跳变；这个脚本就是给"回看"用的。

---

## 三、`motor_test.bat` —— 电机三档判定

```bat
tools\motor_test.bat                        :: 默认 通道1 / 1.5s / 100%
tools\motor_test.bat 192.168.2.100 3 2 80   :: IP / 通道3 / 2秒 / 80%
```

| 判定 | 含义 |
|---|---|
| **转** | 方向变了 **且** 峰值电流 ≥150mA → 正常 |
| 方向给了但没电流 | PWM 不够或离合没吸合 |
| 方向都没给 | 软件没驱动（固件/协议问题） |

> 电机 **PWM 低于 80% 起不来**，测的时候别用低占空比。

---

## 四、`log_monitor.py` —— 轻量日志/状态监控

```bash
python tools/log_monitor.py                    # 默认 192.168.2.100，1 秒
python tools/log_monitor.py --ip 192.168.2.100 --interval 1
python tools/log_monitor.py --status-only      # 只看状态
python tools/log_monitor.py --log-only         # 只看日志
```

不抓 485 原始帧，只轮询 `/log` + `/status`，负载小。适合长时间挂后台。

---

## 五、`flash_s3.py` —— 串口刷写

```bash
python flash_s3.py COM6
python flash_s3.py COM6 --bootloader bl.bin --partitions pt.bin --firmware fw.bin
python flash_s3.py COM6 --no-watch             # 不抓 boot 日志
```

- 固化偏移：bootloader `0x0` / partitions `0x8000` / **app `0x20000`**
- **擦除 otadata（0xF000）** ← 不擦的话刷完重启还是旧分区（"刷机不生效"的根源）
- **不擦 NVS（0x9000）**，WiFi 和料盘数据保留
- 刷完硬复位并抓 45s 串口日志，校验 `Loaded app from partition at offset 0x20000`

依赖：`pip install esptool pyserial`

---

## 六、`ana485*.py` —— 旧格式分析脚本

针对早期 `tools/新建 文本文档.txt` 那种 `[HH:MM:SS] RX/TX t=ms len=N HEX` 格式。

| 脚本 | 用途 |
|---|---|
| `ana485e.py` | 0xC5 帧 `(statu,motion,read)` 分布 + 状态变化时间线 |
| `ana485f.py` | 只打 ams0 的动作帧时间线 |
| `ana485g.py` | 提取我方 Dxx 应答的 `use_flag` / `pressure` / `meters` |
| `ana485h.py` | `python tools/ana485h.py 15:06:09 15:06:12` 按时间窗打完整 RX/TX 交错对话 |

**新抓包请用 `ana_log.py`**，这一批是历史遗留，保留只为复现老日志。

---

## 七、常见问题

| 现象 | 处理 |
|---|---|
| `/bus485raw 取不到` | 固件低于 v1.1.3，或 IP 不对。浏览器直接开 `http://<ip>/bus485raw` 试 |
| 一直红字丢帧 | 间隔改 20ms；关掉 Web 页面；确认没有别的脚本在轮询 |
| 状态行 `阶段=none` 但 `动作=need_send_out` | 打印机还在重试送料，没进 `before_on_use`。看 485 时间线 |
| 日志文件是空的 | `tools\logs` 无写权限，或脚本启动就崩了（看控制台第一屏） |
| `.bat` 双击闪退 | 在 cmd 里跑，能看到报错 |
