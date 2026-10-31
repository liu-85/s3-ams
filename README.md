# s3-ams —— ESP32-S3 自动换料系统（AMS）

单电机 + 8 路电磁离合方案，通过 RS-485 以 **BambuBus 协议**直连拓竹（Bambu Lab）打印机。
板载 Web 界面负责配置、诊断和 OTA 升级，不需要拆机、不需要串口。

> 当前版本：**v1.1.27**（`1.1.27-s3ams`）—— 修退料时又送料 + 缓冲送料超时 WEB 可调
> 最新验证：**2026-10-25，A1 通道进料全流程通过**（详见 [docs/TEST-2026-10-25-A1-FEED.md](docs/TEST-2026-10-25-A1-FEED.md)）

---

## 一、这是什么

两个项目的合并：

| 来源 | 贡献了什么 |
|---|---|
| **Tianshu**（RP2040） | BambuBus 协议栈、485 状态机、打印机交互时序 |
| **esp32s3-485** | 硬件架构：ESP32-S3 + 单共享电机 + 8 路电磁离合 + 双微动电阻编码 + Web |

合并后的取舍：

- **保留** Tianshu 的 `BambuBus.cpp` 协议逻辑（`send_for_Dxx` / `process_motion_state_machine` / `get_pressure_value` 与上游逐行一致）
- **替换** 执行层：Tianshu 是 4 个独立电机（BMCU + AS5600），本工程是**一台电机 + 8 路离合**分时驱动
- **新增** 网页配置 / 诊断 / OTA、电流采样与堵转保护、485 原始报文抓取

> ⚠️ **关键硬件差异**：Tianshu 打印时电机不通电 = 耗材完全自由，所以它不需要张力控制；
> 本工程停料时压轮仍压着耗材，阻力来自齿轮组反向拖动，**必须主动跟随送料**。
> 这是机械结构决定的，不是移植遗漏。详见 [ARCHITECTURE.md](docs/ARCHITECTURE.md#六单电机--离合带来的特殊约束)。

---

## 二、硬件

| 项 | 配置 |
|---|---|
| MCU | ESP32-S3，16MB Flash，**无 PSRAM**，240MHz |
| 分区 | 双 OTA（`ota_0` @0x20000 / `ota_1` @0x300000，各 3MB）+ NVS + storage + coredump |
| 电机 | 1 台直流电机，AT8236 H 桥，PWM 调速 |
| 离合 | 8 路电磁离合，ULN2803 反相驱动，高电平吸合 |
| 传感器 | 每通道 2 只微动（开始位 / 停止位），电阻编码合占 1 个 ADC 脚 |
| 电流 | AT8236 ISEN → GPIO3（ADC1_CH2），10ms 采样 |
| 485 | 1.2288 Mbps，8E1，TX=GPIO14 / RX=GPIO48 / DE=GPIO39 |
| 指示 | 板载 LED（GPIO42）：蓝=待机 绿=动作 红=故障 |

完整引脚表、微动分压计算、离合映射见 [docs/HARDWARE.md](docs/HARDWARE.md)。

---

## 三、快速开始

### 3.1 编译

```bash
pio run -e s3ams
```

产物在 `.pio/build/s3ams/`：`bootloader.bin` / `partitions.bin` / `firmware.bin`。

> ⚠️ **不要开 PSRAM、不要用 QIO**。板子实际是无 PSRAM 的 16MB 型号，固件头标了 PSRAM
> 会导致 ROM boot 时 SHA-256 校验失败 + GPIO0 看门狗循环重启。`platformio.ini` 已对齐。

### 3.2 串口刷写（首次 / 刷砖）

```bash
python flash_s3.py COM6
```

脚本固化了正确偏移，这是踩过坑的地方：

| 段 | 偏移 |
|---|---|
| bootloader | `0x0` |
| partitions | `0x8000` |
| **app** | **`0x20000`** ← 不是 0x10000 |

同时会**擦除 otadata（0xF000）**——不擦的话 OTA 选中的还是旧分区，刷完重启版本号不变，
这是"刷机不生效"的总根源（v1.1.8 修复）。NVS（0x9000）不擦，WiFi 配置保留。

### 3.3 网页 OTA（日常升级）

浏览器打开 `http://<设备IP>` → 「系统升级」卡片 → 选 `firmware.bin` 上传。

前端按 `app_version` 版本号比对判断是否成功，**不看 HTTP 状态**——早期版本会出现
"提示失败但刷新后版本已变"（上传完成后设备重启，连接被掐，浏览器误判 onerror）。

### 3.4 配网

首次上电没有 WiFi 配置时进入 AP 模式：

- 热点：`ESP-AMS-XXXX`（XXXX = MAC 低 16 位）
- 密码：`12345678`
- 打开 `http://192.168.4.1` → 「WiFi 配置」→ 扫描 → 填密码 → 连接

连接成功后 AP 关闭，设备用 STA 模式运行。

---

## 四、Web 界面

单页应用，`web/` 是源码，构建时由 `tools/generate_web_files.py` 生成 `src/web_files.h`（已提交）。

| 卡片 | 作用 |
|---|---|
| 运行状态 | 版本、IP、WiFi、内存、运行时长 |
| 换料动作 | 16 个料位状态、计米、**清零计米** |
| 硬件状态 | 电机方向/占空比/电流、离合掩码、微动三态 |
| 上电诊断 | 复位原因、启动阶段、「接上负载就重启」查这里 |
| WiFi 配置 / 配置热点 | 配网 |
| 打印机 MQTT 配置 | 打印机侧参数 |
| 打印机通道设置 | 每个料位的颜色/耗材类型 |
| 硬件调试 | 直接驱动电机+离合（会动，注意安全） |
| 手动点动 | 非阻塞点动，测电流/占空比用 |
| 电机电流采样 | 采样电阻、堵转阈值标定 |
| 辅助送料 / 换色冲刷 / 退料参数 / 换料温度 | 换料工艺参数 |
| 微动开关 | 每路阈值与实时 mV |
| 系统升级 | OTA |

主要 HTTP 接口（`src/web_server.cpp`）：

```
GET  /status           全部状态（前端每秒轮询）
GET  /log              设备日志（环形缓冲 + seq 增量）
GET  /bus485raw?since= 485 原始报文（seq 增量，抓包用）
POST /wifi_scan  /wifi_connect  /wifi_set
POST /hardware_test  /jog_set  /current_set  /current_clear
POST /sensor_set  /access_set  /comm_set  /retract_set  /temper_set  /assist_set
POST /color_set  /current_channel_set  /act485_set
POST /meters_reset     清零 16 个料位的计米
POST /diag_reset       清离合冲突计数
POST /ota_upload       固件上传
```

---

## 五、目录结构

```
s3-ams/
├── platformio.ini            构建配置（env=s3ams）
├── partitions_16mb.csv       16MB 双 OTA 分区表
├── flash_s3.py               一键刷写（固化偏移 + 擦 otadata）
├── src/                      固件源码
│   ├── main.cpp              启动 / 主循环 / 看门狗 / 栈尺寸
│   ├── ams_log.h             日志宏劫持（串口 + 网页双写）
│   ├── board_pins.h          ★ 全部引脚定义，改接线只改这里
│   ├── AMCU.{h,cpp}          执行层：电机 + 离合 + 微动 + 跟随送料
│   ├── BambuBus.{h,cpp}      ★ 485 协议栈 + 状态机（移植自 Tianshu）
│   ├── clutch.{h,cpp}        8 路离合异步状态机
│   ├── motor.{h,cpp}         H 桥 PWM / 方向 / 软启停
│   ├── filament_sensor.{h,cpp} 双微动电阻编码 + 三档判定 + 滤波
│   ├── current_sense.{h,cpp} ISEN 采样 / 峰值 / 堵转保护
│   ├── settings_storage.{h,cpp} 参数持久化（NVS "s3ams"）
│   ├── bus485_raw.{h,cpp}    485 原始报文环形缓冲（32 帧）
│   ├── web_server.{h,cpp}    HTTP 服务 + 全部接口
│   ├── web_files.h           前端打包产物（自动生成，勿手改）
│   └── time64.{h,cpp}        毫秒时钟
├── web/                      前端源码（index.html / app.js / style.css）
├── tools/                    PC 端工具（见下）
├── release/vX.Y.Z/           每个版本的 bootloader/partitions/firmware + RELEASE.md
└── docs/                     本文档
```

代码结构、数据流、状态机详解见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

---

## 六、PC 端工具

| 工具 | 用途 |
|---|---|
| `tools/mon485.bat [IP] [ms]` | **485 抓包监控**：打印机帧（青）+ 我方应答（绿）+ 电机动作事件（黄）+ 丢帧告警（红），同时写 `tools/logs/*.log` |
| `tools/ana_log.bat` | 把抓到的日志压成一页：概况 / 状态机时间线 / 电机事件 / 异常 |
| `tools/motor_test.bat` | 电机三档判定（转 / PWM 没推动 / 软件没驱动） |
| `tools/log_monitor.py` | 只盯 `/log` + `/status`，轻量版 |
| `tools/ana485*.py` | 针对旧格式抓包的分析脚本（历史遗留，逐步被 `ana_log.py` 取代） |
| `flash_s3.py` | 串口刷写 |

用法与排错见 [docs/TOOLS.md](docs/TOOLS.md)。

---

## 七、文档索引

| 文档 | 内容 |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 代码结构、启动流程、主循环、状态机、NVS、实时性约束、踩坑清单 |
| [docs/PROTOCOL-485.md](docs/PROTOCOL-485.md) | 485 帧格式、字段偏移、状态码表、完整进料时序、抓包方法 |
| [docs/HARDWARE.md](docs/HARDWARE.md) | 引脚表、微动电阻编码、离合映射、电流采样、接线注意 |
| [docs/TOOLS.md](docs/TOOLS.md) | 工具脚本用法与常见问题 |
| [docs/TEST-2026-10-25-A1-FEED.md](docs/TEST-2026-10-25-A1-FEED.md) | **A1 通道进料成功总结**：现象、误判、根因、修复、验证 |
| [CHANGELOG.md](CHANGELOG.md) | v1.1.0 → v1.1.12 全部版本 |

---

## 八、已知遗留

| 项 | 说明 |
|---|---|
| 堵转阈值 `stall_ma = 0` | 只显示电流不保护。需实测堵转电流后填 |
| `on_use` 计米 | 仍用 `time_used_ms/300000` 估算，累计 10 秒后停止增长 |
| `on_use` 切换延迟 | 「送料满 5 秒才切 on_use」是 Tianshu 原逻辑，未改。若顶料就调小 |
| 485 环形缓冲 32 帧 | 设备忙时 60~90 帧/秒，只够 400ms。抓包脚本已用"抓帧优先"补偿，固件未改 |
| 只验证了 A1 | 其余 7 通道机械/电气待逐个验证 |

---

## 九、安全提示

- 「硬件调试」和「手动点动」会**真的驱动电机和离合**，手指别在齿轮附近
- 电磁离合长时间吸合会发热，固件里所有动作都有超时释放，别绕过
- 刷机时不要断电，OTA 期间设备会重启两次
