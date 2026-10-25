# 代码结构与运行原理

面向要改这份固件的人。看完应该能回答：**哪个文件管什么、主循环每轮做了什么、
一个 485 帧从进来到电机转起来经过哪几层、哪些地方一碰就会出事。**

---

## 一、分层

```
┌──────────────────────────────────────────────────────────────┐
│  web/  (index.html + app.js + style.css)                     │
│  ↓ HTTP 每秒轮询 /status，增量拉 /log、/bus485raw             │
├──────────────────────────────────────────────────────────────┤
│  web_server.cpp      HTTP 路由、JSON 拼装、OTA、WiFi 配网      │
├──────────────────────────────────────────────────────────────┤
│  AMCU.cpp            ★ 执行层：决定"电机怎么转、哪个离合吸合"   │
│    ├─ clutch.cpp     8 路离合异步状态机（吸合要时间，不能阻塞）  │
│    ├─ motor.cpp      H 桥 PWM / 方向 / 软启停                  │
│    ├─ filament_sensor.cpp  双微动电阻编码 → 三档 + 滤波        │
│    └─ current_sense.cpp    ISEN 采样 / 峰值 / 堵转保护          │
├──────────────────────────────────────────────────────────────┤
│  BambuBus.cpp        ★ 协议层：收帧 → 解析 → 改状态机 → 应答    │
│    bus485_raw.cpp    旁路抓包（32 帧环形缓冲，供 PC 端轮询）    │
├──────────────────────────────────────────────────────────────┤
│  board_pins.h  引脚   settings_storage.cpp  NVS   time64.cpp  │
└──────────────────────────────────────────────────────────────┘
```

**单向依赖**：`web_server` / `AMCU` 都可以调 `BambuBus` 的查询函数，
但 `BambuBus` **不调** `AMCU`——协议层只管"现在该是什么状态"，不管电机怎么转。
这条边界是移植 Tianshu 时保留下来的，破坏它会让状态机和执行层互相打脸。

---

## 二、模块职责

| 文件 | 管什么 | 关键入口 |
|---|---|---|
| `main.cpp` | 启动顺序、主循环、TWDT、loopTask 栈尺寸 | `setup()` / `loop()` / `getArduinoLoopTaskStackSize()` |
| `ams_log.h` | 劫持 `log_i/w/e`，串口 + 网页双写 | `web_log_install_hook()` |
| `board_pins.h` | **全部引脚**，改接线只改这里 | — |
| `AMCU.cpp` | 执行层总调度、退料超时、跟随送料、点动、LED | `AMCU_init/run/motion` |
| `BambuBus.cpp` | 485 收发、CRC、帧解析、**状态机**、应答拼装、计米存储 | `BambuBus_init/run` |
| `clutch.cpp` | 离合请求队列（单槽）+ 吸合/释放时序 | `clutch_request_single()` / `clutch_pump()` |
| `motor.cpp` | PWM 占空比、方向、软启停 | `motor_set_dir_speed()` |
| `filament_sensor.cpp` | ADC 采样、分压反推、三档判定、中值滤波 | `sensor_read_channel()` |
| `current_sense.cpp` | 10ms 采样、IIR、1s 峰值窗、堵转 latch | `current_sense_poll()` |
| `settings_storage.cpp` | 参数 NVS 持久化（命名空间 `s3ams`） | `settings_load/save_to_flash()` |
| `bus485_raw.cpp` | 原始报文环形记录，自旋锁保护 | `bus485raw_push()` / `dump()` |
| `web_server.cpp` | 全部 HTTP 接口 + 前端文件（`web_files.h`） | `web_server_start/handle()` |

---

## 三、启动流程

```
setup()
 ├─ Serial.begin(115200)
 ├─ web_log_install_hook()        ← 必须最早，否则漏掉启动日志
 ├─ watchdog_init()               ← 10s，只监控主循环任务
 ├─ settings_storage_init()       ← NVS 读参数
 ├─ AMCU_init()
 │   ├─ init_io_pins()            离合/电机/LED 全置安全态
 │   ├─ motor_init() / clutch_init()
 │   ├─ filament_sensor_init()
 │   ├─ current_sense_init()
 │   ├─ BambuBus_init()           485 起 1.2288Mbps 8E1，读 NVS 料盘数据
 │   └─ jog_config_load()
 └─ web_server_start()            WiFi + HTTP
```

---

## 四、主循环

```cpp
void loop() {
    watchdog_feed();
    AMCU_run();            // ← 几乎所有事都在这
    web_server_handle();   // 处理一个 HTTP 请求
    delay(10);
}
```

`AMCU_run()` 展开：

```cpp
AMCU_run()
 ├─ 跟踪 get_now_filament_num() 变化 → 需要时落盘
 ├─ 退料 latch 管理（进入 need_pull_back 置位，离开必须清位）
 ├─ BambuBus_run()            ← 收帧 + 解析 + 改状态机 + 应答
 │    └─ BambuBus_UART_Poll()   while(RS485.available()) RX_IRQ(byte)
 ├─ check_buffer_status()     ← 打印时按微动补料（退料期间整段跳过）
 └─ AMCU_motion()             ← 把"当前状态"翻译成电机+离合动作
```

### ⚠️ 收帧是轮询，不是中断

`BambuBus_UART_Poll()` 在主循环里 `while(RS485.available())` 逐字节读。
**主循环任何一处阻塞超过 ~400ms，485 就会丢帧**——因为 `bus485_raw` 只有 32 帧，
设备忙时每秒 60~90 帧。历史上 `/log` 拼 JSON、OTA 写 flash 都踩过这个坑。

改代码时的红线：
- 不要在 `loop()` 链路上加 `delay()` 或长循环
- 大块内存操作（OTA、JSON 拼装）要么分片，要么在期间暂停业务

---

## 五、状态机

### 5.1 内部状态（`_filament_motion_internal`）

| 值 | 状态 | 含义 | 谁驱动电机 |
|---|---|---|---|
| 0 | `idle` | 空闲 | 停 |
| 1 | `send_out` | 送料 | AMCU 正转 |
| 2 | `before_on_use` | 打印机已咬住，正在往里拉 | **跟随送料泵**（脉冲） |
| 3 | `on_use` | 正常打印 | 缓冲微动按需补料 |
| 4 | `stop_on_use` | 暂停 | 停 + 断离合 |
| 5 | `before_pull_back` | 准备退料 | 停（等打印机拉出） |
| 6 | `pull_back` | 退料 | AMCU 反转 |

### 5.2 485 编码 → 内部状态

`process_motion_state_machine()`（`BambuBus.cpp:237`）：

| 485 `statu` / `motion` | 内部状态 | 前置条件 |
|---|---|---|
| `0x03` / `0x00` | `send_out` | 无条件 |
| `0x09` / `0xA5` | `before_on_use` | `allow_any` |
| `0x07`或`0x0F` / `0x7F`或`0xFF` | `on_use` | `allow_any` + 送料已满 5 秒 |
| `0x07`或`0x0F` / `0x00` | `stop_on_use` | `allow_stop` |
| `0x09` / `0x3F` | `before_pull_back` | `allow_stop` |
| `read=0xFF` + `statu=0x03` | `pull_back` | 由 `set_filament_motion()` 设 |

```cpp
allow_any  = (loaded == 0xFF) || (loaded == 本通道);   // 没占用 或 就是自己
allow_stop = (loaded == 本通道);                        // 必须是自己
// loaded 来自 get_loaded_filament()，now_filament_num=0xFF 时恒为 0xFF
```

> **`allow_stop` 是退料的命门**。`now_filament_num` 一旦被错误清成 `0xFF`，
> `allow_stop` 恒 false，`09/3F` 和 `03/00/FF` 全被拒 → 退料电机不动。
> v1.1.12 修的正是这个连锁（`read=0xFF` 时不再无条件清空）。

### 5.3 内部状态 → 执行（`AMCU_motion()`）

**⚠️ v1.1.15 起，服务通道不是 `cur`，而是仲裁结果 `svc`。**

```cpp
// ★ 先仲裁：只要有通道在 need_pull_back / need_send_out / act_*_mm，
//   本循环就服务它，打印态 / 缓冲补料全部让位。
//   详见 find_motion_channel() 上方的长注释。
int svc = cur;
int mc  = find_motion_channel();
if (mc >= 0) svc = mc;

if (is_need_pull_back) {            // 退料优先级最高，整段独占
    反转 + 保持离合，超时 PULLBACK_TIMEOUT 后放弃
} else if (is_channel_printing(svc)) {
    吸合离合 + 绿灯
    if (is_channel_before_on_use(svc)) follow_feed_pump(svc+1);   // 脉冲跟随
    else                                follow_feed_reset_ex(true);
} else {
    按 need_send_out / waiting 走送料或停
}
```

`switch(act)` 里所有 `cur` 也一并换成 `svc`（`s_last_action` /
`last_pullback_meters` / `get_filament_meters` / `is_filament_online` /
`AMCU_drive_channel(svc+1, ...)`）。

**为什么必须仲裁**：打印中换料时 `cur` 往往还指着旧通道（它还是 `on_use`），
而真正要动的电机是新通道的 `need_send_out`。直接用 `cur` 会
① 走进"打印中"分支直接 `return`，新通道进料永远执行不到；
② 和 `check_buffer_status()` 互抢 `s_pending_clutch` 单槽，
离合反复吸放"咔咔响"。详见第六节。

---

## 六、单电机 + 离合带来的特殊约束

这是本工程和 Tianshu 最本质的区别，也是最容易改错的地方。

| | Tianshu（BMCU） | 本工程 |
|---|---|---|
| 电机 | 每通道 1 个，共 4 个 | **1 个共享**，8 路离合选通 |
| 停料时耗材状态 | 电机不通电 = **完全自由** | 压轮常压，**被齿轮组拽住** |
| 打印时张力控制 | 不需要 | **必须主动跟随送料** |

### 6.1 单电机的硬约束：同一时刻只能服务 1 个通道

离合器选通 + 单电机，意味着**任何两个"要动电机"的需求都不能同时成立**。
所以代码里所有会驱动电机的地方都必须先过一道仲裁：

```cpp
/* 优先级：退料 > 进料 > 打印中缓冲补料 > before_on_use 跟随泵 */
static int find_motion_channel(void);
```

两个调用点：

| 位置 | 作用 |
|---|---|
| `check_buffer_status()` 开头 | 有进/退料 → 缓冲补料整体 `return`，不去吸合 |
| `AMCU_motion()` 开头 | 有进/退料 → `svc` 指向它，不走"打印中"分支 |

**违反这个约束的后果**（v1.1.15 修的现场故障）：
`s_pending_clutch` 是**单槽**，`s_pending_ch` 是单值。两个模块每循环交替
`clutch_request_engage(不同通道)` → 后台任务反复"全断 60ms + 吸合 80ms" →
现场听到"咔咔咔"，电机永远等不到稳定吸合，**料退不出也送不进**。

判断是否又在互抢：网页「AMS」卡片 → **离合仲裁**，
`target_switch` / `target_abort` 两个计数。正常打印时只在真正换料时各 +1；
若每秒涨几十就是在打架，超 50 次会标红。

### 6.2 缓冲器跟随送料

Bambu 官方 wiki 对缓冲器的说明：

> By controlling the feed speed of the AMS, we can make sure the tension on the filament
> is within a given range so that the toolhead extruder can work easily and precisely.

所以 `before_on_use` 阶段不能"停着不动"，而是：

```cpp
#define FOLLOW_FEED_ON_MS   250    // 送 250ms
#define FOLLOW_FEED_OFF_MS  350    // 停 350ms
// 100% PWM ≈ 50mm/s → 平均 50 × 250/600 ≈ 21mm/s
// 略高于挤出机拉料速度（10~20mm/s），保证不绷紧；
// 停的 350ms 让缓冲器吸收多送的料，不顶进喷嘴。
```

**为什么不用微动驱动这一段**：微动是 ADC 三态分压，现场信号不稳
（悬空 371mV 就落在 START 区），而 `before_on_use` 只持续几秒，等微动触发太慢。

**注意**：`follow_feed_pump()` 在"停"相位只 `motor_stop()`，**保留离合**
（下一拍还要接着送，反复吸合会磨离合）。

---

## 七、关键数据流

### 7.1 计米

```
送料驱动周期 ──> accumulate_feed_meters(f)
                    meters = (dt_ms / 1000) * FILAMENT_PUSH_M_PER_SEC(0.05)
                    add_filament_meters(f, meters)
                 ──> 写 filament[ams][ch].meters
                 ──> Bambubus_set_need_to_save() 落 NVS
```

> **v1.1.11 修的就是这里**：原来 `dt` 是毫秒却当秒用，放大 1000 倍，
> 送料 3.3 秒报 141 米。打印机按 meters 折算余量，于是判"早就到位"，
> 3.3 秒就发 `09/A5` 让挤出机拉料 → 拉不到 → 报挤出失败。

### 7.2 `/status` 的 `feed` 段

```json
"feed": {
  "cur": 0,                    // 当前料位
  "motion_text": "waiting",    // API 层状态（need_send_out/need_pull_back/waiting）
  "phase": "before_on_use",    // ★ 细分阶段，v1.1.12 新增
  "follow": 1,                 // ★ 跟随泵当前是否在"送"相位
  "meters": 0.1523,
  "clutch_mask": 1, "motor_pct": 100,
  "online": 1, "printing": 0, "tripped": 0, "jog": 0, "clutch_rb": 1, "cmd": 0
}
```

`phase` 是诊断的关键——`motion_text` 把 `before_on_use` / `on_use` / `stop_on_use`
全映射成 `waiting`，光看它分不出当前在哪一步。

---

## 八、NVS 命名空间

| 命名空间 | 内容 |
|---|---|
| `s3ams` | 料盘数据（颜色/类型/meters）、通道模式、退料超时、AMS 数量 |
| `s3ams-wifi` | SSID / 密码 |
| `s3ams-cur` | 采样电阻、堵转阈值、时间窗 |
| `s3ams-jog` | 点动时长、占空比 |
| `s3ams-bus` | **上报给打印机的 AMS 固件版本号**（v1.1.13） |

刷机时 `flash_s3.py` **不擦 NVS**，WiFi 和料盘数据保留。
`otadata` 则**必须擦**，否则 OTA 仍从旧分区启动（刷机不生效的总根源）。

---

## 九、实时性与资源

| 约束 | 值 | 说明 |
|---|---|---|
| loopTask 栈 | **32KB** | 用强符号覆盖 `getArduinoLoopTaskStackSize()`。LTO 把 WebServer 整条链内联后实测吃 8.8KB，默认 8KB 必崩 |
| TWDT | 10s，只监控主循环 | Arduino 核心已初始化过，必须先 `deinit` 再 `init` |
| 485 帧缓冲 | 32 帧 | 只够 400ms 流量 |
| 电机电流 | 10ms 采样 | IIR + 1s 峰值窗 |
| 离合吸合 | 异步，单槽队列 | `s_pending_clutch` 只有一个槽，谁都能覆盖谁 |

---

## 十、踩坑清单（改代码前必读）

| # | 坑 | 后果 | 版本 |
|---|---|---|---|
| 1 | `s_pending_clutch` 只有单槽 | 缓冲器逻辑覆盖挂起的退料吸合 → 退料不动 | v1.1.12 加优先级 |
| 2 | `s_last_need_pull_back[]` 只置不清 | 第二次退料不触发 | v1.1.10 |
| 3 | 离合咬合后调 `motor_set_dir(motor_get_dir())` | 拿到 STOP → 送料时电机不转 | v1.1.3 |
| 4 | `clutch_rb` 回读恒 0 | 上层以为没吸合，反复重试 | v1.1.9 |
| 5 | 计米 `dt` 毫秒当秒 | 放大 1000 倍 → 打印机提前判到位 | v1.1.11 |
| 6 | `read=0xFF` 无条件清空状态 | 状态反复被抹 → 挤出失败 + 退料不动 | v1.1.12 |
| 7 | 只认 `0x07` 不认 `0x0F` | 状态永远卡在 `before_on_use` | v1.1.12 |
| 8 | `read=0xFF` 固定回 `pressure=0xFFFF` | 打印机认为空料/掉线 | v1.1.12 |
| 9 | 微动悬空 371mV 落在 START 区 | 误触发补料 | v1.1.0 加 350mV 门限 |
| 10 | 固件头标 PSRAM / QIO | ROM 校验失败 + 循环重启 | — |
| 11 | OTA 用 `octet-stream` 裸 POST | 整包进 RAM → realloc 失败 → 上传中断 | v1.1.1 改 multipart |
| 12 | 刷机不擦 otadata | 版本号不变（实际没换分区） | v1.1.8 |
| 13 | 3 参数 `server.on()` 上传回调为 null | HTTP 500 | v1.1.4 |
| 14 | 中文日志按字节截断 | UTF-8 断在多字节中间 → 乱码/崩溃 | v1.1.8 |
| 15 | 上报的 AMS 版本号是编造的 `90.00.00.00` | 新版打印机固件按白名单校验 → 弹「仅支持 AMS / AMS 2 Pro / AMS HT」 | v1.1.13 |
| 16 | `AMCU_motion()` 用 `cur` 当服务通道、且 `is_channel_printing(cur)` 提前 `return` | 打印中换料时旧通道还 `on_use` → 新通道进料永不执行；且和 `check_buffer_status()` 互抢 `s_pending_clutch` 单槽 → 离合咔咔响、料退不出也送不进 | v1.1.15 |
