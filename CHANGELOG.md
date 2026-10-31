# 版本记录

固件版本号在 `src/web_server.cpp` 的 `APP_VERSION`，形如 `1.1.12-s3ams`。
每个版本的三件套（`bootloader.bin` / `partitions.bin` / `firmware.bin`）和详细
RELEASE.md 在 `release/vX.Y.Z/`。

---

## v1.1.27 —— 修「退料时又送料」+ 缓冲送料超时 WEB 可调

**2026-10-31**

现场：**打印结束、退料过程中开始微动一触发，又把料送进挤出机了。**

**根因：`statu == 0x09` 在 485 协议里是歧义的**，必须结合 motion_flag：

| statu | motion_flag | 含义 | 该送料吗 |
|---|---|---|---|
| 0x09 | 0xA5 | before_on_use（准备使用） | ✅ |
| 0x09 | 0x3F | before_pull_back（**准备退料**） | ❌ |

`is_channel_active_recently()` 只看 statu 就判"在用" → 退料准备阶段准入放行 →
缓冲器因料刚退走而空 → 开始微动触发 → 又送料。

`if (is_need_pull_back) return;` 也挡不住：退料 3 秒超时后该标志就被清了，
但打印机可能还在退料流程里（motion 仍是 pull_back）。

**修法**：
1. `is_channel_active_recently()` 排除 `before_pull_back` / `pull_back`
2. 新增 `is_channel_pulling_back()`，`check_buffer_status()` 准入前显式让出
3. `buffer_feed_holds_motor()` 同步排除（退料不许被缓冲送料顶掉）

**新增缓冲送料超时 WEB 可调**：`POST /feed_timeout_set {"sec":n}`，
默认 30 秒，范围 5~120，NVS `s3ams-feed` 的 `feed_timeout`。
按实测正常补料 7~15 秒，30 秒留一倍余量。到点强制停并打日志，
之后要等微动松开才重新计时（不是每 30 秒重试一次）。

改动：`src/BambuBus.cpp` / `.h` / `src/AMCU.cpp` / `src/settings_storage.h` /
`.cpp` / `src/web_server.cpp` / `web/index.html` / `web/app.js` / `src/web_files.h`

---

## v1.1.26 —— 修「停印后电机一直转」+ 电流剖析 + 送料 PWM 可调

**2026-10-31**

现场：**"都停止打印了，电机还是一直在转"**。

**根因（v1.1.23 埋的）**：v1.1.23 让 `AMCU_motion()` 在
`s_buffer_feed_decision==1` 时不停电机，但 `decision` **只在
`AMCU_buffer_assist_feed()` 内部被清零**。一旦准入失败（打印机不发了 /
该通道不在用 / cur 切走），那个函数再也不会被调用 → decision 永远停在 1 →
电机无限转。**需要清理的时候，恰恰是清理函数不会被调用的时候。**

**修法三层**：
1. 判据收紧 —— 新增 `buffer_feed_holds_motor(ch)`：decision 之外还要看
   有起点、未超 30s 硬上限、15s 内有反馈、该通道仍"在用"，五条全满足才让位
2. 独立看门狗 —— `buffer_feed_watchdog()` 放在 `AMCU_run()` 最前面，
   不受任何 early-return 影响；清掉脏 decision，并在电机确实被这一路占着时停机
3. 窗口统一 —— `BUFFER_IN_USE_WINDOW_MS=5000`，准入与让位必须用同一个值
   （否则"准入开着、让位关了"，电机只抖一下）

**效果**：停止打印后最多 5 秒电机自动停。

**移除微动日志**：删掉每 3 秒一条的「缓冲被挡：…」（现场嫌吵）。
排查改用 `/sensor_raw` 或 `/status` 的 `feed.statu` / `feed.active`。

**新增电机电流剖析**：每次连续运行 ≥0.5s 停止时打印
`电流统计: 进料 PWM=100% 时长=3200ms 启动峰值=1980mA 稳态均值=420mA 稳态峰值=505mA 全程最大=1980mA`
- 启动峰值用**原始值**（`current_sense_raw_ma()` 新增）—— IIR(shift=3) 时间常数
  约 80ms，会把 50~200ms 的启动尖峰削掉一大截，按滤波值定阈值会定得太低 → 误跳
- 稳态均值/峰值用**滤波值**，不被噪声带偏
- 网页「电机电流采样」卡片显示「最近一次运行」；`/status` 的 `current.last`
- 跟随送料的 250ms 脉冲不计（否则日志被刷爆）

**新增自动送料 PWM 可调**：`POST /feed_pwm_set {"pct":n}`，范围 40~100，默认 100，
立即生效并写 NVS `s3ams-feed`（独立命名空间，避免改动 SettingsData 导致老配置校验和失效）。
网页「参数设置」页新增卡片。实测 PWM 低于 80 电机起不来，填 40~79 会提示。

改动：`src/AMCU.cpp` / `src/motor.h` / `src/motor.cpp` / `src/current_sense.h` /
`src/current_sense.cpp` / `src/settings_storage.h` / `src/settings_storage.cpp` /
`src/web_server.cpp` / `web/index.html` / `web/app.js` / `src/web_files.h`

---

## v1.1.25 —— 485 链路诊断 + 缓冲送料超时兜底

**2026-10-31**

现场：刷 v1.1.24 后又报 `[0500-40A3] AMS(或AMSlite)A 通信异常`，而 v1.1.22 通信正常。

**先确认不是协议回归**：逐行比对 v1.1.22 → v1.1.24 的 diff，
`BambuBus.cpp` 只加了 `last_statu`/`last_statu_ms` 两个字段赋值和 3 个只读
查询函数，**协议帧、应答、台号逻辑一行未动**。时间线也指向硬件
（通信正常是在焊线之前测的，焊完线就报错）。

**新增 485 链路计数**，把 40A3 定性成四种情况之一：

| 现象 | 结论 |
|---|---|
| `rx_bytes` 不涨 | 物理层没数据（线松/焊点虚/收发器坏/A-B 反） |
| `rx_bytes` 涨 + `crc_err` 猛涨 | 干扰/共地不良/电源纹波 |
| `rx_frames` 涨 + `tx_402`=0 | 我们没应答序列号查询 |
| `tx_402` 涨 + 仍报 40A3 | 应答内容不对（台号/长度/型号指纹） |

`/status` 的 `ams.link` 输出：`rx_bytes` / `rx_frames` / `crc_err` /
`overflow` / `tx_402` / `tx_103` / `tx_06` / `last_rx_age_ms`。
`/diag_reset` 一并清零这些计数，便于改接线后对照。

**缓冲送料连续运行超时兜底（30 秒）**：
v1.1.23 让 `AMCU_motion()` 在 `decision==1` 时不停电机，副作用是微动卡在
触发态（常闭接反/短到 GND）时 `decision` 恒 1、`start_trig` 每轮刷新反馈
时间戳 → 15 秒无反馈超时永不触发 → **电机无限转** → 大电流让电源纹波变大
→ 485 误码。现加第二道闸 `BUFFER_FEED_MAX_MS = 30000`，超时强制停并打日志。

改动：`src/BambuBus.cpp` / `src/BambuBus.h` / `src/AMCU.cpp` / `src/web_server.cpp`。

---

## v1.1.24 —— 微动原始电平诊断 + 修缓冲开关关不掉

**2026-10-31**

**症状**：通道 2~4 的微动在 WEB 上完全不变（一直"触发"），只有通道 1 会变。
v1.1.14 测试微动时正常。

**先确认不是回归**：`git log -- src/filament_sensor.cpp` 显示最后一次改动就是
`71a38a7`（v1.1.14），之后一行没动；引脚表 `BOARD_PIN_SENSOR_ARRAY` 也没动。
所以先加可观测性，不动逻辑。

**新增**：

- `GET /sensor_raw` —— 8 只微动的**原始** `digitalRead` 电平 + 去抖状态 + 启用掩码
  ```
  {"mode":"digital","mask":15,"pins":[1,2,9,8,10,11,12,13],
   "raw":[1,0,0,0,1,1,1,1],"trig":[[0,0],[0,1],[0,1],[0,1]]}
  ```
- `filament_sensor.cpp`：`sensor_raw_level(idx)` / `sensor_pin_of(idx)`
- `/status` 的 `sensors` 段增加 `raw` 数组

**判读**：

| 现象 | 结论 |
|---|---|
| 按下 `raw` 1→0 | 硬件正常，问题在软件 |
| `raw` 恒 1 | 没接通 / GND 没共 |
| `raw` 恒 0（松开也不回 1） | 接了**常闭(NC)**触点 / 短路 / 引脚被占 |

**顺带修 bug**：前端 `app.js` 发的是 `enabled`，后端找的是 `"enable"`
（`"enable"` 带引号在 `"enabled"` 里匹配不上）→ `enable` 恒 -1 →
`enable != 0` 恒 true → **缓冲送料开关只能开、关不掉**。现在两种写法都认。

**没动**：引脚表、`SENSOR_TEMP_DIGITAL` / `SENSOR_TEMP_ACTIVE_LOW` 宏、
去抖 `N=3`、v1.1.23 的缓冲准入逻辑。

---

## v1.1.23 —— 修流量校准阶段微动不送料

**2026-10-31**

**症状**：流量校准阶段，送料微动压下（WEB 上显示"开始位(空) 触发"）但电机不转、
不送料。v1.1.14（微动直连、不看状态机）测试正常。

**根因（两处叠加）**：

1. **准入太严** —— `check_buffer_status()` 用 `is_channel_printing()` 做准入，
   它只认 `on_use` / `before_on_use` / `stop_on_use`。流量校准时打印机会持续
   从 AMS 抽料，但状态机 `motion` 可能一直停在 `idle`（帧没匹配上）
   → 整段 `return`，微动触发完全没有电机动作。

2. **刚送就被停** —— 即使放行，`AMCU_run()` 的执行顺序是
   `check_buffer_status()` → `AMCU_motion()`。前者刚吸合离合、驱动电机，
   后者 `get_filament_motion()` 把 `idle` 映射成 `waiting`，
   `case waiting:` 里一句 `AMCU_drive_stop()` 立刻把电机停掉。
   现场表现就是"微动触发了，电机不动/只抖一下"。

3. 附带：`AMCU_buffer_assist_feed()` 里 `s_buffer_feed_decision = 1` 只在
   `AMCU_drive_channel()` 返回 true 时置位 —— 离合**已吸合**（走 else 分支）
   或正在等吸合时都不置位，导致上面第 2 条的判据失效。

**改动**：

- `BambuBus.cpp`：`_filament_ext` 增加 `last_statu` / `last_statu_ms`，
  在 `process_motion_state_machine()` 每次收到状态帧时记录
- `BambuBus.cpp`：新增 `is_channel_in_use()`（含 `send_out`）、
  `is_channel_active_recently(ch, 10s)`（看 statu 是否 0x03/0x07/0x09/0x0F）、
  `get_channel_last_statu()`
- `AMCU.cpp` `check_buffer_status()`：准入改为
  `is_channel_in_use(cur) || is_channel_active_recently(cur, 10000)`，
  并加限流诊断日志（每 3 秒最多一条，只在"有触发但被挡"时打）
- `AMCU.cpp` `AMCU_motion()` `case waiting`：`s_buffer_feed_decision[svc] == 1`
  时不停电机
- `AMCU.cpp` `AMCU_motion()` 兜底 `else` 分支：缓冲送料中只清跟随状态、
  不停电机
- `AMCU.cpp` `AMCU_buffer_assist_feed()`：`start_trig` 一进来就置
  `s_buffer_feed_decision = 1`
- `web_server.cpp`：`/status` 的 `feed` 段增加 `statu`（最近一次 statu 位掩码）
  和 `active`（10s 内是否判为在用）

**"其他通道不读"的限制不变** —— 仍然只认 `cur` 一路。

**诊断用法**：流量校准中若微动还是不动，看 WEB 右侧日志有没有
`缓冲被挡：cur=N ... last_statu=0xXX`；或看 `/status` 的 `feed.statu`。
`0x00/0x01` = 打印机没把这一路当在用，`0x03/0x07/0x09/0x0F` = 在用。

---

## v1.1.22 —— 修 v1.1.21 引入的 `[0500-40A3]` AMS A 通信异常

**2026-10-31**

**症状**：刷 1.1.21 后打印机弹
`AMS(或AMS lite) A 通信异常，请拔插模块连接线或重启打印机。[0500-40A3]`，
1.1.20 及以前正常。

**根因**：v1.1.21 把 `0x402` 序列号包的越界处理从 **clamp 到 0** 改成了
**直接 return 不应答**（当时想统一成"其他长包越界就不回"的写法）。

0x402 的台号不一定在 `payload[33]`：包长 ≤ 33 时会退到 `payload[0]` 兜底，
而 `payload[0]` 不一定是台号，值一旦 ≥ `AMS_COUNT` 就 `return` →
打印机**一个序列号包都收不到** → 判定 AMS-A 通信异常、无法打印。

参考实现 `esp32s3-485/main/comm_485.c:on_version()` 同样有 `pl[0]` 兜底，
但紧接着是 `if (unit >= AMS_UNITS) unit = 0;` —— **clamp，不是 return**。
这是我 1.1.21 偏离参考的地方。

**改动（三处回退）**：

- `send_for_long_packge_version()` `0x402`：`return` → `AMS_num = 0`（恢复 clamp）
- `send_for_Fxx()` `0x06`：撤销 `if (req_num >= AMS_COUNT) return;`，恢复无条件回包
  （0x06 是打印机逐台握手帧，少回一次同样会被记成通信异常）
- `0x103` 版本包：22 字节 → **21 字节**，台号只写 `[20]`（回到 1.1.20 已验证的形态）

**保留 1.1.21 的**：

- `AMS_COUNT = 2`（符合参考实现 `BOARD_CHANNEL_COUNT / 4` = 8/4 = 2）
- `statu = (a < AMS_COUNT) ? online : offline`（幽灵 C/D 报 offline、不注册）
- `0x402` 读 `datas[33]` 前先查 `data_length > 33`（防越界读，与参考一致）

**原则**：链路上宁可多回一个"台 0 的序列号"（最多耗材页多张卡片，不影响打印），
也绝不能让任何一次查询出现**无应答** —— 无应答直接等于通信异常。

---

## v1.1.21 —— 只保留 AMS-A / AMS-B（修耗材页重复 AMS）

> ⚠️ **本版的 0x402 return / 0x06 return / 22 字节版本包已被 v1.1.22 回退**
> （导致 `[0500-40A3]` AMS A 通信异常）。台数改 2 的改动保留。

**2026-10-30**

**症状**：打印机耗材页显示 `AMS(4)`，但列表里出现 AMS-A 1 个、AMS-B 3 个、
AMS-C 3 个、AMS-D 2 个，共 9 张卡片，其中大量是重复的空卡。

**根因（四个叠加）**：

1. `DEFAULT_AMS_COUNT = 4`，但本板 8 通道 ÷ 4 料位 = **2 台**，C/D 是幽灵设备
2. v1.1.16 把 **全部 4×4 = 16 个料位**设成 `online`，`send_for_Fxx()` 的 0x05
   注册循环条件正是 `statu == online` → 幽灵 C/D 也被注册
3. `0x402` 序列号包越界时 `AMS_num = 0` **照样应答**（其他长包都是 return），
   打印机问台 2/3/4… 时收到"台 0 的序列号"→ 可能认为又发现一台
4. 版本包只有 **21 字节**，台号只写 `[20]`；esp32s3-485 是 22 字节 + `[20]`/`[21]`
   都写（BMCU370 读 `[20]`、Tianshu 读 `[21]`）

**改动**：

- `settings_storage.h`：`MAX_AMS_COUNT = 2`、`DEFAULT_AMS_COUNT = 2`
- `settings_storage.cpp`：NVS 旧值 4 不合法 → 回落默认 2
- `BambuBus_init()`：`statu = (a < AMS_COUNT) ? online : offline`（只有真 2 台在线）
- `send_for_long_packge_version()`：
  - `0x402` 越界 `return`，`datas[33]` 前先查 `data_length > 33`
  - `0x103` 包 21 → **22 字节**，`[20]`/`[21]` 都写台号
- `send_for_Fxx()`：`0x06` 握手 `buf[5] >= AMS_COUNT` 直接 `return`

**没改（有意保留）**：

- `use_flag`（Dxx 应答 `[19]`）语义错位 —— 送料中报 `0x02`、使用中报 `0x04`、
  空料位强制 `0x02`。这套从 Tianshu 继承，A1 进料已实测跑通，改它风险大于收益。
  详见 `docs/PROTOCOL-485.md` 4.1 节。
- `0x06` 的 `res_for_06_num` 台号锁死 —— 当初为修"第二台不出现"加的，
  本次只加越界保护。若仍重复，下一步动它。

**文档**：`docs/PROTOCOL-485.md` 新增 4.1 `use_flag` 语义对照、
4.2 `[41]` 2bit 料位位域、4.3 HMS 错误码不在 485 上、5.4 AMS 台数规则。

---

## v1.1.20 —— 修通道颜色设置不生效 + 说明全部折叠 + 文档补 IO 定义


**2026-10-26**

**现象**：通道设置选完颜色、弹窗关闭后，颜色**两秒内变回原色**。

**根因（两处叠加）**：

1. 前端 `applyStatus` 在固件不返回 `access_list` 时走 `else if (!accessFromDevice)`，
   该分支**从不置 `accessFromDevice=true`** → 每 2 秒轮询都重建整块为默认色。
2. 固件**根本不存** `access_list`/`color_list`，`/access_set` 是 `handle_ok` 空占位。

**修复**：

- 前端：默认映射只画一次；设备数据真变才重建；只有「使用中」变时走 `updateCurrentOnly()`。
- 固件：新增 `s_access_list[16]`/`s_color_list[16]`，存 NVS `s3ams-access`；
  `/status` 输出两数组；`/access_set` 改为 `handle_access_set()` 真保存；
  新增 `json_get_int_array()` 解析 JSON 数组。

**说明折叠**：

- `collapseTips()` 显式 `det.open=false`；微动页动态说明每次重绘后也 `collapseTips()`。
- OTA 说明修正：`esp-ams-s3.bin`（ESP-IDF 遗留）→ **`firmware.bin`**（PlatformIO）；
  去掉「MicroPython 版」段落。
- 「退料参数」卡片加提示：本卡当前固件未实现，调退料时长用「退料时间」卡片。

**文档**：`docs/HARDWARE.md` 新增 1.4 完整 GPIO 速查（按 GPIO 号）、
1.5 临时直连微动配对表，并标注 ADC2 陷阱。

---

## v1.1.19 —— WEB 控制台改版 + 网络配置补齐 MQTT 设置

**2026-10-26**

**需求**：
1. 按 `E:\ai\sw\esp-ams静态控制台` 模板改版 WEB 页面。
2. 网络配置里 MQTT 设置缺失（打印机 IP / 访问码 8 位 / 序列号），只剩一个「保存并连接」，补齐。

**WEB 改版**（三件套整体替换，固件接口不变）：

- 左侧栏 + 4 分组导航（监控 / 连接 / 调试 / 系统），序号 1-7
- 新增独立「参数设置」页；微动并入「硬件调试」；打印机配置并入「网络配置」
- 通道颜色改为**点色块弹窗选色**（预设 / 其他 / 自定义）
- 硬件点动 **8 通道双列**（左 1-4 / 右 5-8）
- 右侧固定日志列（可收起）；说明文字默认折叠

**MQTT 设置补齐（根因）**：

- 模板是静态控制台，MQTT 表单原本靠 `GET /get_mqtt_info` **动态生成**；
  无设备/无接口时请求失败 → 表单不渲染 → 只剩「保存并连接」按钮。
- 修复：表单改为 **index.html 静态字段**（打印机 IP / 访问码 8 位 / 序列号 /
  用户名 / 端口 / 客户端 ID），永远显示；`loadMqttForm()` 改为只填值不重建 DOM。
- 固件新增 `GET /get_mqtt_info`（不回传访问码）+ `POST /mqtt_connect`（存 NVS
  `s3ams-mqtt`，掉电保留）；`/status` 的 `mqtt_configured` 动态化。
- ⚠️ 本固件走 RS-485 直连，**MQTT 参数只保存，不实际连接**。

**顺带补齐接口**（避免 404 弹「请求失败」）：

- `POST /stop` → **真实现**：取消点动 + 停电机 + 断全部离合（新增 `amcu_jog_stop()`）
- `POST /ap_set` → **真实现**：`wifi_start_ap()` / `wifi_stop_ap()`
- `GET /boot_clear`、`POST /creep_set`、`POST /assist_test`、`POST /extruder_src_set`
  → 统一回 `{"ok":false,"info":"本固件（RS-485 直连）不支持该操作"}`（新增 `handle_unsupported()`）

**新增**：`json_get_str()`（简易 JSON 字符串字段解析，无需引入 ArduinoJson）。

---

## v1.1.18 —— 退料时间改为 4 秒 + WEB 可调

**2026-10-26**

**需求**：自动换料退料时间太长，改为 **4 秒**，并在 WEB 增加设置项。

**改动**：

- `DEFAULT_PULLBACK_TIMEOUT` 由 **7 → 4**（秒）
- WEB 新增「**退料时间**」卡片：输入框 3~30 秒、默认 4，点保存立即生效并写入 NVS
- 后端新增 `POST /pullback_set {"sec": 4}`，`/status` 新增 `pullback_sec` 回显
- 新增 `settings_set_pullback_timeout_all()` / `settings_get_pullback_timeout_common()`
  （本硬件单电机+8 离合，全局一个值，保存时统一写 16 路）
- `AMCU.cpp` 退料超时兜底值硬编码 `7` → 改用宏 `DEFAULT_PULLBACK_TIMEOUT`，与默认值同步

**⚠️ 关于 v1.1.17 的重要更正**：
v1.1.17 判定的「离合没吸合彻底就给电机」后来经用户确认**真正根因是电机减速箱打齿**
（换电机后一切正常），不是软件时序问题。
但「等离合机械稳定后再加载」本身是合理保护（减少离合磨损），**予以保留未回退**。
直接刷 v1.1.18 即可，不必单独刷 v1.1.17。

**⚠️ 页面上有两个「退料」卡片，别搞混**：

| 卡片 | 接口 | 是否生效 |
|---|---|---|
| **退料时间**（本次新增） | `/pullback_set` | ✅ 生效，控制换色退料超时 |
| **退料参数**（连续退料/蠕动） | `/retract_set` | ❌ 不生效，后端是 `handle_ok` 空占位（esp32s3-485 遗留 UI） |

**遗留**：退料仍是「到时强制停」的时间兜底，不是「退到位自动停」——本机没有可靠的到位反馈。

---

## v1.1.17 —— 离合没吸合彻底就给电机

**2026-10-25**

**现象**（用户实测 v1.1.16）：WEB 硬件调试点动逐通道测试，
**进料时电机在转但当前通道离合"没吸合彻底"**；
**退料却正常**，手摸电磁齿轮转得很快、明显吸合彻底。

**根因：电机在电磁铁还没机械咬合时就开始带负载。**

`clutch_apply_one()` 是「先 `gpio_set_level()` 再 `vTaskDelay(CLUTCH_ENGAGE_MS=80ms)`」，
所以 `clutch_is_engaged()` 在电平置高后**立即**返回 true，而机械咬合还要 80ms。
上层却靠 `clutch_is_engaged()` 决定何时给电机 → 主循环下一轮（几毫秒）就
`motor_set_dir_speed()` → 带着负载吸合 → 打滑、吸合不实。

**为什么退料"看着正常"（最易误判）**：

| | 进料 | 退料 |
|---|---|---|
| 离合状态 | 多要从上一通道切过来 | 往往已吸着同一通道 |
| `clutch_engage()` | 完整流程 全断60+静默20+吸合80 = **160ms** | **幂等分支立即返回** |
| 结果 | 没过稳定期就给电机 → 吸不实 | 早就稳定 → 彻底 |

退料"彻底"是假象——它根本没经历等待。

**修复**：`AMCU_clutch_pump()` 增加 `settled` 判据，两个条件满足其一才给电机：

```cpp
const bool settled = done ||                                   /* 后台已跑完，80ms 已过（权威） */
    ((uint32_t)(millis() - s_engage_req_ms) >=
     (CLUTCH_ENGAGE_MS + CLUTCH_SETTLE_MS));                    /* 时间兜底，防 req_slot 被覆盖 */
if (eng && settled) motor_set_dir_speed(s_pending_dir, s_pending_speed);
```

配套：
- `AMCU_drive_channel()` 记 `s_engage_req_ms`；"已吸合"分支若本通道还在等稳定则返回 false
- `amcu_jog_pump()`（WEB 点动）在吸合未稳定时不在这里给电机，交给 pump 统一启动
- 新增 `amcu_engage_pending()`（前向声明）

效果：电机比之前晚约 100~160ms 启动，换来离合完全咬合后再带负载。
**点动时长建议 ≥ 1000ms**（默认值），200ms 下限会被稳定时间占掉大半。

详见 [release/v1.1.17/RELEASE.md](release/v1.1.17/RELEASE.md)

---

## v1.1.16 —— WEB 点动进料不吸合 + 微动只认当前使用通道

**2026-10-25**

**现象**（用户实测 v1.1.15）：WEB 硬件调试用「进料/退料」按键逐个通道测试，
**进料时当前通道电磁离合不吸合**，退料正常。
用户同时明确：微动只在打印时当前使用通道缓冲送料时启用，
其他通道不管什么状态都不做动作、状态也不读取。

**根因一：点动期间 `check_buffer_status()` 照样在跑**

`AMCU_run()` 里 `check_buffer_status()` 在 `AMCU_motion()` **之前**调用，
而 jog 的让出保护只写在 `AMCU_motion()` 里。于是点动期间缓冲补料照常运行：
停止位微动触发 → `AMCU_drive_stop()` 断掉点动刚请求的离合；
开始位微动触发 → `AMCU_drive_channel(别的通道)` 抢离合槽位。
单槽 `s_pending_clutch` 被反复改写 → 离合吸不上。

连带 bug：`AMCU_drive_stop()` 原来无条件清 `s_pending_dir`，
一旦夹在「发起吸合」和「吸合完成」之间，pump 就按 STOP 启动 →
**离合吸合了但电机不转**。

**根因二：微动遍历全部 8 个通道**

```cpp
for (int i = 0; i < BOARD_CHANNEL_COUNT; i++)
    if (is_channel_printing(i)) AMCU_buffer_assist_feed(i);   // 每个 printing 通道都动
```

单电机 + 单离合槽位，同一时刻只能服务 1 个通道。多通道同时 printing 会互抢；
打印中在 WEB 点动别的通道时，旧通道微动仍在动作，把点动的离合覆盖掉。

**修复**：
- 微动只认「当前使用通道」，四个条件全满足才读才动：
  `cur` 在本板 8 通道内 + `is_channel_printing(cur)` +
  `sensor_channel_enabled(cur)` + 非 `before_on_use`
- 点动期间缓冲补料完全让出：`if (amcu_jog_busy()) return;`
- `AMCU_drive_stop()` 有挂起吸合请求时不再清 `s_pending_dir`

**新增诊断**：`/status` 的 `feed` 段加 `jog_ch` / `jog_dir` / `jog_ms_left`。
WEB 点「通道 3 进料」时 `jog_ch` 应为 3、`jog_dir` 为 1、`clutch_mask` 应为 `0b100`；
不一致就说明离合被别处抢走。

详见 [release/v1.1.16/RELEASE.md](release/v1.1.16/RELEASE.md)

---

## v1.1.15 —— 打印中换料：离合咔咔响、料退不出也送不进

**2026-10-25**

**现象**（用户实测）：网页手动把料送入料管正常；**打印中换料**时离合"咔咔响"，
料既退不出来也送不进去；停止打印后再点动该通道，离合也吸不上。

**根因：两个模块互抢同一个离合槽位。**

本机是「1 台电机 + 8 路离合」，同一时刻只能服务 1 个通道。但打印中换料时：

| 模块 | 看到什么 | 做什么 |
|---|---|---|
| `check_buffer_status()` | 旧通道还是 `on_use` | 按缓冲微动 `engage(旧)` |
| `AMCU_motion()` 的 switch | 新通道 `need_send_out` | `engage(新)` |

两者每循环交替，而 `s_pending_clutch` 只有**一个槽**（`s_pending_ch` 单值）→
目标通道每循环被改一次 → 后台任务反复"全断 60ms + 吸合 80ms" → **咔咔响**，
电机永远等不到稳定吸合 → 两个方向都不转。

**第二个 bug**：`is_channel_printing(cur)` 分支在 `switch(act)` 之前就 `return`。
打印中 `cur` 往往还指着**旧通道**（它还是 `on_use`），于是新通道的
`need_send_out` **永远执行不到** → 料送不进去。

**修复**：引入统一通道仲裁。

```c
/* 优先级：退料 > 进料 > 打印中缓冲补料 > before_on_use 跟随泵 */
static int find_motion_channel(void);   /* 找出在 need_pull_back/need_send_out/act_*_mm 的通道 */

// check_buffer_status()：有进/退料就整体让出
if (find_motion_channel() >= 0) return;

// AMCU_motion()：服务通道从 cur 改成 svc（仲裁结果）
int svc = cur;
int mc = find_motion_channel();
if (mc >= 0) svc = mc;
if (ams_enable) act = get_filament_motion(svc);
...
else if (is_channel_printing(svc)) { ... }
```

`switch` 里所有 `cur` 一并换成 `svc`（`s_last_action` / `last_pullback_meters` /
`get_filament_meters` / `is_filament_online` / `drive_channel(svc+1, ...)`）。

**新增诊断**（网页「AMS」卡片 → 离合仲裁）：

| 字段 | 含义 |
|---|---|
| `target_switch` | 挂起的吸合目标被改成别的通道的次数 |
| `target_abort` | 挂起的吸合请求被"停机"打断的次数 |

正常打印这两个数很慢地涨（只在真正换料时各 +1）。若"咔咔响"时每秒涨几十，
就是又在互抢。超过 50 次网页会标红提示。`/diag_reset` 一并清零。

**已排除的疑点**（都查过，不是原因）：
- 离合层 `clutch_engage()` 的"残留吸合复核" —— `clutch_apply_all_off()` 会先清
  `s_active`，软件记账兜底不会误判
- `clutch_assert_single(true)` —— 只在 >1 路同时吸合时才动作
- `follow_feed_reset_ex(true)` —— 只 `motor_stop()`，不动离合
- 堵转保护 —— `stall_ma` 默认 0（不保护）
- `AMCU_drive_stop()` 每循环刷队列 —— 有 `already_releasing` / `already_off` 幂等保护

详见 [release/v1.1.15/RELEASE.md](release/v1.1.15/RELEASE.md)

---

## v1.1.14 —— 临时直连微动模式（电阻编码微动未到货的过渡版）

**2026-10-25**

**背景**：4.7K/10K 电阻编码微动还没到货，先用 8 只普通微动直接接原传感器 IO，
能测 4 通道换色。

**约束**：只改「微动怎么读」，其他一行不动 —— AMCU / 缓冲补料 / 485 状态机 /
网页 / 计米 / 离合全部保持原样，上层 `sensor_triggered()` API 没变。

**改动**：
- `filament_sensor.h` 新增两个宏：`SENSOR_TEMP_DIGITAL`（默认 1）、
  `SENSOR_TEMP_ACTIVE_LOW`（默认 1，压下为低）
- `filament_sensor.cpp` 新增 `sensor_dbg_apply()` / `sensor_poll_digital()`，
  用 `digitalRead` 读 8 脚，通道 c 的开始位 = 微动 c、停止位 = 微动 c+4
- 后 4 个"通道"的 `s_enabled_mask` 位被清掉（它们的 IO 已被当成停止位用掉）
- `/status` 的 `sensors` 段新增 `"mode":"digital"`
- 启动日志逐通道打印 GPIO 配对表，一眼能核对接线

**为什么用 digitalRead**：`GPIO11/12/13` 属 ADC2，WiFi 开着时 ADC 读不到
（正式模式 6/7/8 路"降级"就是这个）。`digitalRead` 不受限，测试全程不用关 WiFi。

**接线**（GPIO 不是连续的 0..7）：

| 通道 | 开始位 | GPIO | 停止位 | GPIO |
|:---:|:---:|:---:|:---:|:---:|
| 1 | 微动 1 | GPIO1 | 微动 5 | GPIO10 |
| 2 | 微动 2 | GPIO2 | 微动 6 | GPIO11 |
| 3 | 微动 3 | GPIO9 | 微动 7 | GPIO12 |
| 4 | 微动 4 | GPIO8 | 微动 8 | GPIO13 |

切回正式模式：`SENSOR_TEMP_DIGITAL` 改回 0，重编译即可。

详见 [release/v1.1.14/RELEASE.md](release/v1.1.14/RELEASE.md)

---

## v1.1.13 —— 修「当前固件仅支持 AMS / AMS 2 Pro / AMS HT」弹窗

**2026-10-25**

**根因**：上报给打印机的 AMS 固件版本号是 Tianshu 编造的 `90.00.00.00`
（`00 00 00 5A`）。旧打印机固件不校验，新版会拿它跟白名单比对，对不上就把本机
判成"不认识的 AMS 设备"→ 弹窗 `[0500-40A5]`。

**修复**：
- 默认版本号改为 **01.00.06.87**（`0x01000657`，BMCU-X/BMCU-U 在用，A1 FW 01.08.01.00 验证过）
- 版本包改为运行时拼装，不再用写死数组
- 网页「和打印机的通信方式」新增「AMS 版本号」下拉框（01.00.06.87 / 00.00.06.49 / 90.00.00.00），
  存 NVS `s3ams-bus`，**立即生效**
- `/status` 新增 `ams_ver` / `ams_ver_raw`；新接口 `POST /ams_ver_set`

对照：`esp32s3-485` 用的是 `00.00.06.49`（真机抓包），`BMCU-U` 用 `01.00.06.87`，
Tianshu 与本工程 v1.1.12 及更早用的 `90.00.00.00` 是错的。

详见 [release/v1.1.13/RELEASE.md](release/v1.1.13/RELEASE.md)

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
