# v1.1.14 —— 临时直连微动模式（电阻编码微动未到货的过渡版）

## 这一版只改了「微动怎么读」，其他一行没动

电阻编码微动（4.7K / 10K 分压那套）还没到货，先把 8 只**普通微动**直接接到原来的
8 个传感器 IO 上，用 `digitalRead` 读，两两配对成 4 个通道的「开始位 / 停止位」。

- 上层 API 完全不变：`sensor_triggered(ch, SENSOR_START|SENSOR_STOP)` 一行没改
- AMCU / 缓冲补料 / 485 状态机 / 网页 / 计米 / 离合 —— **全部不动**
- 一个宏就能切回去：`SENSOR_TEMP_DIGITAL 0`

---

## 接线（关键：GPIO 不是连续的 0..7）

沿用正式模式的引脚表，所以编号是 `1,2,9,8,10,11,12,13`：

| 通道 | 开始送料微动 | GPIO | 停止送料微动 | GPIO |
|:---:|:---:|:---:|:---:|:---:|
| 1 | 微动 1 | **GPIO1** | 微动 5 | **GPIO10** |
| 2 | 微动 2 | **GPIO2** | 微动 6 | **GPIO11** |
| 3 | 微动 3 | **GPIO9** | 微动 7 | **GPIO12** |
| 4 | 微动 4 | **GPIO8** | 微动 8 | **GPIO13** |

接法（每只微动两线）：

```
微动 COM ── GPIO（上表）
微动 NO  ── GND
```

- 内部上拉已开（`pinMode(INPUT_PULLUP)`），**不用外接电阻**
- 极性：**压下 = 低电平 = 触发**。由 `SENSOR_TEMP_ACTIVE_LOW`（默认 1）控制
- 若你的接法是"微动接 3.3V + 引脚下拉"，把 `SENSOR_TEMP_ACTIVE_LOW` 改成 0

### 动作语义

| 微动 | 压下时固件做什么 |
|---|---|
| 微动 1~4（开始位） | 该通道吸合离合 + 电机进料（`AMCU_buffer_assist_feed`） |
| 微动 5~8（停止位） | 停电机 + 释放离合 |

中间态（两只都松开）= 保持上一次决策，超时 `BUFFER_FEED_TIMEOUT_MS` 后自动停。

---

## 为什么用 digitalRead 而不是 ADC

`GPIO11 / GPIO12 / GPIO13` 属于 **ADC2**，WiFi 开着时 ADC 读不到（正式模式下 6/7/8 路
"降级"就是这个原因）。`digitalRead` 不受 ADC2 限制 —— 所以 4 通道测试**全程开着 WiFi
也没问题**，不需要关 WiFi 去测。

---

## 代码改动

### `src/filament_sensor.h`

```cpp
#define SENSOR_TEMP_DIGITAL      1   /* 1=临时直连微动（当前）；0=正式电阻编码 */
#define SENSOR_TEMP_ACTIVE_LOW   1   /* 1=微动接 GND，压下为低电平 */
```

文件头注释里写清了完整接线表、极性说明、如何切回正式模式。

### `src/filament_sensor.cpp`

新增三个东西，都在 `#if SENSOR_TEMP_DIGITAL` 里，正式模式编译时全部不参与：

1. `s_dbg_counts[][] / s_dbg_last[][]` —— 独立去抖状态（`-1` = 还没采过）。
   不复用正式模式的 `s_counts[] / s_last_raw_combo[]`，因为那是三态 combo 的计数，
   混用会互相踩。

2. `sensor_dbg_apply(ch, kind, raw)` —— 单只微动的去抖 + 边沿计数，写回
   `s_triggered / s_trigger_seq / s_release_seq / s_last_*_ms`，跟正式模式写的字段完全一样。

3. `sensor_poll_digital()` —— 读 8 脚 → 按 `通道 c 开始=c，停止=c+4` 配对 → 喂给
   `sensor_dbg_apply`。后 4 个"通道"固定清成不触发（它们的 IO 已被当成停止位用掉了）。

`/status` 里 `s_mv[]` 填 `0mV`（压下）/ `3300mV`（松开），只是让网页「微动」那栏有数可看，
不是真实 ADC 读数。

### `src/filament_sensor.cpp` 初始化

```cpp
#if SENSOR_TEMP_DIGITAL
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        pinMode(s_pins[i], INPUT_PULLUP);
        s_adc_unit[i] = 1;          /* 标记可读，绕开降级判断 */
    }
    /* 后 4 个 IO 已被当成停止位用掉，关掉这 4 个"通道"的缓冲补料 */
    s_enabled_mask &= (uint8_t)((1u << (BOARD_CHANNEL_COUNT / 2)) - 1u);
#else
    sensor_adc_init();
#endif
```

`s_enabled_mask` 只留低 4 位 → `sensor_channel_enabled(4..7)` 返回 false →
`AMCU_buffer_assist_feed()` 对后 4 通道直接 return，不会去驱动不存在的料盘。

### `src/web_server.cpp`

- `APP_VERSION` → `1.1.14-s3ams`
- `/status` 的 `sensors` 段新增 `"mode":"digital"`（正式模式是 `"encoded"`）
- `/status` 的 `pins` 段在临时模式下按真实配对输出：通道 i 的
  `[停止, 开始]` = `[PINS[i+4], PINS[i]]`，通道 4~7 输出 `[-1,-1]`
  （正式模式仍是 `[PINS[i], PINS[i]]`）

### `web/index.html` + `web/app.js` + `src/web_files.h`

网页不写清楚接线，现场一定会按电阻编码那套去接，怎么接都不对。所以：

- 卡片标题在临时模式下变成「★ 临时直连：8 只独立微动两两配对（digitalRead，仅前 4 通道）」
- 顶部加一条黄色提示，写清配对规则、`COM→GPIO / NO→GND`、`压下=触发`、`不用外接电阻`
- 每路的「接线：…」按模式分三种文案，临时模式显示
  `接线：开始位 GPIO1 · 停止位 GPIO10 · 内部上拉，微动另一端接 GND，压下 = 触发`
- 通道 5~8 显示「本通道无独立微动 —— 它的 IO 已被当成通道 N 的停止位用掉」
- 底部补一张 4 行配对表（直接读固件下发的 `pins`，不写死）
- `web_files.h` 用 `tools/generate_web_files.py` 重新生成

---

## 启动日志（串口 / `/log` 应能看到）

```
W (xxx) filament_sensor: 微动: ★临时直连模式★ 8 只独立微动 → 4 通道（后 4 通道已禁用）
W (xxx) filament_sensor:   极性=低电平触发(微动接 GND)
W (xxx) filament_sensor:   通道1: 开始=微动1(GPIO1)  停止=微动5(GPIO10)
W (xxx) filament_sensor:   通道2: 开始=微动2(GPIO2)  停止=微动6(GPIO11)
W (xxx) filament_sensor:   通道3: 开始=微动3(GPIO9)  停止=微动7(GPIO12)
W (xxx) filament_sensor:   通道4: 开始=微动4(GPIO8)  停止=微动8(GPIO13)
W (xxx) filament_sensor:   电阻编码微动到货后把 SENSOR_TEMP_DIGITAL 改回 0 重新编译
```

**看到这几行才说明刷的是 v1.1.14。** 没看到就是没刷进去。

---

## 验证步骤（4 通道换色测试）

1. 网页 OTA 刷 `firmware.bin`，确认版本号变成 `1.1.14-s3ams`
2. 看串口日志有没有上面那 6 行 `★临时直连模式★`
3. 打开网页，手动压微动 1 → 「微动」栏通道 1 的「开始」应变成 `1`，松开变回 `0`
4. 微动 5 → 通道 1 的「停止」变 `1`
5. 通道 2/3/4 同样用微动 2/6、3/7、4/8 各试一次
6. 再跑 4 通道换色流程

### 排查对照

| 现象 | 原因 | 处理 |
|---|---|---|
| 压下微动网页没反应 | 接的不是表里那个 GPIO | 按上表核对，别按"IO0..IO7"猜 |
| 一直是 `1`，松开也不掉 | 极性反了 / 微动接的是常闭 | 改 `SENSOR_TEMP_ACTIVE_LOW 0`，或换 NO 触点 |
| 压微动 1 却动了通道 2 | 引脚接错位 | 看启动日志里的 GPIO 表逐根对 |
| 通道 5~8 网页一直 0 | 正常，后 4 通道已禁用 | 无需处理 |
| 压开始位电机不转 | PWM 低于 80 起不来（已知） | `MOTOR_SPEED_FULL` 已 ≥80，若改过要检查 |

---

## 怎么切回正式模式

电阻编码微动到货后：

```cpp
// src/filament_sensor.h
#define SENSOR_TEMP_DIGITAL   0
```

重新编译烧写即可。`sensor_adc_init()` 会自动接管，8 个通道全部恢复，
接线按 `docs/HARDWARE.md` 的 4.7K/10K 分压图接。

**不要**手动去注释 `sensor_poll_digital()` 之类的代码 —— 那个宏是唯一开关。

---

## 已知遗留（本版未动）

- 后 4 个通道（5~8）在本版**没有独立微动**，它们的 IO 被当成前 4 通道的停止位了。
  要测 5~8 通道必须等电阻编码微动到货，或者再买 4 只微动并改配对逻辑。
- `AMS_COUNT` 默认 4 但本板 8 通道只有 2 台 AMS（v1.1.13 遗留）
- 堵转阈值 `stall_ma = 0` 未标定（v1.1.12 遗留）
- 退料（尤其第二次退料）尚未单独验证
- `on_use` 计米 10 秒后停增、`on_use` "送料满 5 秒才切"延迟可能偏大
