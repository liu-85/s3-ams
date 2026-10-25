# v1.1.12 —— 挤出失败 / 退料偶发不动：三个真实根因

v1.1.11 修了计米（确实是 bug），但不是本次现象的主因。重新抓包逐帧对时序后，找到三个
互相叠加的问题，其中第 1 个同时解释了"挤出失败"和"退料有时不动作"。

---

## 根因 1：`0x0F` 被当成"清空全部通道"，状态被反复抹掉

### 证据

```
[15:06:10] RX type=04 ams=00 statu=0F motion=00 read=FF   ← 打印机"使用中"的查询
[15:06:10] TX len=60        use=0x00 pressure=0xFFFF 料位=FF   ← 我方应答：状态全没了
```

同一时刻内部状态被整段清空，随后打印机立刻发 `0x0F/FF`。

### 代码

`send_for_Dxx()` 末尾：

```cpp
else if (read_num == 0xFF) {
    for (uint8_t i = 0; i < 4; i++) set_motion_internal(AMS_num, i, idle);
    data_save.BambuBus_now_filament_num = 0xFF;      // ← 无条件执行
}
```

打印机在"使用中"会周期性发 `statu=0x0F + read=0xFF`（本日志 33 帧），
每次都触发这段清空。于是：

**① 电机反复启停 → 耗材断续 → 挤出机拉一点就拉不动**

**② `now_filament_num` 变成 0xFF 后，连锁把退料也堵死**

```cpp
static uint8_t get_loaded_filament(uint8_t ams_num) {
    int now = data_save.BambuBus_now_filament_num;
    if (now >= ams_num*4 && now < (ams_num+1)*4) return now % 4;
    return 0xFF;                       // ← now=0xFF 时恒返回 0xFF
}
bool allow_stop = (loaded == filament_channel);   // ← 恒 false
```

`allow_stop=false` → 打印机发 `09/3F`（before_pull_back）和 `03/00/FF`（pull_back）
全部被拒 → **退料电机不动**。这就是"有的时候退料没动作"。

### 修复

```cpp
else if (read_num == 0xFF) {
    if (statu_flags == 0x00) {      // 只有明确的"复位/取消"才清空
        ... 清空 ...
    }
    // 其他未定义状态（0x0F 等）一律忽略，保留当前状态
}
```

---

## 根因 2：`0x0F` 是"使用中"，我们只认 `0x07`

整份日志里 `0x07/0x7F`（原以为的 on_use）**一帧都没有**，而：

| 帧 | 次数 | 出现时机 |
|---|---|---|
| `0x0F/0xFF/read=00` | 123 | 送料完成之后、下次送料之前 |
| `0x0F/0x00/read=FF` | 33 | 同上 |
| `0x07/0x00/read=00` | 3 | 极少 |

`0x0F = 0x01|0x02|0x04|0x08`，比 `0x07` 多一个 bit3 —— 就是"正在使用"。

只认 `0x07` 的后果：`on_use` / `stop_on_use` 永远匹配不上，状态卡在 `before_on_use`，
打印机认为 AMS 没进入使用态。

### 修复

```cpp
bool is_stop_on_use = ((statu == 0x07) || (statu == 0x0F)) && (motion == 0x00);
bool is_on_use      = ((statu == 0x07) || (statu == 0x0F)) &&
                      ((motion == 0x7F) || (motion == 0xFF));
```

同时给"停止广播"（`statu=0x01`，本日志 1259 帧）加保护：**打印中的通道不许被清成 idle**。

---

## 根因 3：`read=0xFF` 时固定回 `pressure=0xFFFF`（无耗材）

```cpp
uint16_t pressure = 0xFFFF;
if (read_num < 4) { pressure = get_pressure_value(...); }   // read=0xFF 时不进
```

打印机在 `0x0F`（使用中）阶段周期性查询当前料位，我们回"无耗材"，
打印机会认为 AMS 掉线/空料。

修复：`read=0xFF` 且 `statu` 是 `0x03/0x0F` 时，用**当前料位**的状态回答；
只有 `statu=0x01`（全部停止广播）才回 `0xFFFF`。

---

## 根因 4（你要求的"跟随送料"）：`before_on_use` 时电机停死

### 为什么"断开离合"不够

日志证明 `09/A5` 期间本来就是 `离合=0x00`、`电机=停止`、`电流=0mA`，耗材照样拉不动。
因为**离合只断开"电机↔主动轮"，压轮始终压着耗材**，阻力来自齿轮组反向拖动。

Bambu 官方 wiki 对缓冲器的说明：

> The Filament Buffer ... has a slide, a spring, and a hall sensor.
> **By controlling the feed speed of the AMS, we can make sure the tension on the filament
> is within a given range so that the toolhead extruder can work easily and precisely,
> without adding too much or too little tension.**

→ 打印时 AMS **必须主动控制送料速度**，不是停着不动。

### 修复：`before_on_use` 跟随送料

```cpp
#define FOLLOW_FEED_ON_MS   250    /* 送料脉宽 */
#define FOLLOW_FEED_OFF_MS  350    /* 间隔 */

// 100% PWM ≈ 50mm/s → 平均 50 × 250/600 ≈ 21mm/s
// 略高于挤出机拉料速度（10~20mm/s），保证耗材不绷紧；
// 停的 350ms 让缓冲器把多送的料吸收掉，不会顶进喷嘴。
```

为什么不直接用缓冲微动驱动：微动是 ADC 三态分压，现场信号不稳（悬空 371mV 就落在
START 区），而 `before_on_use` 只持续几秒，等微动触发太慢。这里直接按固定节奏送。

`on_use`（正常打印）仍交给缓冲微动按需补料。

---

## 其它加固

1. **`check_buffer_status()` 退料期间整段跳过**
   缓冲器满时 `AMCU_buffer_assist_feed()` 会调 `AMCU_drive_stop()`，
   而它内部的 `clutch_request_release_all()` 会**把正在挂起的退料吸合请求覆盖掉**
   （`s_pending_clutch` 只有一个槽）→ 退料请求永远等不到吸合 → 电机不转。

2. **`before_on_use` 通道从缓冲逻辑里让出**
   否则跟随泵和微动逻辑两边交替驱动同一台电机，现场就是"抖一下、抖一下"。

3. **跟随泵退出时若停在"送"相位会主动停电机**
   否则交回 `on_use` 时微动若处于 MID，电机会一直转着把料顶进喷嘴。

---

## 与 Tianshu 的差异（本次又查了一遍）

| 项 | Tianshu | 本工程 |
|---|---|---|
| 电机结构 | 4 个独立电机（BMCU） | 单电机 + 8 路电磁离合 |
| 计米 | `AS5600_get_distance_E()`，**实为随机数模拟** | 时间估算（v1.1.11 已修单位） |
| 微动 | 1 个开关，变化时送 200ms 停 | 每路 2 个（开始位/停止位），ADC 三态 |
| `0x0F` | 同样没处理 | **v1.1.12 已识别为 on_use** |
| `read=0xFF` 清空 | 同样无条件清空 | **v1.1.12 已加状态判断** |
| 打印中张力控制 | 无（4 电机各自独立，无离合阻力） | 需要（单电机+离合，压轮常压） |

**关键结论**：Tianshu 是 4 电机方案，打印时电机不通电就等于耗材完全自由，
所以它不需要"跟随送料"；我们单电机 + 离合的机械结构，停料就等于耗材被齿轮组拽住，
必须主动跟随。这是硬件差异，不是移植遗漏。

---

## 验证方法

刷完（网页 OTA，`release/v1.1.12/firmware.bin`），用 `tools/mon485.ps1` 看状态行：

```
状态: 电机=进料/100% 离合=0x01 电流=5mA 峰值=65mA act485=1 | 料位0 动作=need_send_out
      在线=1 打印=0 堵转=0 点动=0 回读=1 cmd=0 计米=0.152m 阶段=none 跟随=0 微动=S00/1200mV
```

关键看三段：

1. **`计米`**：送料 1 秒约 +0.05 米。若涨几十米说明单位又跑飞
2. **`阶段`**：送料完成应依次出现 `before_on_use` → `on_use`（不再是 `none` 卡住）
3. **`跟随`**：`阶段=before_on_use` 期间应在 1/0 之间跳动（250ms 送、350ms 停）
4. **`微动`**：`S<停止位><开始位>`，`S00` = 缓冲器在中间档（正常待机）

如果 `阶段` 一直是 `none` 且 `动作=need_send_out` 不变，说明打印机还在重试送料，
把这段状态行发我。

---

## 已知遗留

- `on_use` 分支的"送料满 5 秒才切 on_use"延迟保留未改（Tianshu 原逻辑）。
  若发现耗材顶进喷嘴，把这个 5000ms 改小。
- `on_use`（打印中）计米仍是 `time_used_ms/300000` 且累计 10 秒后停止增长。
- 堵转阈值 `stall_ma` 仍为 0（只显示不保护）。
