# v1.1.16 —— WEB 点动进料不吸合 + 微动只认当前使用通道

## 现象（用户实测 v1.1.15）

在 **WEB 硬件调试**里用「进料 / 退料」按键逐个通道测试：

- **进料：当前通道的电磁离合不吸合**（料不动）
- **退料：正常转动退料**

用户同时明确了微动的生效范围：

> 微动是只在打印时当前使用通道缓冲送料时启用，
> 其他通道不管什么状态都不做动作或者说状态不读取。

---

## 根因一：点动期间 `check_buffer_status()` 照样在跑

`AMCU_run()` 的调用顺序是：

```cpp
void AMCU_run() {
    ...
    check_buffer_status();   // ← 先跑，没有任何 jog 保护
    AMCU_motion();           // ← jog 的保护只写在这里面
}
```

而 jog 的让出保护只写在 `AMCU_motion()` 里：

```cpp
void AMCU_motion() {
    AMCU_clutch_pump();
    if (s_jog_active) { amcu_jog_pump(); return; }   // ← 只有这里有
```

于是**点动期间缓冲补料逻辑照常运行**，`AMCU_buffer_assist_feed()` 会：

- 停止位微动触发 → `AMCU_drive_stop()` → 把点动刚请求的离合**断开**
- 开始位微动触发 → `AMCU_drive_channel(别的通道)` → **抢离合槽位**

`s_pending_clutch` 只有一个槽，被反复改写 → 离合吸不上 / 咔咔响 / 电机不转。

### 连带 bug：`AMCU_drive_stop()` 清掉挂起的方向

```cpp
void AMCU_drive_stop() {
    ...
    s_pending_dir = MOTOR_DIR_STOP;   // ← 原来无条件清
}
```

`s_pending_dir` 是「这次吸合完成后要跑的方向」，由 `AMCU_clutch_pump()` 吸合成功后读取。
一旦 `drive_stop()` 夹在「发起吸合」和「吸合完成」之间，pump 就按 `STOP` 启动 →
**离合吸合了但电机不转**。

---

## 根因二：微动遍历了全部 8 个通道

```cpp
// 原实现
for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
    if (is_channel_printing(i)) AMCU_buffer_assist_feed(i);   // ← 每个 printing 通道都动
}
```

本硬件是「1 电机 + 8 离合」，同一时刻只能服务 1 个通道。遍历全部通道意味着：

- 多个通道同时 printing 时互相抢离合
- 打印中在 WEB 点动**别的**通道时，旧通道的微动仍在动作，把点动的离合覆盖掉

---

## 修复

### 1. 微动只认「当前使用通道」

```cpp
int cur = get_now_filament_num();
if (cur < 0 || cur >= BOARD_CHANNEL_COUNT) return;   /* 本板只有 8 个物理通道 */
if (!is_channel_printing(cur)) return;               /* 必须正在打印 */
if (!sensor_channel_enabled(cur)) return;            /* 该通道必须开了缓冲送料 */
if (is_channel_before_on_use(cur)) return;           /* 交给 follow_feed_pump */
AMCU_buffer_assist_feed(cur);
```

四个条件全满足才读微动、才动作。**其他通道的微动状态既不读也不驱动任何动作。**

### 2. 点动期间缓冲补料完全让出

```cpp
if (is_need_pull_back) return;          /* v1.1.12：退料优先 */
if (amcu_jog_busy()) return;            /* v1.1.16：点动优先 */
if (find_motion_channel() >= 0) return; /* v1.1.15：485 进/退料优先 */
```

### 3. `AMCU_drive_stop()` 不再清挂起的方向

```cpp
if (s_pending_kind != 1) s_pending_dir = MOTOR_DIR_STOP;
```

有挂起的吸合请求时保留方向，避免「吸合了但按 STOP 启动」。

---

## 新增诊断：点动目标

`/status` 的 `feed` 段新增三个字段：

| 字段 | 含义 |
|---|---|
| `jog_ch` | 点动目标通道（0 = 没在点动） |
| `jog_dir` | 点动方向（1 = 进料，-1 = 退料） |
| `jog_ms_left` | 点动剩余毫秒 |

**核对方法**：WEB 点「通道 3 进料」→ `jog_ch` 应为 `3`、`jog_dir` 为 `1`；
此时 `clutch_mask` 应为 `0b100`（bit2 = 通道 3）。
如果 `jog_ch` 是 3 而 `clutch_mask` 是别的位，就是离合被别处抢走了。

---

## 验证步骤

1. 刷 `firmware.bin`，确认版本号 `1.1.16-s3ams`
2. **不打印**，WEB 硬件调试逐个通道点「进料」：
   - 应听到该通道离合「嗒」一声吸合，电机转动送料
   - 再点「退料」：断开 → 反方向吸合 → 退料
3. 打印中再点一次**别的**通道进料：现在点动优先，微动不再干扰
4. 若仍不吸合，抓 `/status` 看 `jog_ch` / `jog_dir` / `clutch_mask` /
   `motor_dir` / `motor_pct` 五个字段，把截图发我

### 排查对照

| 现象 | 看哪里 |
|---|---|
| `jog_ch=0` 但按钮返回 ok | 点动已结束（`jog_ms_left` ≤ 0），时长设太短 |
| `clutch_mask=0` 且 `jog_ms_left>0` | 离合没吸上 → 看串口有无「切换离合超时」 |
| `motor_pct<80` | 电机启动阈值不足（实测低于 80 起不来） |
| `clutch_rb=0` | 引脚回读不可信，用软件记账兜底 |

---

## 已知遗留

- 单槽仲裁仍是"逻辑上避免并发"，不是优先级队列。新增任何会驱动电机的模块，
  都要在开头加 `if (amcu_jog_busy()) return;` 之类的让出判断。
- 微动的**硬件轮询**（`sensor_poll_digital`，5ms 一次）仍在读 8 个脚，
  但读到的状态只对「当前使用通道」生效，其他通道不驱动任何动作。
  网页仍会显示全部 8 路状态（显示 ≠ 动作）。
- v1.1.15 遗留项不变（后 4 通道无独立微动、退料二次验证、堵转阈值待标定）
