# v1.1.23 —— 修流量校准阶段微动不送料

## 一句话

流量校准时打印机一直在抽料，但状态机 `motion` 可能停在 `idle`，
而缓冲微动的准入条件只认 `on_use/before_on_use/stop_on_use` → 微动压下
完全没反应。更糟的是即使放行，`AMCU_motion()` 的 `case waiting` 会在
同一个循环里把刚驱动的电机停掉。本版两处都修了。

---

## 现场症状

流量校准阶段，送料微动压下（WEB 微动页显示「开始位(空) 触发」），
但电机不转、不送料。v1.1.14（微动直连、不判状态机）测试正常。

---

## 根因（三处）

### ① 准入太严

`check_buffer_status()` 用 `is_channel_printing()` 做准入：

```cpp
if (!is_channel_printing(cur)) return;
```

它只认 `on_use` / `before_on_use` / `stop_on_use`，**不含 `send_out`、
更不含 `idle`**。流量校准阶段打印机的状态帧没匹配上任何一条，
`motion` 停在 `idle` → 整段 `return`，微动触发完全没有电机动作。

### ② 刚送起来就被停

`AMCU_run()` 的执行顺序：

```
check_buffer_status();   ← 吸合离合 + 电机转起来
AMCU_motion();           ← case waiting: AMCU_drive_stop() 立刻掐掉
```

`get_filament_motion()` 把 `idle` 映射成 `waiting`，`case waiting` 里
一句 `AMCU_drive_stop()` 就把电机停了。现场表现是"微动触发了，
电机不动或只抖一下"。

### ③ 判据本身还失效

`AMCU_buffer_assist_feed()` 里 `s_buffer_feed_decision = 1` 只在
`AMCU_drive_channel()` 返回 true 时置位。离合**已经吸合**时走 else 分支
（`motor_set_dir`），不置位 → 第 ② 条的让位判据拿不到信号。

---

## 改动

| 文件 | 改动 |
|---|---|
| `BambuBus.cpp` | `_filament_ext` 增加 `last_statu` / `last_statu_ms`，每次收到状态帧记录 |
| `BambuBus.cpp` | 新增 `is_channel_in_use()`（含 `send_out`） |
| `BambuBus.cpp` | 新增 `is_channel_active_recently(ch, 10s)`：看 statu 是否 0x03/0x07/0x09/0x0F |
| `BambuBus.cpp` | 新增 `get_channel_last_statu()`（诊断） |
| `AMCU.cpp` | `check_buffer_status()` 准入改为 `is_channel_in_use() \|\| is_channel_active_recently(cur, 10000)` + 限流日志 |
| `AMCU.cpp` | `case waiting`：`s_buffer_feed_decision[svc] == 1` 时不停电机 |
| `AMCU.cpp` | 兜底 `else` 分支：缓冲送料中只清跟随状态、不停电机 |
| `AMCU.cpp` | `AMCU_buffer_assist_feed()`：`start_trig` 进来就置 decision=1 |
| `web_server.cpp` | `/status` 的 `feed` 段增加 `statu` / `active` |

**「其他通道不读」的限制不变** —— 仍然只认 `cur` 一路。

---

## 如果还是不动，怎么查

1. **WEB 右侧日志**：看有没有
   ```
   缓冲被挡：cur=N 微动(开始=1 停止=0) 该通道不在使用态（... last_statu=0xXX）
   ```
   有这条 = 打印机确实没把这一路当"在用"。

2. **`/status` 的 `feed.statu`**：
   - `0x00` / `0x01` → 打印机没把这一路当在用
   - `0x03` / `0x07` / `0x09` / `0x0F` → 在用（应能送料）
   - `feed.active` = 1 表示最近 10 秒判为在用

3. 如果 `statu` 一直是 `0x00/0x01` 而打印机确实在抽料，
   那说明流量校准走的是另一条我们没识别的状态帧，
   需要抓一次 485 日志（`tools/mon485.ps1`）看实际字节。

---

## 烧写

```bash
python flash_s3.py COM6
```

或网页 OTA 上传 `firmware.bin`。

---

## 验证要点

1. 流量校准阶段：微动压下 → 电机转、送料；松开/到停止位 → 停
2. 正常打印：微动闭环照旧（到开始位补料、到停止位停）
3. 空闲时其他通道的微动不触发任何动作
4. WEB 硬件调试的点动 / 进料 / 退料不受影响
