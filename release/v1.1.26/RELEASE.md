# v1.1.26 —— 修「停印后电机一直转」+ 电流剖析 + 送料 PWM 可调

> 版本：`1.1.26-s3ams`
> 日期：2026-10-31
> 主题：**修掉 v1.1.23 埋下的电机无限转 bug**，并给出标定堵转阈值的手段

---

## 一、修了什么：停止打印后电机一直转

### 现场现象

> "我都停止打印了，电机还是一直在转"

### 根因（v1.1.23 埋的）

v1.1.23 为了让"微动压下了电机不被立刻停掉"，让 `AMCU_motion()` 在
`s_buffer_feed_decision[ch] == 1` 时**不再** `AMCU_drive_stop()`：

```cpp
// v1.1.23 的让位判断
const bool buffer_feeding = (s_buffer_feed_decision[svc] == 1);
if (!is_need_pull_back && !buffer_feeding) AMCU_drive_stop();
```

但 `decision` **只在 `AMCU_buffer_assist_feed()` 内部被清零**。一旦准入失败
（打印机不发了 / 该通道不在用 / `cur` 切走了），那个函数就再也不会被调用 →
`decision` 永远停在 1 → `AMCU_motion()` 永远不停电机 → **电机无限转**。

一句话：**需要清理的时候，恰恰是清理函数不会被调用的时候。**

### 修法（三层）

**① 判据收紧** —— 新增 `buffer_feed_holds_motor(ch)`，"让位"不再只看
`decision` 的值，而是看它是否**还新鲜、还该占着**：

| 条件 | 含义 |
|---|---|
| `decision == 1` | 微动要求补料 |
| `s_buffer_feed_start != 0` | 有起点，不是脏值 |
| 连续运行 ≤ 30 秒 | 没超过硬上限 |
| 15 秒内有过反馈 | 不是陈旧状态 |
| 该通道此刻仍"在用" | `is_channel_in_use() \|\| is_channel_active_recently(5s)` |

五条全满足才允许它压着电机不让停。

**② 独立看门狗** —— 新增 `buffer_feed_watchdog()`，放在 `AMCU_run()` **最前面**，
不受任何 `early-return` 影响。它做两件事：

- 把不该再占电机的 `decision` 清掉
- 如果电机**确实**被这一路占着（`ch == cur && s_current_channel == ch+1`），主动停机

```
缓冲送料看门狗：通道1 已不在使用态，停止电机（避免停印后空转）
```

**③ 窗口统一** —— 新增 `BUFFER_IN_USE_WINDOW_MS = 5000`，`check_buffer_status()`
的准入和 `buffer_feed_holds_motor()` 的让位**必须用同一个值**，否则会出现
"准入还开着、让位已经关了"的撕裂（电机刚驱动就被停，只抖一下）。

**效果**：停止打印后，打印机不再报"在用"→ 最多 5 秒电机自动停。

---

## 二、移除微动日志

删掉了 `check_buffer_status()` 里那条每 3 秒一条的：

```
缓冲被挡：cur=0 微动(开始=1 停止=0) 该通道不在使用态（... last_statu=0x00）
```

现场反馈太吵。要排查微动仍可用：
- `http://192.168.2.100/sensor_raw` —— 8 只微动的原始电平
- `/status` 的 `feed.statu` / `feed.active`

启动时打印的那几条（引脚表、极性）是**一次性**的，保留。

---

## 三、新增：电机电流剖析（启动值 / 正常值 / 堵转值）

### 日志

每次电机**连续运行 ≥ 0.5 秒**，停止时打一条：

```
电流统计: 进料 PWM=100% 时长=3200ms 启动峰值=1980mA 稳态均值=420mA 稳态峰值=505mA 全程最大=1980mA
```

跟随送料的 250ms 脉冲**不计**（否则日志被刷爆）。

### 三个数怎么用

| 字段 | 含义 | 用途 |
|---|---|---|
| **启动峰值** | 起动瞬间的电流（正常是稳态的 3~8 倍） | 定「起步屏蔽」`blank_ms` 够不够。启动峰值高但屏蔽只有 300ms → 必然误跳 |
| **稳态均值** | 正常送料时的典型电流 | 这就是"正常值"。换 PWM 档位后对比，掉太多说明带不动 |
| **稳态峰值** | 正常波动上限 | **堵转阈值取它的 2 倍左右**，且明显低于启动峰值 |

### 关键技术点：峰值用原始值，均值用滤波值

`current_sense_ma()` 是 IIR 滤波后的（`shift=3`，10ms 采样 → 时间常数约 80ms），
会把只持续 50~200ms 的启动尖峰**削掉一大截** → 按它定阈值会定得太低 → 误跳闸。

所以新增 `current_sense_raw_ma()`（未滤波原始值）：

- `start_peak_ma` / `max_ma` → 用**原始值**（真实峰值）
- `run_avg_ma` / `run_peak_ma` → 用**滤波值**（不被噪声带偏）

### 网页显示

「电机电流采样」卡片下面新增一行：

```
最近一次运行  进料 · PWM 100% · 3.2 秒
启动峰值 1.98 A　稳态均值 420 mA　稳态峰值 505 mA
```

`/status` 里对应 `current.last`：
```json
"last": {"valid":true,"dir":1,"pct":100,"ms":3200,
         "start_peak":1980,"run_avg":420,"run_peak":505,"max":1980}
```

---

## 四、新增：自动送料 PWM 可在网页调整

### 位置

「参数设置」页 → **自动送料 PWM** 卡片（在「退料时间」下面）。

### 接口

```http
POST /feed_pwm_set   {"pct": 90}
→ {"ok":true,"pct":90,"info":"自动送料 PWM 已保存"}
```

范围 **40~100**，默认 **100**，立即生效并写入 NVS（`s3ams-feed`），重启不丢。

### 为什么范围下限是 40 而不是 1

本硬件**实测电机 PWM 低于 80 起不来**（项目铁律）。40 只是留给轻载/空转的余量，
实际请用 **80~100**。填 40~79 时网页会提示"低于 80% 电机可能起不来"。

### 怎么定这个值

1. 保持 100，点动送料一次，看日志的「稳态均值」
2. 改成 90 再送一次，对比稳态均值
3. 稳态均值明显下降但仍能稳定送料 → 可以停在这档
4. 送料跟不上打印机拉料、微动频繁触发 → 回到 100

### 为什么单独存 NVS

`SettingsData` 带 checksum，往里加字段会让**老配置整包失效**（用户设过的
退料时间等会被重置）。所以用独立命名空间 `s3ams-feed`，与
`s3ams-access` / `s3ams-mqtt` 一致。

---

## 五、改动文件

| 文件 | 改动 |
|---|---|
| `src/AMCU.cpp` | `buffer_feed_holds_motor()` / `buffer_feed_watchdog()` / `BUFFER_IN_USE_WINDOW_MS`；删除微动日志；送料用 `settings_get_feed_pwm()` |
| `src/motor.h` / `src/motor.cpp` | `motor_current_stat_t` + 20ms 采样剖析；`motor_get_current_stat()` / `motor_reset_current_stat()` |
| `src/current_sense.h` / `.cpp` | 新增 `current_sense_raw_mv()` / `current_sense_raw_ma()` |
| `src/settings_storage.h` / `.cpp` | `FEED_PWM` + `settings_set_feed_pwm()` / `settings_get_feed_pwm()`（NVS `s3ams-feed`） |
| `src/web_server.cpp` | `POST /feed_pwm_set`；`/status` 输出 `feed_pwm` 与 `current.last` |
| `web/index.html` / `web/app.js` / `src/web_files.h` | 「自动送料 PWM」卡片；「最近一次运行」电流显示 |

---

## 六、升级

```bash
python flash_s3.py COM6
```

或网页 OTA 上传 `firmware.bin`。

### 刷完建议做的三件事

1. **验证电机不再空转**：打印一小段 → 停止打印 → 电机应在 **5 秒内**停下
2. **标定电流**：点动送料 3 秒（日志出现「电流统计」），记下稳态均值；
   再手动捏住耗材堵转 3 秒，记下堵转时的稳态峰值 → 阈值取两者中间
3. **试送料 PWM**：在「参数设置」改 90 再试一次，看稳态均值和送料是否更稳
