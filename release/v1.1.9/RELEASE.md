# s3-ams v1.1.9 发布说明（送料电机不转的根因：离合状态回读恒为 0）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**RAM** 23.4%（76812/327680）· **Flash** 39.6%（1192756/3014656）
**上一版**：v1.1.8（OTA / 日志 / 栈都已正常，问题只剩"送料电机不转"）

---

## 一、现场症状（用户实测）

> 刚才测试进料，该电机送料时电机没转……手动送入挤出机，挤出机拉进一点
> 耗材后将耗材退出，退出后再送入，还是拉动一点就退料。

配套 485 原始报文日志（`tools/新建 文本文档.txt`，7330 行）解析结论：

| 项目 | 数据 |
|---|---|
| 打印机发出的送料命令 | `3D C5 0D F1 04 00 03 00 03 00 00` → **378 帧**，`ams=0 statu=0x03 motion=0x00 read=0x00` |
| 固件解析结果 | `statu_flags==0x03 && motion_flag==0x00` → `is_send_out` 成立 → `motion_set = need_send_out` ✅ |
| 旁证 | 我们的应答里 pressure = **0x4700**（`get_pressure_value()` 对 `send_out` 的固定返回值），证明内部状态确实切到送料 ✅ |
| 电机状态 | 全部 47 条状态行都是 **`电机=停止 离合=0 电流=0mA`** ❌ |

**结论：485 协议层完全正常，状态机也正确切到了送料；卡在"驱动电机"这一环。**

---

## 二、根因：`GPIO_MODE_OUTPUT` 关掉了输入缓冲，`gpio_get_level()` 恒返回 0

`clutch.cpp` 原来这样配置 8 路离合引脚：

```cpp
gpio_config_t cfg = { ..., .mode = GPIO_MODE_OUTPUT, ... };
```

ESP-IDF 的 `GPIO_MODE_OUTPUT` 只打开输出驱动，会把该脚的**输入缓冲关掉**
（IO_MUX 的 `FUN_IE = 0`）。此时 `gpio_get_level()` 读回来的**永远是 0**。
（要能回读必须用 `GPIO_MODE_INPUT_OUTPUT`。）

于是整条链这样断掉：

```
clutch_engaged_mask()  → 恒 0（8 路全读成"未吸合"）
      ↓
clutch_is_engaged(ch)  → 恒 false
      ↓
AMCU_drive_channel()   → 永远走"未吸合 → 异步发起吸合"分支，return false
      ↓
motor_set_dir_speed()  → 一次都没被调用
      ↓
电机永远不转；/status 里 hardware.engaged=0、motor_direction=0(停止)
```

而离合其实**被真实驱动了**（后台任务照常写引脚、你也听得到"咔"的一声），
只是固件读不回来 —— 所以表现为"离合响了但电机一点不动、电流 0mA"。

> 顺带解释了两件事：
> 1. `clutch_assert_single()`（运行期体检）永远读到 mask=0，形同虚设；
> 2. `clutch_init()` 里"上电回读自检"永远不报错。

---

## 三、修复内容

### 1. `clutch.cpp` —— 离合引脚改 `GPIO_MODE_INPUT_OUTPUT` + 回读自检 + 软件兜底

- 引脚配置改 `GPIO_MODE_INPUT_OUTPUT`（输出 + 输入缓冲都开），`gpio_get_level()` 可正常回读；
- `clutch_init()` 增加**回读能力自检**：写一次"吸合"电平（仅 1ms，不会真吸合）
  立刻读回，失败则把 `s_readback_ok` 置 false，并打印
  `★ 离合引脚回读不可信……改用软件记账判断吸合状态`；
- `clutch_engaged_mask()` 在回读不可信时退回**软件记账** `s_active`，保证送料链不会因
  mask=0 而永远不启动电机；
- `clutch_engage()` 改为"先记账再等机械稳定"，回读兜底模式下 80ms 等待期间状态也正确；
- 新增 `clutch_readback_ok()` 供 `/status` 展示。

启动日志现在会明确写出回读是否正常：
```
8 路电磁离合已初始化（高电平吸合，吸合等待80ms，回读正常）
```

### 2. `AMCU.cpp` —— 离合请求不再每循环重复入队（同时修 Web 卡顿）

`AMCU_drive_channel()` 在 `need_send_out` 分支里是**每循环**调用的（主循环几百 Hz），
而它每次都会 `clutch_request_engage()`；`AMCU_drive_stop()` 同样每次
`clutch_request_release_all()`。后果：

- 离合环形队列（8 深）瞬间塞满 → `clutch_queue_push()` 走兜底分支，
  **退化成在主循环里同步执行 `clutch_engage()`（内含 80ms + 60ms `vTaskDelay`）**
  → 主循环被卡、网页变慢、485 收包延迟；
- `req_slot` 只有 4 个槽，`epoch` 每请求自增 → 句柄被反复回收，
  `clutch_result_poll()` 永远等不到完成。

修复：

- 新增 `s_pending_kind` / `s_pending_ch` 记账；**同一通道已有挂起请求时不重复入队**；
- `AMCU_drive_stop()` 改为幂等：已经全断且无挂起请求时直接返回，不再入队；
- `AMCU_clutch_pump()` 按请求类型分别结算，吸合失败（后台完成但电平不对）时放弃本笔，
  下一循环重新发起，不再死等。

### 3. `BambuBus.cpp` —— `0xFF % 4 == 3` 的通道误判

`BambuBus_now_filament_num` 的初值/全复位值是 `0xFF`，`0xFF % 4 == 3`：

```cpp
else if (read_num == 0xFF && statu_flags == 0x01) {
    uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;  // 0xFF → 3 ！
    if (now_ch < 4) { set_motion_internal(AMS_num, now_ch, idle); }  // 恒真
}
```

结果：打印机发"停止"广播时，**清掉的是通道 3，真正的送料通道停不下来**。
已在 `statu_flags==0x03` 与 `statu_flags==0x01` 两个分支都加上 `< 16` 前置判断。

### 4. `AMCU.cpp` —— `s_last_action` 抢先赋值导致"送料基准"永不记录

`AMCU_run()` 里 `s_last_action[now_f] = need_send_out` 先执行，
使 `AMCU_motion()` 的 `case need_send_out` 里
`if (s_last_action[cur] != need_send_out)` 永远为假 →
`last_pullback_meters[cur]` 一直是 0。一旦 `target_meters > 0`，
就会立刻判"已到位"→ 送一点就停。已移除该提前赋值。

### 5. 可观测性（这次不用再猜）

**`/status` 新增 `feed` 段**：

```json
"feed": {
  "cur": 0,                    // 当前料位（ams*4+ch）
  "motion": 1, "motion_text": "need_send_out",
  "online": 1,                 // 0 → AMCU 会 drive_stop，不驱动
  "printing": 0,               // 1 → 判成打印中，AMCU_motion 直接让出
  "tripped": 0,                // 1 → 堵转 latch，禁止驱动（/current_clear 清）
  "jog": 0,                    // 1 → 点动未结束，状态机让出
  "clutch_mask": 2,            // 实际吸合掩码（bit0=通道1）
  "clutch_rb": 1,              // 1=引脚回读可信，0=用软件记账兜底
  "motor_dir": 1, "motor_pct": 100,
  "cmd": 1                     // 最近一次 485 包类型
}
```

**送料分支增加一次性日志**（进网页「设备日志」看，不刷屏）：

```
送料开始: 料位0→通道1 PWM=100% 电机已启动
送料开始: 料位0→通道1 PWM=100% 电机等离合吸合后启动
送料被挡: 料位0 被判为离线(online=0)，不驱动电机
```

**`tools/mon485.ps1` 状态行同步升级**（对 v1.1.8 及更早固件自动降级为旧格式）：

```
状态: 电机=进料/100% 离合=0x02 电流=0mA 峰值=0mA act485=1 | 料位0 动作=need_send_out 在线=1 打印=0 堵转=0 点动=0 回读=1 cmd=1
```

---

## 四、刷机

**串口（推荐，会顺带擦 otadata）**

```powershell
cd D:\DIY\S3-AMS\s3-ams
python flash_s3.py COM6
```

**网页 OTA**：浏览器打开 `http://192.168.2.100` → 升级 → 选
`release/v1.1.9/firmware.bin`。

> 刷完在 `/status` 里确认 `version` 为 `1.1.9-s3ams`，
> 并确认启动日志出现「回读正常」四个字。

---

## 五、验证要点

1. `/status` → `feed.clutch_rb` 应为 **1**（启动日志「回读正常」）；
2. 打印机下发进料 → 网页日志出现「送料开始: 料位0→通道1 PWM=100% 电机已启动」；
3. 同时 `/status` → `feed.motor_dir=1`、`feed.motor_pct=100`、`feed.clutch_mask=0x02`；
4. 电流应从 0mA 涨起来（空转几十 mA，堵转几百 mA）—— **这一步的读数就是堵转阈值标定依据**；
5. 若日志显示「送料被挡: …… 离线」，说明是微动/在线判定问题，按 `feed.online` 继续查。

---

## 六、仍待办

- **堵转阈值标定**：`stall_ma` 目前仍是 0（只显示不保护）。请把上面第 4 步
  实测的「空转电流 / 堵转电流」发我，我按 `堵转电流 × 0.7` 填进去。
- **BMCU-U 协议对齐**（AMS 版本号 4 字节 `00.00.00.60` → `01.00.06.87`、
  注册 latch 加 2500ms 超时回退）：需要你提供打印机弹窗的具体文字 + 机型/固件版本再动。
