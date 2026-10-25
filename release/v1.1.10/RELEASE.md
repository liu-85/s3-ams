# s3-ams v1.1.10 发布说明（第二次退料电机不动 + OTA 假失败）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO, env=s3ams）
**RAM** 23.4%（76804/327680）· **Flash** 39.7%（1197536/3014656）
**上一版**：v1.1.9（离合回读修好，进料/第一次退料已能正常驱动电机）

---

## 一、v1.1.9 实测结果：进料修好了 ✅

`tools/新建 文本文档.txt`（15:04:59~15:07:01，6460 帧）里的状态行：

```
[15:05:54] 电机=进料/100% 离合=0x01 电流=0mA 峰值=135mA | 料位0 动作=need_send_out 在线=1 打印=0 回读=1
[15:06:03] 电机=退料/100% 离合=0x01 电流=10mA 峰值=125mA | 料位0 动作=need_pull_back 在线=1 打印=0 回读=1
[15:06:20] 电机=进料/100% 离合=0x01 电流=0mA 峰值=100mA | 料位0 动作=need_send_out 在线=1 打印=0 回读=1
[15:06:32] 电机=停止/100% 离合=0x00 电流=0mA 峰值=0mA   | 料位0 动作=need_pull_back 在线=1 打印=0 回读=1   ← ❌
```

- `回读=1` → v1.1.9 的离合回读修复生效；
- 进料、**第一次退料**电机都真的转了（`离合=0x01`、`电机=进料/退料`、峰值电流 100~135mA）；
- **第二次退料**：动作已是 `need_pull_back`，但 `电机=停止 离合=0x00 电流=0mA` —— 完全没驱动。

485 报文侧两次循环的序列**完全一致**：

```
送料   03/read=00  →  09/A5(ready)  →  07/00(打印中)  →  09/3F(before_pull_back)  →  03/read=FF(触发退料)
循环1  t=128100..138778   退料正常 ✅
循环2  t=154460..168808   退料不动 ❌
```

→ **协议层没问题，是固件里的软件 latch 没复位。**

---

## 二、根因：`s_last_need_pull_back[]` 置位后永不清除

```cpp
// AMCU_run()
if (state == need_pull_back) {
    if (!s_last_need_pull_back[now_f]) {      // ← 第一次为真
        s_last_need_pull_back[now_f] = true;  // ← 置位
        is_need_pull_back = true;
        ...
    }
}
```

全项目搜 `s_last_need_pull_back` 只有三处：定义、这里置位、
以及 `last_action_set()` 里的 `if (m != need_pull_back) s_last_need_pull_back[f] = false;`
—— **而 `last_action_set()` 和 `last_action_need_pull_back_set()` 定义了却从未被任何地方调用**。

结果：

| 第几次退料 | `s_last_need_pull_back[0]` | `is_need_pull_back` | 电机 |
|---|---|---|---|
| 第 1 次 | false → 置 true | true | **反转退料** ✅ |
| 第 2 次 | 已是 true | 不再置位（仍为 false） | **不动** ❌ |

`AMCU_motion()` 里那段退料驱动是 `if (is_need_pull_back) { ... motorCmd(-1) ... }`，
`is_need_pull_back` 为 false → 整块被跳过 → 只剩 `case need_pull_back: RGB_GREEN; break;`
（只点灯不驱动）→ 正好是"状态是退料、电机却停着"。

### 对照 Tianshu 原版

`D:\DIY\S3-AMS\Tianshu\src\AMCU.cpp:403-421`：

```cpp
auto state = get_filament_motion(now_f);
if (state == need_pull_back) {
    if (last_action[now_f] != need_pull_back) {   // ← 用"上次动作"判断
        last_action[now_f] = need_pull_back;
        ...
    }
} else if (state == need_send_out) {
    last_action[now_f] = need_send_out;            // ← 状态变化时被改写
} else if (state == waiting) {
    last_action[now_f] = waiting;                  // ← 关键：这里会把标记冲掉
}
```

Tianshu 的 `last_action` 是个"当前动作"变量，**每次状态变化都会更新**，
所以下一轮再进 `need_pull_back` 时条件必然成立 —— 天然支持反复触发。
移植到 s3-ams 时改成了独立的布尔 latch，却丢掉了"离开状态时复位"这一步。

---

## 三、修复内容

### 1. `AMCU.cpp` —— 退料 latch 两处复位（双保险）

**(a) 离开退料状态就清**（`AMCU_run`）：

```cpp
} else {
    if (s_last_need_pull_back[now_f]) s_last_need_pull_back[now_f] = false;
    ...
}
```

**(b) 本轮退料结束（超时）也清**（`AMCU_motion`）：

```cpp
if (pull_back_channel >= 0 && pull_back_channel < MAX_FILAMENT_NUM) {
    s_last_need_pull_back[pull_back_channel] = false;
}
```

为什么两处都要：退料途中打印机可能已经切换 `now_f`，
那时 (a) 清的是**新料位**的 latch，原通道的会一直留 true → 该通道以后再也退不了料。

### 2. `AMCU.cpp` —— 退料途中不许被 `case waiting` 掐掉电机

对应 Tianshu 的 `if (!is_need_pull_back) switchInterrupt();`：

```cpp
case waiting:
    if (!is_need_pull_back) AMCU_drive_stop();
    RGB_BLUE;
    break;
```

原来 `case waiting` 无条件 `AMCU_drive_stop()`：退料时若打印机把 `now_f` 切到别的
通道，那个通道的 `act` 是 waiting → 一进 switch 就把正在反转的退料电机停掉，
表现为"退料只转了一下就停"。`default` 分支同样加了保护。

### 3. 退料链路补日志

```
退料开始: 料位0→通道1 PWM=100%
退料结束: 料位0 已反转 7 秒，停电机
```

下次现场不用再靠猜，网页「设备日志」直接看到退料起止。

### 4. `web/app.js` + `web_server.cpp` —— OTA "报失败但其实成功了"

**现象**：网页升级 1.1.9，提示上传失败；刷新后版本号已经是 1.1.9。

**原因**：设备收完最后一块 → `Update.end(true)` 成功 → `server.send(200,{ok:true})`
→ `delay(1000)` → `ESP.restart()`。**重启会把 TCP 连接直接掐断**，
弱信号（RSSI −73）下浏览器连响应头都没读完就收到 RST → `xhr.onerror`
→ 页面显示"上传中断/升级失败"，但固件其实已经写进 ota_1 了。

**修复（三层）**：

1. **固件**：`server.send()` 之后加 `server.client().flush()`（主动把 TCP 发送
   缓冲推出去），`delay` 1000→1500ms 再重启；
2. **前端**：新增 `otaWaitReboot()` —— 只要 `xhr.upload.onprogress` 已经到 100%
   （`uploadDone=true`），**无论 `onload` 拿到什么、还是 `onerror`，都不再直接报失败**，
   而是转成"等待重启并核对版本"；
3. **判据换成版本号**：上传前记下 `/status` 的 `app_version`，上传后每 1.5 秒轮询一次，
   - 版本变了 → **升级成功**（并自动刷新页面）
   - 设备掉线 → 显示"正在重启中…（已等 N 秒）"
   - 90 秒还没变化 → 提示手动刷新核对，明确告诉用户"若已是新版本说明只是回执丢了"

> 顺带修正：版本字段是 `app_version`（不是 `version`），前端已对齐。

---

## 四、刷机

**网页 OTA**（这次请留意：即使还提示失败，也请刷新看版本号 —— 那就是新逻辑要解决的误报）

浏览器打开 `http://192.168.2.100` → 升级 → 选 `release/v1.1.10/firmware.bin`

**串口（最稳，会顺带擦 otadata）**

```powershell
cd D:\DIY\S3-AMS\s3-ams
python flash_s3.py COM6
```

---

## 五、验证要点

1. `/status` → `app_version` = `1.1.10-s3ams`；
2. 连续做 **两次** 进料→退料，网页日志应出现两组：
   ```
   退料开始: 料位0→通道1 PWM=100%
   退料结束: 料位0 已反转 7 秒，停电机
   ```
3. 第二次退料时 `/status` 应为 `电机=退料/100% 离合=0x01 电流≠0`
   （不再是 `电机=停止 离合=0x00`）；
4. OTA 时即使网络抖一下，页面应最终显示「升级成功：1.1.10-s3ams → …」而不是失败。

---

## 六、仍待办

- **堵转阈值标定**：`stall_ma` 仍是 0（只显示不保护）。本次日志里
  进料峰值 100~135mA、退料峰值 125mA、空转瞬时 0~30mA，
  **数据偏噪（瞬时值大量读 0）**，需要你手动堵住料盘再跑一次，
  把「堵转时的稳定峰值」发我，我按 0.7 倍填进去。
- **BMCU-U 协议对齐**（AMS 版本号 `00.00.00.60` → `01.00.06.87`、
  注册 latch 加 2500ms 超时回退）：需打印机弹窗原文 + 机型/固件版本。
- **`statu_flags=0x0F` 未处理**：本次日志出现 33 帧 `0x0F/read=0xFF`
  （第 1 轮退料之后），当前固件对它什么都不做。暂未发现副作用，
  后续对照 BMCU 确认语义。
