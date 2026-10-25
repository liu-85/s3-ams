# v1.1.15 —— 修打印中换料：离合咔咔响、料退不出也送不进

## 现象（用户实测）

- 网页手动把料送入料管：**正常**
- **打印中换料**：电磁离合"咔咔响"，料**既退不出来也送不进去**
- 停止打印后再点动该通道：**离合也吸不上**

---

## 根因：两个模块互抢同一个离合槽位

本机是「**1 台电机 + 8 路离合**」，同一时刻只能服务 1 个通道。

打印中换料时，打印机先退旧料、再送新料。此时：

| 模块 | 它看到 | 它就做 |
|---|---|---|
| `check_buffer_status()` | 旧通道 motion 还是 `on_use` | 按缓冲微动 `AMCU_drive_channel(旧, FEED)` → `clutch_request_engage(旧)` |
| `AMCU_motion()` 的 switch | 新通道 `need_send_out` | `AMCU_drive_channel(新, FEED)` → `clutch_request_engage(新)` |

两个调用**每循环交替**，而 `s_pending_clutch` 只有**一个槽**、
`s_pending_ch` 是**单值**（`src/AMCU.cpp`）：

```cpp
static clutch_req_t *s_pending_clutch = NULL;   /* 只有一个槽 */
static int           s_pending_ch    = 0;       /* 单值 */
```

于是目标通道每循环被改一次 → 后台 `clutch_task` 反复执行
"全断 60ms + 吸合 80ms"：

- 现场听到 **咔咔咔**（电磁铁反复吸放）
- 电机永远等不到一次稳定的吸合 → `motor_set_dir_speed()` 一直不被调用
- **两个方向都不转**，料退不出也送不进

### 第二个 bug：`is_channel_printing(cur)` 提前 return

```cpp
} else if (is_channel_printing(cur)) {   /* ← 在 switch(act) 之前 */
    ...
    return;                              /* ← 新通道的 need_send_out 永远执行不到 */
}
```

打印中换料时 `cur`（`get_now_filament_num()`）往往还指着**旧通道**——
它的 motion 还是 `on_use`。于是走进这个分支直接 `return`，
`switch(act)` 里的 `need_send_out` **一次都跑不到**。

如果 `cur` 已经切成新通道，则旧通道 `on_use` 仍会被
`check_buffer_status()` 吸合，照样互抢。

---

## 修复：统一通道仲裁

### 1. 新增 `find_motion_channel()`

```cpp
/* 优先级：退料 > 进料（打印机换料一定是先退旧料再送新料） */
static int find_motion_channel(void)
{
    int send = -1, pull = -1, send_mm = -1, pull_mm = -1;
    for (int i = 0; i < MAX_FILAMENT_NUM; i++) {
        _filament_motion_state_set m = get_filament_motion(i);
        if (m == need_pull_back)      { if (pull    < 0) pull    = i; }
        else if (m == need_send_out)  { if (send    < 0) send    = i; }
        else if (m == act_pull_mm)    { if (pull_mm < 0) pull_mm = i; }
        else if (m == act_send_mm)    { if (send_mm < 0) send_mm = i; }
    }
    if (pull    >= 0) return pull;
    if (send    >= 0) return send;
    if (pull_mm >= 0) return pull_mm;
    if (send_mm >= 0) return send_mm;
    return -1;
}
```

### 2. `check_buffer_status()` 开头让出

```cpp
if (is_need_pull_back) return;          /* v1.1.12 已有 */
if (find_motion_channel() >= 0) return; /* v1.1.15 新增：有进/退料，补料整体让出 */
```

### 3. `AMCU_motion()` 用 `svc` 代替 `cur`

```cpp
int svc = cur;                          /* 本循环真正服务的通道 */
{
    int mc = find_motion_channel();
    if (mc >= 0) svc = mc;
}
if (ams_enable) act = get_filament_motion(svc);

if (is_need_pull_back) {
    follow_feed_reset_ex(false);
} else if (is_channel_printing(svc)) {  /* ← 原来写的是 cur */
    ...
    if (is_channel_before_on_use(svc)) follow_feed_pump(svc + 1);
    else follow_feed_reset_ex(true);
    return;
}
```

`switch(act)` 里的 `cur` **全部**换成 `svc`：
`s_last_action[]` / `last_pullback_meters[]` / `get_filament_meters()` /
`is_filament_online()` / `AMCU_drive_channel(svc + 1, ...)` / 日志里的料位号。

---

## 新增诊断：离合仲裁计数

网页「AMS」卡片多一行 **离合仲裁**：

```
切目标 N 次 · 被打断 M 次
```

| 字段 | 来源 | 含义 |
|---|---|---|
| `target_switch` | `amcu_clutch_switch_count()` | 挂起的吸合目标被改成别的通道的次数 |
| `target_abort` | `amcu_clutch_abort_count()` | 挂起的吸合请求被"停机"打断的次数 |

**正常打印这两个数很慢地涨**（只在真正换料时各 +1）。
如果"咔咔响"时**每秒涨几十**，就是又在互抢。超过 50 次网页标红。

`/diag_reset` 现在一并清零（原来只清 `conflicts`）。

---

## 验证步骤

1. 刷 `firmware.bin`，确认版本号 `1.1.15-s3ams`
2. 网页点「上电诊断 → 清除计数」，把两个计数归零
3. 打印中触发一次换色
4. **换色过程中盯「离合仲裁」那一行**：
   - `切目标` +1、`被打断` +1（各一次）→ 正常，修复生效
   - 数字飙到几十上百 → 还有别的路径在抢，把日志发我
5. 听离合：应该只有**一次**"咔"（旧通道断开）+ **一次**"嗒"（新通道吸合），
   不再是连续的"咔咔咔咔"

---

## 已排除的疑点（都逐行查过，不是原因）

| 怀疑对象 | 为什么不是 |
|---|---|
| `clutch_engage()` 的"残留吸合复核" | `clutch_apply_all_off()` 会先清 `s_active`，软件记账兜底不会误判成残留 |
| `clutch_assert_single(true)` | 只在 `count > 1`（多路同时吸合）时才 `release_all` |
| `follow_feed_reset_ex(true)` | 只调 `motor_stop()`，**不动离合** |
| 堵转保护反复跳闸 | `CURRENT_DEFAULT_STALL_MA = 0` = 只显示不保护 |
| `AMCU_drive_stop()` 每循环刷队列 | 有 `already_releasing` / `already_off` 幂等保护 |
| `clutch_engaged_mask()` 回读不可信 | 有 `s_readback_ok` 软件记账兜底，且自检会打日志 |

---

## 已知遗留

- **单槽仲裁是"逻辑上避免并发"，不是队列化**。如果以后出现第三个会驱动电机的模块，
  仍要在它开头加 `find_motion_channel() >= 0 → return`。更好的做法是把
  `s_pending_clutch` 换成带优先级的请求队列。
- v1.1.14 遗留：后 4 通道在临时微动模式下没有独立微动
- 退料（尤其第二次退料）尚未单独验证
- 长时间打印验证跟随送料 250/350ms 节奏
- 堵转阈值 `stall_ma = 0` 待实测标定
