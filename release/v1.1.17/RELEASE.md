# v1.1.17 —— 离合没吸合彻底就给电机（进料吸不实、退料却"正常"的真相）

## 现象（用户实测 v1.1.16）

WEB 硬件调试点动，逐通道测试：

- **进料：电机在转，但当前通道离合"好像没吸合彻底"**
- **退料：正常，手摸电磁齿轮转得很快，明显吸合彻底**

---

## 根因：电机在电磁铁还没机械咬合时就开始带负载

`clutch_apply_one()` 的时序是**先置电平、再等机械稳定**：

```cpp
static inline void clutch_apply_one(int index, bool on, bool settle)
{
    gpio_set_level((gpio_num_t)s_pins[index], on ? s_active_level : s_inactive_level);
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(on ? CLUTCH_ENGAGE_MS : CLUTCH_RELEASE_MS));  // 80ms
    }
}
```

**电平一置高，`clutch_is_engaged()` 立刻就返回 true**，但电磁铁的机械咬合还要
`CLUTCH_ENGAGE_MS = 80ms`。

而上层正是靠 `clutch_is_engaged()` 决定什么时候给电机：

```cpp
// 改之前
if (clutch_is_engaged(want)) {
    motor_set_dir_speed(s_pending_dir, s_pending_speed);   // ← 立刻给电
}
```

主循环下一轮（几毫秒）就看到 engaged 为真 → 电机开始转 → **电磁铁还没咬合就带负载**
→ 打滑、吸合不彻底、扭矩不足。

---

## 为什么"退料看着正常"（这是最容易误判的地方）

| | 进料测试 | 退料测试 |
|---|---|---|
| 离合状态 | 通常要从**上一个通道切过来** | 往往**已经吸着同一通道**（刚测过进料） |
| `clutch_engage()` 走哪条路 | 完整流程：全断 60ms + 静默 20ms + 吸合 80ms = **160ms** | 开头就是**幂等分支**，立即返回 |
| 机械稳定时间 | 还没过就给电机 → **吸不实** | 早就稳定了 → **彻底** |

`clutch_engage()` 的幂等分支：

```cpp
/* 同通道重复吸合是幂等的 */
if (s_active == channel && gpio_get_level(s_pins[index]) == s_active_level) {
    xSemaphoreGive(s_lock);
    return true;        // ← 立即返回，没有 80ms 延时
}
```

所以退料"彻底"是**假象**——它根本没经历等待。进料才是真正暴露问题的场景。

---

## 修复：电机必须等机械稳定时间过去才给电

`AMCU_clutch_pump()` 用两个判据，满足其一才算稳定：

```cpp
const bool eng = clutch_is_engaged(want);

/* ① done —— 后台 clutch_task 已跑完 clutch_engage()，
 *    里面的 vTaskDelay(CLUTCH_ENGAGE_MS) 自然已经过去（权威判据）
 * ② 时间 —— 距请求发起已过 CLUTCH_ENGAGE_MS + CLUTCH_SETTLE_MS（兜底）
 *    因为 req_slot 只有 4 个槽，高频请求会覆盖槽位导致 done 永远不来 */
const bool settled = done ||
    ((uint32_t)(millis() - s_engage_req_ms) >= (CLUTCH_ENGAGE_MS + CLUTCH_SETTLE_MS));

if (eng && settled) {
    motor_set_dir_speed(s_pending_dir, s_pending_speed);   // 这时候才给电
    ...
}
```

配套三处改动：

1. **`AMCU_drive_channel()`**：新增 `s_engage_req_ms = millis()` 记录请求发起时刻；
   "已吸合"分支若发现本通道还在等稳定，改为返回 false 等 pump，不直接启动。
2. **`amcu_jog_pump()`**（WEB 点动）：`clutch_is_engaged()` 为真但吸合请求还没稳定时，
   **不**在这里给电机，交给 `AMCU_clutch_pump()` 统一在稳定后启动。
   新增 `amcu_engage_pending()`（前向声明，因为 `amcu_jog_pump` 在那些静态变量之前）。
3. 新增 `amcu_engage_pending()` 辅助函数。

---

## 效果

点「通道 N 进料」现在的时序：

```
t=0    请求吸合通道 N（异步入队，电机不给电）
t=0    clutch_task: 全断其他通道 + vTaskDelay(60ms)
t=60   vTaskDelay(20ms) 静默
t=80   gpio_set_level(N) 置高 → vTaskDelay(80ms)   ← 机械稳定
t=160  clutch_engage() 返回，标记 done
t≈160 主循环 settled → motor_set_dir_speed(FEED, 100%)   ← 这时候才转
```

**电机比之前晚约 100~160ms 启动**，换来离合完全咬合后再带负载。
点动总时长建议 ≥ 1000ms（默认值），否则稳定时间会占掉大半。

---

## 验证步骤

1. 刷 `firmware.bin`，确认版本号 `1.1.17-s3ams`
2. WEB 硬件调试，**逐个通道切换**着点进料（这是关键：必须换通道才触发 160ms 流程）
3. 手摸电磁齿轮：应该先听到"嗒"一声吸合，**约 0.15 秒后**电机才开始转，
   且齿轮是"咬住着转"，不再是打滑的空转
4. 再点退料，确认同样正常
5. 若仍不实，看串口有无：
   - `切换离合超时：有别的动作正占着总线` → `s_lock` 竞争
   - `吸合通道 N 前检测到残留吸合，已强制全断开并取消` → 有通道没断开干净

---

## 已知遗留

- **点动时长别设太短**：`JOG_TIME_MS_MIN = 200ms`，但离合切换要 ~160ms，
  设 200ms 只剩 40ms 给电机。建议 ≥ 1000ms。
- 单槽仲裁仍是"逻辑上避免并发"，不是优先级队列。
- `clutch_engage()` 幂等分支**没有 `motor_stop()`**（完整路径有）。
  同通道重复吸合时电机不会先停，切换瞬间可能有冲击。本次未改，怕影响既有节奏。
- v1.1.16 遗留项不变（后 4 通道无独立微动、退料二次验证、堵转阈值待标定）
