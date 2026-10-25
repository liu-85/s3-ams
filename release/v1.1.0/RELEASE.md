# s3-ams v1.1.0 发布说明

**构建日期**：2026-10-25 · **本地编译**（PlatformIO 6.1.18, env=s3ams）

## 产物（一键刷写）

```bash
python flash_s3.py COM6 \
  --bootloader release/v1.1.0/bootloader.bin \
  --partitions release/v1.1.0/partitions.bin \
  --firmware   release/v1.1.0/firmware.bin
```

偏移固化（与 partitions_16mb.csv 一致，保留 NVS/WiFi 配置）：

| 段 | 偏移 |
|---|---|
| bootloader | 0x0 |
| partitions | 0x8000 |
| firmware (ota_0) | 0x20000 |

## 本版变更

### 修复
1. **485 运动执行开关生效**：`/act485_set` 原来走 `handle_ok` 占位，只回
   `{"ok":true}` 根本没碰 `ams_enable`。现在真正写 `amcu_set_ams_enable()`，
   `/status` 增加 `act485` 回显，勾选状态刷新页面不丢。
2. **485 进料占空比**：`MOTOR_SPEED_485_FEED` 90% → **100%**（duty 255）。
   实测电机 ≥80% 才转得动，取满占空比留裕量；`duty = pct*255/100` 映射不变。
3. **微动悬空误触发**：开始位 4.7K / 停止位 10K 并到同一 ADC 脚，微动未接时
   ADC 读 ~371mV 噪声，正好落在 START 区（<433mV）。新增无信号门限
   `BOARD_SENSOR_MV_NOISE = 350mV`：低于门限判中间态，不触发 START/STOP。

### 新增：电机电流采样 + 堵转保护（AT8236 ISEN · GPIO3）
- ADC1_CH2 12bit/11dB 校准采样，10ms 周期，IIR 滤波 + 1s 滑动峰值窗口；
- `V = I × R`（板载 R11 = 200mΩ → 200mV/A；芯片硬限流约 2.5A）；
- 堵转保护参数 **Web 可调**（/current_set，NVS 持久化）：
  `r_mohm / stall_ma / stall_ms / blank_ms`，全部夹取到合法范围；
- **默认 stall_ma = 0（只显示、不保护）** —— 阈值按实测流程标定后填写：
  1. 网页「手动点动」正常送一段，记均值；
  2. 故意卡住料路堵转，记堵转值；
  3. 阈值取两者中间偏向正常值一侧（如正常 600mA / 堵转 1800mA → 1200~1500mA）；
- 起步屏蔽 blank_ms（默认 800ms）防起动电流（稳态 3~8 倍）误跳；
- 跳闸动作：停电机 + 断离合，latch 至「清除故障/峰值」（/current_clear）；
- latch 期间状态机禁止再驱动电机（防 驱动→跳闸→停止→再驱动 死循环）。

### 新增：手动点动（Web 硬件调试）
- `/jog_set`：点动时长（0.2~60s）与 PWM（5~100%）可选保存，NVS 持久化；
- `/hardware_test`：按保存的参数单次点动指定通道进/退料，非阻塞到时自停；
- 点动期间 AMCU_motion 让出总线；堵转 latch / 离合忙时拒绝启动。

### 其他
- `/comm_set` 如实返回"本固件固定 RS-485 直连打印机"，不再假装保存；
- APP_VERSION → 1.1.0-s3ams。

## 已知边界
- 单电机共享架构：电流采样不区分通道（/current_channel_set 仅确认）；
- WiFi 开启时微动 6/7/8（GPIO12/13 属 ADC2）读不到，属硬件限制；
- stall_ma 未实测标定前保护不生效（设计如此，防误停）。
