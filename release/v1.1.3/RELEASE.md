# s3-ams v1.1.3 发布说明（送料不起转修复 + 485 原始报文 + 监控脚本）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO 6.1.18, env=s3ams）

## 修复：该送料时"离合响了但电机没电没转"

**根因**：`AMCU_clutch_pump()` 里离合咬合后调 `motor_set_dir(motor_get_dir())`
"启动电机"——但电机从未启动过，`motor_get_dir()` 拿到的是 **STOP**，
实际执行的是 `motor_stop()`。症状：打印机点进料/退料 → 离合吸合 → 电机没电。

**修复**：`AMCU_drive_channel()` 发起异步离合请求时把 dir/speed 存进
`s_pending_dir/s_pending_speed`，咬合后 pump 按记忆值
`motor_set_dir_speed(s_pending_dir, s_pending_speed)` 启动。

## 新增：485 原始报文记录（/bus485raw）

- 从上电第一帧起记录双向报文（环形 32 帧 × 256 字节），RX=打印机发来、
  TX=本机回复，自旋锁保护
- `GET /bus485raw` → 文本 dump；`?since=seq` 只回新帧（脚本轮询用）
  行格式：`<seq> <t_ms> <RX|TX> <len> <hex>`
- 网页"通讯"页的「原始报文」链接（前端早有，固件此次才实现）

## 新增：两个 Windows 监控脚本（tools/）

| 脚本 | 用途 |
|---|---|
| `motor_test.bat [IP] [通道] [秒] [PWM%]` | 电机检测：自动点动进料/退料，实时显示方向/离合/电流/峰值，给出"转 / PWM没推动 / 软件没驱动"三档判定 |
| `mon485.bat [IP] [轮询ms]` | 485 监控：RX 青色=打印机指令、TX 绿色=本机回复；附带 WEB 日志（新行才出、连续相同折叠 xN）和状态行（没变化不刷新） |

结构为 bat 启动器 + ps1 负载（`-File` 方式，无 iex/引号转义问题；
.ps1 带 UTF-8 BOM，PowerShell 5.1 中文正常）。POST 一律
`application/json`（固件读 `server.arg("plain")`，表单编码会 400）。

## 实测备注

- 空闲时 ISEN 噪声约 0~100mA（20mV 级 ADC 噪声）——堵转阈值标定时
  要远高于此（建议 ≥500mA），并以"正常送料均值/堵转值"两点法标定
- 本机 COM 口在编译环境枚举不到，刷写需在用户侧 PowerShell 执行

## 产物（一键刷写）

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.3\bootloader.bin --partitions release\v1.1.3\partitions.bin --firmware release\v1.1.3\firmware.bin --no-watch
```
