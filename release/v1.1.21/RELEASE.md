# v1.1.21 —— 只保留 AMS-A / AMS-B（修耗材页重复 AMS）

## 一句话

本板 8 通道 ÷ 4 料位 = **2 台 AMS**，但 `AMS_COUNT` 一直是 4，
多出来的 AMS-C / AMS-D 是**没有任何物理通道的幽灵设备**。
这一版把台数收到 2，并把所有"越界请求"改成**不应答**。

---

## 根因（四个叠加）

### ① `AMS_COUNT = 4`，物理只有 2 台

`DEFAULT_AMS_COUNT` 是 4，本板只有 8 个物理通道 = 2 台 × 4 料位。
AMS-C / AMS-D 从来不存在，但一直被当成真设备上报。

### ② v1.1.16 把全部 4×4 料位设成 `online`

`BambuBus_init()` 里那句 `j.statu = online;` 是遍历 **4×4 = 16 个料位**的。
而 `send_for_Fxx()` 的 0x05 注册循环条件正是
`if (data_save.filament[i][0].statu != online) continue;`
→ **幽灵 C/D 也满足条件，被注册上去了** → 耗材页多出两组空卡片。

### ③ `0x402` 越界 clamp 到 0（制造重复的关键）

```cpp
AMS_num = printer_data_long.datas[33];
if (AMS_num >= AMS_COUNT) AMS_num = 0;   // ← 越界照样应答，只是换成台 0 的序列号
```

工程里其他长包（`0x21A` / `0x211` / `0x05`）都是 `>= AMS_COUNT` 就 `return`，
**只有这里不一致**。打印机问台 2/3/4… 时，我们回的是"台 0 的序列号"，
打印机很可能据此认为"又发现一台 AMS"→ 列表累积。

### ④ 版本包只有 21 字节，台号只写 `[20]`

esp32s3-485 的 `build_ver()` 是 `VER_FRAME_LEN = 22`，台号同时写 `[20]` 和
`[21]`（注释：BMCU370 读 `[20]`、Tianshu 读 `[21]`）。
本项目 `ver_pkt[21]` 根本没有 `[21]` 这个字节 —— 如果打印机读 `[21]`，
就完全读不到台号，所有 AMS 台号都是 0。

---

## 改动

### `src/settings_storage.h`

```c
#define MAX_AMS_COUNT     2
#define DEFAULT_AMS_COUNT 2
```

### `src/settings_storage.cpp`

NVS 里可能还存着旧值 4，加载时上限收到 `MAX_AMS_COUNT`：

```cpp
if (data.ams_count >= 1 && data.ams_count <= MAX_AMS_COUNT) {
    AMS_COUNT = data.ams_count;
} else {
    AMS_COUNT = DEFAULT_AMS_COUNT;   // 旧值 4 不合法 → 回落 2
}
```

### `src/BambuBus.cpp`

| 位置 | 改动 |
|---|---|
| `BambuBus_init()` 料位初始化 | `statu = (a < AMS_COUNT) ? online : offline` —— 只有真实存在的 2 台报在线 |
| `send_for_long_packge_version()` 0x402 | 越界 `return`（不再 clamp 到 0）；`datas[33]` 前先查 `data_length > 33` |
| `send_for_long_packge_version()` 0x103 | `ver_pkt[21]` → `ver_pkt[22]`，`[20]` 和 `[21]` 都写台号 |
| `send_for_Fxx()` 0x06 分支 | `buf[5] >= AMS_COUNT` 直接 `return` |

---

## ⚠️ 没改的两处（有意保留）

### 1. `use_flag`（Dxx 应答 `[19]`）的语义错位

社区文档的语义是 `0x00` 无料 / `0x01` 到五通 / `0x02` 到挤出机 /
`0x04` 退料中 / `0x08` 进料中。本项目现在是：

- 送料中报 `0x02`、使用中报 `0x04`、空料位被强制改成 `0x02`

**明知错位但不动**：这套编码从 Tianshu 继承，A1 进料已实测跑通，
说明打印机主要靠 `[41]` 料位位域和 `pressure` 判断。改它等于把已验证的
进料路径重洗一遍。详见 `docs/PROTOCOL-485.md` 第 4.1 节。

### 2. `0x06` 握手的台号锁死

```cpp
if (res_for_06_num == 0xFF) res_for_06_num = req_num;
else ams_num = res_for_06_num;   // 之后所有 0x06 都用第一次的台号
```

`res_for_06_num` 上电后**永不重置**。这是当初为了修"第二台永远不出现"加的，
但方向可能是反的（会造成"同一台被反复发现"）。本次只加了越界保护，
没动锁死逻辑 —— 怕把已经能认到的 AMS-B 弄丢。如果刷完这一版**仍然重复**，
下一步就动它。

---

## 验证

1. 刷 `firmware.bin`，确认版本 `1.1.21-s3ams`
2. **打印机要重新枚举**：拔掉 485 线等 5 秒再插，或重启打印机
   （打印机会缓存 AMS 列表，不重新枚举的话旧的幽灵卡片可能还在）
3. 耗材页应只剩 **AMS-A + AMS-B**（共 8 槽）
4. 逐通道点动进料/退料，8 个通道都要正常
5. 打印中换色，确认 A/B 之间的料位切换正常

> 如果重新枚举后幽灵卡片仍在，说明打印机把列表存在自己的 NVS 里，
> 需要在打印机上"移除 AMS"或恢复出厂网络设置再连一次。

---

## 编译数据

```
RAM:   23.4% (76604 B / 327680 B)
Flash: 41.0% (1234996 B / 3014656 B)
```

firmware.bin md5: `2a70d9471f7b6b02edf0999e8314f5ba`
