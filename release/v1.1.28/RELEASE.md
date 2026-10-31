# v1.1.28 —— 找到 40A3 的真凶：WiFi 阻塞 25 秒，485 全程失联

> 版本：`1.1.28-s3ams`
> 日期：2026-10-31
> 主题：**修掉"上电/重启就报 `[0500-40A3]`"的根因**

---

## 一、根因

### 启动顺序

```cpp
void setup() {
    AMCU_init();          // ← 485 在这里初始化好（BambuBus_init → BambuBUS_UART_Init）
    web_server_start();   // ← wifi_init() → wifi_try_connect() 阻塞等 WiFi
    loop();               // ← 到这一步才开始跑 AMCU_run() → BambuBus_run()
}
```

### 阻塞点

`wifi_try_connect()` 里的等待：

```cpp
uint32_t start = millis();
while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
    delay(200);          // ← 最长 WIFI_TIMEOUT_MS = 25000ms
}
```

**`WIFI_TIMEOUT_MS = 25000`。**

这 25 秒里主循环**一次都没跑** → 485 一个字节都不处理、一个应答都不发 →
打印机等不到 AMS-A 的应答 → 直接报 `[0500-40A3] AMS(或AMS lite) A 通信异常`。

### 为什么是"间歇"的

| WiFi 连接耗时 | 结果 |
|---|---|
| 1~3 秒 | 落在打印机重试窗口内 → 看着正常 |
| 5 秒以上 | 打印机超时 → **报 40A3** |

这正好解释了之前的全部现象：

- **"同一份固件有时正常有时报错"** —— 取决于路由器/DHCP 快慢
- **"刷完重启就报，再重启又好"** —— 每次上电 WiFi 握手时间不同
- **"改哪一行代码都可能中招"** —— 其实和那些改动无关，是启动时序问题
- **v1.1.22 通信正常、v1.1.24 又报** —— 同版本重刷结果都可能不同

之前我在 v1.1.22 修 `0x402` 的 `return`→`clamp` 是**真实存在的另一个 bug**（那个确实会导致
40A3），但它不是这个"反复出现"的主因。这次找到的是启动时序这条。

---

## 二、修法

### ① 等待期间继续服务 485（核心）

新增 `wifi_wait(ms)`：每 5ms 跑一次 `BambuBus_run()`。

```cpp
static void wifi_wait(uint32_t ms)
{
    const uint32_t start = millis();
    while (millis() - start < ms) {
        BambuBus_run();   /* ★ 485 必须一直在应答 */
        delay(5);
    }
}
```

替换掉 WiFi 流程里所有裸 `delay()`：

| 位置 | 原来 | 现在 |
|---|---|---|
| `wifi_stop_ap()` 关 AP 后 | `delay(200)` ×2 | `wifi_wait(200)` ×2 |
| 配网前 | `delay(300)` | `wifi_wait(300)` |
| `WiFi.disconnect()` 后 | `delay(200)` | `wifi_wait(200)` |
| **等待连接（最长 25 秒）** | `delay(200)` | **`wifi_wait(200)`** |
| 配网失败后 | `delay(300)` | `wifi_wait(300)` |

`BambuBus_run()` 只做收发与状态机记账、**不驱动电机**，开机阶段调用是安全的。

### ② `AMCU_init()` 里的 500ms 也不失联

```cpp
BambuBus_init();
// 原来：delay(500);
for (int i = 0; i < 50; i++) { BambuBus_run(); delay(10); }
```

### ③ 主循环周期 10ms → 2ms

485 是 **1.2288 Mbps ≈ 154 字节/ms**。原来每 10ms 才排空一次 UART FIFO
（FIFO 只有 256 字节）—— 打印机连发两帧就可能溢出丢字节，CRC 错 → 丢帧。

2ms 时单轮最多积压 ~300 字节，仍在 FIFO 容量内；2ms 的 `vTaskDelay`
足够把 CPU 让给 WiFi/Web 任务，不影响吞吐。

### ④ 新增诊断：主循环最大阻塞间隔

`/status` 的 `ams.link` 多了一项：

```json
"loop_max_gap_ms": 7
```

| 值 | 含义 |
|---|---|
| < 20ms | 主循环健康 → 40A3 与固件时序无关，查硬件（接线/A-B 极性/收发器） |
| > 100ms | 有东西在卡主循环（Web 大响应 / OTA / 日志刷屏） |

`POST /diag_reset` 会一起清零。

### ⑤ WiFi 连接耗时打进日志

```
WiFi 已连接: xiaomi2 / IP 192.168.2.100 (PS=NONE, AP 已关, 连接耗时 2380ms)
```

这个数就是"485 曾经失联多久"的直接证据。以后一看就知道路由器状态。

---

## 三、刷完怎么验证

### 最直接的一条

**反复重启 5 次**，看是否还会报 40A3。

修之前：WiFi 慢的那几次必然报。
修之后：不管 WiFi 多久，485 全程在线，不应再报。

### 看日志

```
WiFi 已连接: xiaomi2 / IP 192.168.2.100 (... 连接耗时 2380ms)
```

如果这个数经常 >5000ms，说明路由器/DHCP 慢 —— 现在不影响 485 了，
但值得换个信道或给设备固定 IP。

### 看 `/status` 的 `ams.link`

```json
"link": {
  "rx_bytes": 128640, "rx_frames": 2144, "crc_err": 0, "overflow": 0,
  "tx_402": 12, "tx_103": 8, "tx_06": 4,
  "last_rx_age_ms": 87, "loop_max_gap_ms": 7
}
```

- `loop_max_gap_ms` 小、`crc_err` = 0 → 链路健康
- `crc_err` 猛涨 → 干扰 / 共地 / A-B 极性
- `rx_bytes` 不涨 → 物理层断

---

## 四、改动文件

| 文件 | 改动 |
|---|---|
| `src/web_server.cpp` | 新增 `wifi_wait()`；WiFi 流程 5 处 `delay()` 替换；连接耗时入日志；`/status` 加 `loop_max_gap_ms`；`/diag_reset` 一并清零 |
| `src/main.cpp` | 主循环 `delay(10)` → `delay(2)`；新增主循环最大间隔统计 |
| `src/AMCU.cpp` | `AMCU_init()` 的 `delay(500)` 改成边等边收 485 |

---

## 五、升级

```bash
python flash_s3.py COM6
```

或网页 OTA 上传 `firmware.bin`。

刷完**连续重启 5 次**验证 —— 这是判断有没有真修好的唯一标准。
