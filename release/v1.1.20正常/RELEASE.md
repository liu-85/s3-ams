# v1.1.20 —— 修「通道设置选完颜色两秒变回原色」+ 说明全部默认折叠 + 文档补 IO 定义

**版本号**：`1.1.20-s3ams`
**基于**：v1.1.19
**编译**：RAM 23.4% (76604 B) / Flash 41.0% (1234952 B)

---

## 一、修颜色设置不生效（本次核心）

**现象**：通道设置里点色块 → 弹窗选色 → 弹窗关闭，但颜色**两秒内变回原来的颜色**。

**根因（两处叠加）**：

1. **前端每 2 秒重建整块**：`applyStatus` 里，当固件不返回 `access_list` 时，
   走的是 `else if (!accessFromDevice)` 分支，而这个分支**从不把 `accessFromDevice`
   置 true** → 每次 `/status` 轮询都重新 `renderAccess(默认色)`，
   把用户刚选的颜色冲掉。
2. **固件根本不存 `access_list` / `color_list`**：`/status` 从不输出这两个数组，
   `/access_set` 还是 `handle_ok` 空占位 → 就算前端不重建，**保存也不生效**。

**修复**：

- 前端：默认映射**只画一次**；之后只在设备数据真变时重建；
  只有「使用中」标记变时走 `updateCurrentOnly()`（它只改文字，不碰颜色）。
- 固件：新增 `s_access_list[16]` / `s_color_list[16]`，存 **NVS `s3ams-access`**（掉电保留）：
  - `/status` 输出 `access_list` + `color_list`
  - `/access_set` 改为 `handle_access_set()` **真保存**（新增 `json_get_int_array()` 解析数组）

现在：选色 → 弹窗关闭 → 颜色保持 → 点「保存」→ 写 NVS → 刷新/重启后仍是该颜色。

---

## 二、说明文字全部默认折叠

- `collapseTips()` 显式 `det.open = false`，保证所有 `div.tip` 转成的 `<details>` 默认折叠。
- **动态生成的说明块也折叠**：微动页的说明是 JS 运行时生成的（boot 时的
  `collapseTips` 覆盖不到），现在 `sensor_list` 每次重绘后都调用一次 `collapseTips()`。
- 说明内容同步修正：
  - **OTA 说明改正**：原来写的是 `idf.py build` 出来的 `esp-ams-s3.bin`（ESP-IDF 版遗留），
    本项目是 **PlatformIO**，改为 `firmware.bin`（`.pio/build/s3ams/firmware.bin`
    或 `release/vX.Y.Z/firmware.bin`）。
  - 去掉 OTA 说明里与本项目无关的「MicroPython 版」段落。
  - 「退料参数」卡片顶部加醒目提示：**本卡参数当前固件未实现**（后端空占位），
    调退料时长请用「退料时间」卡片。

---

## 三、文档补充项目 IO 定义（`docs/HARDWARE.md`）

新增两节：

- **1.4 完整 GPIO 速查（按 GPIO 号排序）**：1/2/3/4/5/6/7/8/9/10/11/12/13/14/15/16/17/18/21/39/42/47/48
  每个脚的功能、方向、ADC 通道、备注一览。
- **1.5 临时直连微动模式的 IO 配对**：通道 1~4 的「开始位 / 停止位」各对应哪个微动、哪个 GPIO。

同时标注 **ADC2 陷阱**：GPIO11/12/13 属 ADC2，WiFi 开时 ADC 读不到；
`digitalRead` 不受限（临时模式因此全程开 WiFi 也正常）。

---

## 四、改动文件

| 文件 | 改动 |
|---|---|
| `web/app.js` | 通道设置渲染逻辑重写（不再每轮重建）；`collapseTips` 加 `open=false`；微动说明动态折叠；OTA 说明改 `firmware.bin` |
| `web/index.html` | OTA 说明改 `firmware.bin`；「退料参数」加未实现提示 |
| `src/web_server.cpp` | 新增 access/color NVS 存储、`json_get_int_array()`、`handle_access_set()`；`/status` 输出 `access_list`/`color_list`；`/access_set` 改真实现；`APP_VERSION` → `1.1.20-s3ams` |
| `docs/HARDWARE.md` | 新增 1.4 完整 GPIO 速查、1.5 临时微动配对 |
| `src/web_files.h` | 由 `tools/generate_web_files.py` 重新生成 |

---

## 五、验证

1. 版本号显示 `1.1.20-s3ams`
2. 通道设置 → 点色块 → 选颜色 → 弹窗关闭后**颜色保持不变**（等 10 秒也不变）
3. 点「保存」→ 提示「通道与颜色已保存」
4. **刷新页面 / 重启设备** → 颜色仍是刚才选的颜色（NVS 持久化）
5. 所有页面说明默认折叠，点「说明（点开看）」才展开

---

## 六、说明

- 颜色/通道映射是**本机记录**（用于网页显示与料盘位对应），不参与 485 协议。
- 「退料参数」卡片仍是空占位（历史遗留），已在卡片里明确标注。
