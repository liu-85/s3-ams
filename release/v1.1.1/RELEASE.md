# s3-ams v1.1.1 发布说明（OTA 修复）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO 6.1.18, env=s3ams）

## 修复：Web 升级"上传中断"

**根因**（arduino-esp32 WebServer 库行为）：前端原来用
`Content-Type: application/octet-stream` 裸 POST 固件。WebServer 的
`_parseRequest` 对**非 multipart** 的 POST 会把**整个 body 一次性读进 RAM**
（`readBytesWithTimeout` 反复 realloc）。本板不开 PSRAM，内部堆只有几百 KB，
1.18MB 的固件读到一半 realloc 失败 → 解析函数返回 false → 服务端主动掐断
连接 → 浏览器 `xhr.onerror` → "上传中断"。进度条还在走是因为发送端还没被
通知，TCP 窗口满了才停，然后彻底报错。**与家庭网络质量无关**（ping 通、
0% 丢包也能稳定复现）。

**修复**：
- 前端改用 `FormData`（multipart/form-data），走 WebServer 的流式
  `_parseForm` 路径：每 1436 字节触发一次 `server.upload()` 回调，
  边收边写 flash，内存占用恒定（1436 字节缓冲）
- 固件侧 `handle_ota_upload` 同步加固：
  - 优先按 `Content-Length` 开分区（拿不到再退回未知大小）
  - `Update.write` 返回值逐块校验，失败即 `Update.abort()` 并记录累计量
  - 上传中途不再发送任何响应（旧代码在 begin 失败时于 upload 回调内
    `server.send(500)`，会污染流），统一在 UPLOAD_FILE_END 收尾
  - 全程日志：开始/完成字节数、耗时、失败点

## 产物（一键刷写）

```bash
python flash_s3.py COM6 \
  --bootloader release/v1.1.1/bootloader.bin \
  --partitions release/v1.1.1/partitions.bin \
  --firmware   release/v1.1.1/firmware.bin
```

偏移：bootloader@0x0 · partitions@0x8000 · firmware(ota_0)@0x20000（保留 NVS）

## 升级注意

- 从 1.0.0/1.1.0 升级直接网页上传本目录 firmware.bin 即可（修好后的
  multipart 路径）
- 若仍失败：换 Chrome/Edge 重试；确认上传的就是 1.17MB 左右的完整
  firmware.bin（不是 bootloader/partitions）
- 备用分区不会被写坏：写坏会自动回滚到旧固件
