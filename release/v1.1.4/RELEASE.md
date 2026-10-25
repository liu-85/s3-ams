# s3-ams v1.1.4 发布说明（OTA 500 修复 + 微动全通道稳定滤波）

**构建日期**：2026-10-25 · **本地编译**（PlatformIO 6.1.18, env=s3ams）

## 修复 1：网页升级"上传失败（HTTP 500）"

**根因（两层 bug 的第二层）**：v1.1.1 把上传改成了 multipart 流式，
但 `server.on("/ota_upload", HTTP_POST, handle_ota_upload)` 是 **3 参数
重载**——它把上传回调注册进 `_fileUploadHandler`，而该成员默认是
**nullptr** → `FunctionRequestHandler::canUpload()` 返回 false →
`_parseForm` 里 START/WRITE/END 三个 `server.upload()` 事件**一个都
不会送来**。结果：1.18MB 被逐字节解析后全部丢弃，POST 完成回调里
`Update.end()` 发现从未 begin → 返回 500。

**修复**：
- 改用 **4 参数 on()**：`server.on("/ota_upload", HTTP_POST, handle_ota_upload_done, handle_ota_upload)`
  上传回调作为第 4 个参数，START/WRITE/END 正常送达
- POST 完成回调设为空手（响应已在 END 分支发过，重复 send 会坏连接）
- `Update.begin()` 回到 `UPDATE_SIZE_UNKNOWN`：multipart 的
  Content-Length 含 boundary 头尾（比镜像大几百字节），不拿它当 size，
  走"边写边擦"成熟路径

## 修复 2：微动滤波只对通道 1 有效，其他通道乱跳

**根因**：v1.1.0 加的 350mV 无信号门限只挡得住通道 1 实测的那种
~371mV 悬空电压。其他通道悬空电压各不相同（走线/漏电流），有的落在
START 区（<433mV）就乱跳——所以看起来"只修了通道 1"。

**修复（全通道，不依赖魔法电压）**：加**稳定性窗口**滤波——
- 每通道保留最近 8 次（40ms）原始 mV 读数
- 窗口内 (max−min) > 100mV → 判"读数不稳"→ **中间态（无动作）**
- 悬空引脚读数大幅跳动 → 一律中间态；真实分压稳定 → 正常三档分类
- 与 350mV 门限双重保险，切换识别延迟约 40ms（原有去抖 3×5ms，
  体感无差别）

## 产物（一键刷写，本次必须走串口）

**注意：当前设备上的 1.1.1 网页 OTA 本身是坏的（正是本版要修的），
所以这版必须用 COM6 串口刷**：

```powershell
python flash_s3.py COM6 --bootloader release\v1.1.4\bootloader.bin --partitions release\v1.1.4\partitions.bin --firmware release\v1.1.4\firmware.bin --no-watch
```

刷完验证：
1. 网页「系统升级」再传一次同一个 firmware.bin → 应显示"OTA 成功，
   设备将重启"（以后升级就走网页，不用再插 USB）
2. 网页「微动开关」卡片：没接微动的通道状态应稳定停在中间档，
   不再乱跳；接了的通道按压开始位/顶开停止位照常触发
