/**
 * web_server.h —— ESP32-S3 Web 服务器（Arduino 框架）
 *
 * 保留 esp32s3-485 的 Web 前端内容，提供兼容的后端。
 * 功能：WiFi 管理、状态查询、日志、微动设置、OTA 升级。
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

/* 启动 Web 服务器（在 WiFi 就绪后调用） */
void web_server_start(void);

/* 处理 Web 服务器请求（在 loop 中调用） */
void web_server_handle(void);

/* 向日志环形缓冲写入一条（供 /log 端点读取） */
void web_log(const char *msg);

/* 安装 esp_log vprintf hook：把 log_i/log_w/log_e 同步进网页日志缓冲。
 * 必须在 setup() 最开头调用，否则启动阶段的日志会漏掉。 */
void web_log_install_hook(void);

/* 当前网络模式：0=STA已连, 1=AP热点, 2=离线 */
int web_net_mode(void);

#endif /* WEB_SERVER_H */
