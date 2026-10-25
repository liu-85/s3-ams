/**
 * ams_log.h —— 让 Arduino 的 log_i/log_w/log_e 同时进「网页设备日志」
 *
 * ★ 为什么需要这个文件（2026-10-26 定位）：
 *   固件里到处都是 log_i/log_w/log_e。曾以为挂上
 *   esp_log_set_vprintf(web_log_vprintf) 就能把它们截到网页日志，
 *   实际 /log 永远是空的 —— 因为 Arduino 的 log_* 宏**不走 esp_log**：
 *
 *     esp32-hal-log.h:  log_i(...) → log_printf(...)
 *     esp32-hal-uart.c: log_printf → log_printfv → ets_printf("%s", ...)
 *                       （ets_printf 是 ROM 函数，直接走 putc1 串口钩子）
 *
 *   esp_log_set_vprintf 只能截 ESP_LOGI/esp_log_write 这条 IDF 通道，
 *   Arduino 自己的 log_printf 根本不经过它。（v1.1.4 及以前 /log 全空）
 *
 *   所以在这里对 log_* 宏做「劫持」：串口该出的一个字不少
 *   （还是原来的 ARDUHAL_LOG_FORMAT + log_printf），额外抄送一份
 *   纯消息文本进网页日志环（web_logf → web_log，带去重/seq）。
 *
 * ★ 用法：每个 .cpp 把本头放在**第一个** include（它会先拉 Arduino.h，
 *   拿到原始宏定义后再 #undef/#define，之后的 log_i 就是我们的版本）。
 *   IDF 组件的日志（WiFi/OTA 内部 ESP_LOGx）仍由 web_log_vprintf
 *   （esp_log_set_vprintf）兜底捕获，两条通道互补。
 *
 * ★ 注意：本宏也会影响其后 include 的核心库内联函数里的 log_* 调用
 *   （只是多抄送一份网页日志，串口行为不变，无害）。
 */

#ifndef AMS_LOG_H
#define AMS_LOG_H

#include <Arduino.h>

/* 网页日志入口（定义在 web_server.cpp，行式环形缓冲 + 连续相同消息去重） */
void web_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* ------------------------------------------------------------------ *
 * 串口输出完全保持原样（原宏的格式串一字不改），前面加一次网页抄送。
 * ------------------------------------------------------------------ */
#undef log_i
#define log_i(format, ...)                        \
    do {                                          \
        web_logf(format, ##__VA_ARGS__);          \
        log_printf(ARDUHAL_LOG_FORMAT(I, format), \
                   ##__VA_ARGS__);                \
    } while (0)

#undef log_w
#define log_w(format, ...)                        \
    do {                                          \
        web_logf(format, ##__VA_ARGS__);          \
        log_printf(ARDUHAL_LOG_FORMAT(W, format), \
                   ##__VA_ARGS__);                \
    } while (0)

#undef log_e
#define log_e(format, ...)                        \
    do {                                          \
        web_logf(format, ##__VA_ARGS__);          \
        log_printf(ARDUHAL_LOG_FORMAT(E, format), \
                   ##__VA_ARGS__);                \
    } while (0)

/* log_n 原本就用 ERROR 档格式，照旧 */
#undef log_n
#define log_n(format, ...)                        \
    do {                                          \
        web_logf(format, ##__VA_ARGS__);          \
        log_printf(ARDUHAL_LOG_FORMAT(E, format), \
                   ##__VA_ARGS__);                \
    } while (0)

#endif /* AMS_LOG_H */
