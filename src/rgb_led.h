/**
 * rgb_led.h —— 板载 WS2812 RGB 状态灯（v1.1.29）
 * ============================================================================
 *
 * 硬件：单颗 WS2812，数据脚 = BOARD_PIN_LED（默认 GPIO42）。
 *
 * 驱动：用 Arduino-ESP32 内置的 neopixelWrite()（core 自带，走 RMT 硬件发送，
 *       不需要 FastLED / Adafruit_NeoPixel 等外部库）。
 *
 * 为什么单独一个模块：
 *   原来 AMCU.cpp 里的 set_led_state() 只是拉 GPIO 电平（给普通单色 LED 用的），
 *   对 WS2812 完全无效 —— 灯要么不亮要么只有一颗固定的白点。
 *   这里做真正的 RGB 状态机 + 呼吸灯。
 *
 * 刷新策略：
 *   neopixelWrite() 每次约 30us（24bit × 1.25us）。485 是 1.2288Mbps，
 *   30us 会丢 ~3.7 字节，所以**不能每轮都刷**。
 *   rgb_led_tick() 内部限流到 50ms 一次（20fps），呼吸效果够顺，
 *   而且主循环里单次最多被打断 30us，不影响 485 收帧。
 * ============================================================================
 */

#ifndef RGB_LED_H
#define RGB_LED_H

#include <Arduino.h>

typedef enum {
    LED_MODE_BOOT = 0,   /* 启动自检：蓝色慢闪 */
    LED_MODE_WIFI_WAIT,  /* 等 WiFi / 未联网：黄色慢闪 */
    LED_MODE_IDLE,       /* 就绪空闲：绿色低亮常亮 */
    LED_MODE_PRINTING,   /* 打印中：当前耗材颜色呼吸 */
    LED_MODE_PULLBACK,   /* 退料中：橙色呼吸 */
    LED_MODE_ERROR,      /* 故障（堵转/双微动失效）：红色快闪 */
} led_mode_t;

/* 初始化（上电先灭灯，避免残留旧色） */
void rgb_led_init(void);

/* 切换状态 */
void rgb_led_set_mode(led_mode_t m);
led_mode_t rgb_led_get_mode(void);

/* 打印中要用的颜色（0xRRGGBB）。由主循环按"当前使用通道"的通道颜色传入 */
void rgb_led_set_print_color(uint32_t rgb888);

/* 主循环每轮调用；内部自带 50ms 限流，可以无脑调 */
void rgb_led_tick(void);

#endif /* RGB_LED_H */
