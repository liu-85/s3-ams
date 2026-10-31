/**
 * rgb_led.cpp —— 板载 WS2812 RGB 状态灯实现（v1.1.29）
 *
 * 见 rgb_led.h 的模块说明。
 */

#include "rgb_led.h"
#include "board_pins.h"

/* ==========================================================================
 * 状态
 * ========================================================================== */
static led_mode_t s_mode = LED_MODE_BOOT;
static uint32_t   s_print_color = 0x22CC66;   /* 默认一个柔和的绿，等真实通道色覆盖 */
static uint32_t   s_last_tick_ms = 0;
static uint8_t    s_phase = 0;                /* 每 50ms +1，用来算呼吸/闪烁 */
static uint8_t    s_bright = 0;               /* 当前亮度 0~255（调试用） */

/* 灯整体亮度上限（0~255）。
 * WS2812 很亮，全亮度夜间刺眼；0x60(≈38%) 在室内足够看清，
 * 呼吸/闪烁的对比也还在。要更亮就调这里。 */
#define LED_MASTER_BRIGHT 0x60

/* ==========================================================================
 * 内部
 * ========================================================================== */

/* ★ v1.1.29：只在颜色真的变了才写 WS2812
 *
 * 为什么必须加：neopixelWrite() 走 RMT，每次同步等待约 30us。
 * 485 是 1.2288Mbps ≈ 154 字节/ms，30us 会吃掉 ~3.7 字节 ——
 * 如果正好落在收帧中间就是丢字节 → CRC 错 → 丢帧 → 40A3。
 * 呼吸灯每 100ms 才变一次，所以实际写入频率只有 10Hz，
 * 平均占用 0.03%，不会干扰 485。 */
static uint8_t s_last_r = 255, s_last_g = 255, s_last_b = 255;

static void led_write(uint8_t r, uint8_t g, uint8_t b)
{
    if (r == s_last_r && g == s_last_g && b == s_last_b) return;  /* 没变就不写 */
    s_last_r = r; s_last_g = g; s_last_b = b;
    neopixelWrite((uint8_t)BOARD_PIN_LED, r, g, b);
}

/* 按主亮度缩放并输出 */
static void led_put(uint8_t r, uint8_t g, uint8_t b)
{
    /* 先把颜色压到主亮度，再乘上当前呼吸系数 */
    uint32_t rr = (uint32_t)r * LED_MASTER_BRIGHT * s_bright / (255u * 255u);
    uint32_t gg = (uint32_t)g * LED_MASTER_BRIGHT * s_bright / (255u * 255u);
    uint32_t bb = (uint32_t)b * LED_MASTER_BRIGHT * s_bright / (255u * 255u);
    led_write((uint8_t)rr, (uint8_t)gg, (uint8_t)bb);
}

/* 呼吸曲线：0 → 255 → 0，一个来回 32 拍（100ms × 32 ≈ 3.2 秒） */
static uint8_t breathe_level(uint8_t phase)
{
    const uint8_t t = phase & 0x1F;                          /* 0..31 */
    const uint8_t v = (t < 16) ? (t * 16) : ((31 - t) * 16); /* 0..240 */
    return v;
}

/* 慢闪：亮 1 秒 / 灭 1 秒（100ms × 20） */
static bool blink_slow(uint8_t phase) { return (phase % 20) < 10; }

/* 快闪：亮 200ms / 灭 200ms（100ms × 4） */
static bool blink_fast(uint8_t phase) { return (phase % 4) < 2; }

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

void rgb_led_init(void)
{
    /* neopixelWrite 会自己配置引脚，先写一次全黑把残留颜色清掉 */
    neopixelWrite((uint8_t)BOARD_PIN_LED, 0, 0, 0);
    s_mode = LED_MODE_BOOT;
    s_phase = 0;
    s_bright = 0;
    s_last_tick_ms = millis();
}

void rgb_led_set_mode(led_mode_t m)
{
    if (s_mode == m) return;   /* 同状态不重置相位，呼吸才连续 */
    s_mode = m;
    s_phase = 0;               /* 换状态时重置相位，闪烁从"亮"开始 */
}

led_mode_t rgb_led_get_mode(void)
{
    return s_mode;
}

void rgb_led_set_print_color(uint32_t rgb888)
{
    s_print_color = rgb888 & 0xFFFFFFu;
}

void rgb_led_tick(void)
{
    const uint32_t now = millis();
    /* 100ms 限流：10fps。呼吸效果够顺，而 WS2812 每次写入要 30us，
     * 100ms 一次对 485 的占用只有 0.03%（见 led_write 注释）。 */
    if (now - s_last_tick_ms < 100) return;
    s_last_tick_ms = now;
    s_phase++;

    switch (s_mode) {
    case LED_MODE_BOOT:
        /* 蓝色慢闪：正在初始化 */
        s_bright = blink_slow(s_phase) ? 255 : 0;
        led_put(0, 60, 255);
        break;

    case LED_MODE_WIFI_WAIT:
        /* 黄色慢闪：没连上 WiFi（网页打不开，只能看灯） */
        s_bright = blink_slow(s_phase) ? 255 : 0;
        led_put(255, 180, 0);
        break;

    case LED_MODE_IDLE:
        /* 绿色低亮常亮：就绪，等打印机指令 */
        s_bright = 90;
        led_put(0, 255, 80);
        break;

    case LED_MODE_PRINTING: {
        /* 耗材颜色呼吸：一眼能看出当前用哪一卷 */
        s_bright = breathe_level(s_phase);
        const uint8_t r = (uint8_t)((s_print_color >> 16) & 0xFF);
        const uint8_t g = (uint8_t)((s_print_color >> 8) & 0xFF);
        const uint8_t b = (uint8_t)(s_print_color & 0xFF);
        led_put(r, g, b);
        break;
    }

    case LED_MODE_PULLBACK:
        /* 橙色呼吸：正在退料 */
        s_bright = breathe_level(s_phase);
        led_put(255, 90, 0);
        break;

    case LED_MODE_ERROR:
        /* 红色快闪：故障 */
        s_bright = blink_fast(s_phase) ? 255 : 0;
        led_put(255, 0, 0);
        break;

    default:
        s_bright = 0;
        led_put(0, 0, 0);
        break;
    }
}
