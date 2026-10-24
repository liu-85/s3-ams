/**
 * filament_sensor.cpp —— 微动开关（每路 2 个，电阻编码 + ADC 单脚读取）
 *
 * 从 esp32s3-485/main/filament_sensor.c 移植到 Arduino 框架。
 *
 * 核心逻辑：
 *   - 5ms 轮询定时器读取所有 ADC 脚
 *   - 每路独立去抖（连续 3 次一致才认可）
 *   - 三档电压分类：START / MID / STOP
 *   - ADC2 降级处理（WiFi 占用时 GPIO11~13 读不到）
 */

#include "filament_sensor.h"
#include "board_pins.h"

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ==========================================================================
 * 内部状态
 * ========================================================================== */

/* 引脚数组 */
static const int s_pins[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_ARRAY;

/* ADC 通道和单元 */
static adc_channel_t s_adc_ch[BOARD_CHANNEL_COUNT];
static int s_adc_unit[BOARD_CHANNEL_COUNT];  /* 1=ADC1, 2=ADC2 */

/* 校准句柄 */
static esp_adc_cal_characteristics_t s_adc_chars[BOARD_CHANNEL_COUNT];

/* 原始电压（mV），-1 = 无 */
static int s_mv[BOARD_CHANNEL_COUNT];

/* 分类结果：0=STOP, 1=MID, 2=START（用整数避免 enum 麻烦） */
#define COMBO_STOP  0
#define COMBO_MID   1
#define COMBO_START 2
static int s_combo[BOARD_CHANNEL_COUNT];

/* 降级标记（ADC2 + WiFi 开） */
static bool s_degraded[BOARD_CHANNEL_COUNT];

/* 去抖计数 */
static int s_counts[BOARD_CHANNEL_COUNT];  /* 连续相同读数计数 */
static int s_last_raw_combo[BOARD_CHANNEL_COUNT];  /* 上次原始分类 */

/* 边沿序号 */
static uint32_t s_trigger_seq[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];
static uint32_t s_release_seq[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];
static uint32_t s_last_trigger_ms[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];
static uint32_t s_last_release_ms[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];

/* 稳定状态（去抖后） */
static bool s_triggered[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];

/* 通道启用掩码 */
static uint8_t s_enabled_mask = 0xFF;  /* 默认全启用 */

/* 轮询定时器 */
static esp_timer_handle_t s_poll_timer = NULL;

/* ==========================================================================
 * ADC 初始化
 * ========================================================================== */

static void sensor_adc_init(void)
{
    /* 配置 ADC 宽度和衰减 */
    adc1_config_width(ADC_WIDTH_BIT_12);

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int pin = s_pins[i];
        adc_channel_t ch;
        int unit = 0;

        /* 将 GPIO 映射到 ADC 通道 */
        if (pin >= 1 && pin <= 10) {
            /* ADC1: GPIO1~GPIO10 */
            static const adc1_channel_t adc1_map[] = {
                ADC1_CHANNEL_0, ADC1_CHANNEL_1, ADC1_CHANNEL_2, ADC1_CHANNEL_3,
                ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7,
                ADC1_CHANNEL_8, ADC1_CHANNEL_9
            };
            ch = (adc_channel_t)adc1_map[pin - 1];
            unit = 1;
            adc1_config_channel_atten((adc1_channel_t)ch, ADC_ATTEN_DB_11);
        } else if (pin >= 11 && pin <= 20) {
            /* ADC2: GPIO11~GPIO20 */
            static const adc2_channel_t adc2_map[] = {
                ADC2_CHANNEL_0, ADC2_CHANNEL_1, ADC2_CHANNEL_2, ADC2_CHANNEL_3,
                ADC2_CHANNEL_4, ADC2_CHANNEL_5, ADC2_CHANNEL_6, ADC2_CHANNEL_7,
                ADC2_CHANNEL_8, ADC2_CHANNEL_9
            };
            ch = (adc_channel_t)adc2_map[pin - 11];
            unit = 2;
            adc2_config_channel_atten((adc2_channel_t)ch, ADC_ATTEN_DB_11);
        } else {
            s_mv[i] = -1;
            s_adc_unit[i] = 0;
            continue;
        }

        s_adc_ch[i] = ch;
        s_adc_unit[i] = unit;

        /* 校准 */
        esp_adc_cal_characterize(
            (unit == 1) ? ADC_UNIT_1 : ADC_UNIT_2,
            ADC_ATTEN_DB_11,
            ADC_WIDTH_BIT_12,
            1100,  /* 默认 Vref */
            &s_adc_chars[i]
        );
    }
}

/* ==========================================================================
 * 读取单路 ADC 电压
 * ========================================================================== */

static int sensor_read_mv(int i)
{
    if (s_adc_unit[i] == 0) return -1;

    int raw = 0;
    esp_err_t err;

    if (s_adc_unit[i] == 1) {
        /* adc1_get_raw 直接返回原始值，无 out 参数 */
        raw = adc1_get_raw((adc1_channel_t)s_adc_ch[i]);
        err = ESP_OK;
    } else {
        /* ADC2 可能在 WiFi 占用时失败 */
        err = adc2_get_raw((adc2_channel_t)s_adc_ch[i], ADC_WIDTH_BIT_12, &raw);
        if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
            /* WiFi 占用，降级 */
            s_degraded[i] = true;
            return -1;
        }
    }

    if (err != ESP_OK) return -1;

    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &s_adc_chars[i]);
    return (int)mv;
}

/* ==========================================================================
 * 三档分类
 * ========================================================================== */

static int sensor_classify(int mv)
{
    if (mv < 0) return COMBO_MID;  /* 读不到时保持中间 */
    if (mv < BOARD_SENSOR_MV_START) return COMBO_START;
    if (mv > BOARD_SENSOR_MV_STOP) return COMBO_STOP;
    return COMBO_MID;
}

/* ==========================================================================
 * 轮询回调
 * ========================================================================== */

static void sensor_poll_cb(void *arg)
{
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (s_adc_unit[i] == 0) continue;

        int mv = sensor_read_mv(i);
        s_mv[i] = mv;

        int combo = sensor_classify(mv);
        s_combo[i] = combo;

        /* 去抖 */
        if (combo != s_last_raw_combo[i]) {
            s_counts[i] = 0;
            s_last_raw_combo[i] = combo;
        } else {
            if (s_counts[i] < SENSOR_DEBOUNCE_N) s_counts[i]++;
        }

        if (s_counts[i] >= SENSOR_DEBOUNCE_N) {
            /* 稳定分类 → 更新触发状态 */
            bool start_trig = (combo == COMBO_START);
            bool stop_trig  = (combo == COMBO_STOP);

            /* START 微动：开始位压下 → 触发 */
            if (start_trig != s_triggered[i][SENSOR_START]) {
                s_triggered[i][SENSOR_START] = start_trig;
                if (start_trig) {
                    s_trigger_seq[i][SENSOR_START]++;
                    s_last_trigger_ms[i][SENSOR_START] = millis();
                } else {
                    s_release_seq[i][SENSOR_START]++;
                    s_last_release_ms[i][SENSOR_START] = millis();
                }
            }

            /* STOP 微动：停止位顶开 → 触发（注意 STOP 是"顶开=触发"语义） */
            if (stop_trig != s_triggered[i][SENSOR_STOP]) {
                s_triggered[i][SENSOR_STOP] = stop_trig;
                if (stop_trig) {
                    s_trigger_seq[i][SENSOR_STOP]++;
                    s_last_trigger_ms[i][SENSOR_STOP] = millis();
                } else {
                    s_release_seq[i][SENSOR_STOP]++;
                    s_last_release_ms[i][SENSOR_STOP] = millis();
                }
            }
        }

        /* 恢复降级标记（如果这次读到了） */
        if (mv >= 0 && s_degraded[i]) {
            s_degraded[i] = false;
        }
    }
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

bool sensor_init(void)
{
    sensor_adc_init();

    /* 清零状态 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        s_mv[i] = -1;
        s_combo[i] = COMBO_MID;
        s_counts[i] = 0;
        s_last_raw_combo[i] = COMBO_MID;
        s_degraded[i] = false;
        for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
            s_triggered[i][k] = false;
            s_trigger_seq[i][k] = 0;
            s_release_seq[i][k] = 0;
            s_last_trigger_ms[i][k] = UINT32_MAX;
            s_last_release_ms[i][k] = UINT32_MAX;
        }
    }

    /* 创建 5ms 周期定时器 */
    esp_timer_create_args_t timer_args = {
        .callback = sensor_poll_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sensor_poll"
    };
    esp_err_t err = esp_timer_create(&timer_args, &s_poll_timer);
    if (err != ESP_OK) {
        log_e("微动轮询定时器创建失败: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_timer_start_periodic(s_poll_timer, SENSOR_POLL_MS * 1000);
    if (err != ESP_OK) {
        log_e("微动轮询定时器启动失败: %s", esp_err_to_name(err));
        return false;
    }

    log_i("%d 路微动已初始化（单IO双微动电阻编码 + ADC）", BOARD_CHANNEL_COUNT);
    return true;
}

/* ==========================================================================
 * 查询
 * ========================================================================== */

bool sensor_triggered(int channel, sensor_kind_t kind)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) return false;
    if (!(s_enabled_mask & (1u << channel))) return false;
    return s_triggered[channel][kind];
}

int sensor_combo_mv(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return -1;
    return s_mv[channel];
}

int sensor_combo_adc_unit(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return 0;
    return s_adc_unit[channel];
}

bool sensor_combo_degraded(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    return s_degraded[channel];
}

bool sensor_channel_enabled(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    return (s_enabled_mask & (1u << channel)) != 0;
}

bool sensor_set_channel_enabled(int channel, bool enabled)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    if (enabled) s_enabled_mask |= (1u << channel);
    else s_enabled_mask &= ~(1u << channel);
    return true;
}

uint8_t sensor_enabled_mask(void)
{
    return s_enabled_mask;
}

uint32_t sensor_trigger_seq(int channel, sensor_kind_t kind)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return 0;
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) return 0;
    return s_trigger_seq[channel][kind];
}

uint32_t sensor_release_seq(int channel, sensor_kind_t kind)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return 0;
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) return 0;
    return s_release_seq[channel][kind];
}

bool sensor_wait_trigger(int channel, sensor_kind_t kind, uint32_t timeout_ms)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) return false;

    uint32_t start = s_trigger_seq[channel][kind];
    uint32_t begin = millis();

    /* 已经触发则立即返回 */
    if (s_triggered[channel][kind]) return true;
    if (timeout_ms == 0) return false;

    while (millis() - begin < timeout_ms) {
        if (s_trigger_seq[channel][kind] != start) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

bool sensor_wait_release(int channel, sensor_kind_t kind, uint32_t timeout_ms)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return false;
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) return false;

    uint32_t start = s_release_seq[channel][kind];
    uint32_t begin = millis();

    if (!s_triggered[channel][kind]) return true;
    if (timeout_ms == 0) return false;

    while (millis() - begin < timeout_ms) {
        if (s_release_seq[channel][kind] != start) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

int sensor_describe_json(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) return 0;
    int off = snprintf(buf, buflen,
        "{\"enabled\":%u,\"ch\":[", (unsigned)s_enabled_mask);
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int stop = s_triggered[i][SENSOR_STOP] ? 1 : 0;
        int start = s_triggered[i][SENSOR_START] ? 1 : 0;
        off += snprintf(buf + off, buflen - off, "%s[%d,%d]",
                        i ? "," : "", stop, start);
    }
    off += snprintf(buf + off, buflen - off, "]}");
    return off;
}
