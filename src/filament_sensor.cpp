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

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
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

/* 临时直连模式的独立去抖（每通道每种微动各一套）。
   正式模式用的是"每通道一个 combo"的去抖，两套互不干扰。 */
static int s_dbg_counts[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];
static int s_dbg_last[BOARD_CHANNEL_COUNT][SENSOR_KIND_COUNT];   /* -1 = 还没采过 */

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

/* ==========================================================================
 * 三档分类 + 稳定性滤波
 * ==========================================================================
 * 为什么要稳定性滤波（2026-10-25）：
 *   单 IO 双微动是电阻编码，微动没接/悬空时 ADC 读的是噪声。之前只加了
 *   "350mV 以下算无信号"的门限，但那只是通道 1 实测的 ~371mV —— 其他
 *   通道悬空电压各不相同（走线/漏电流/内部分压），有的落在 START 区
 *   (<433mV) 就乱跳，用户看到"只修了通道 1"。
 *   现在的判据是"稳不稳"而不是"多少 mV"：悬空引脚的读数会在窗口内大幅
 *   跳动，真实分压（开始位 ~300mV / 停止位 ~600mV+）是稳的。
 *   窗口内 (max-min) > SENSOR_STABLE_SPREAD_MV → 判中间态（无动作），
 *   全通道一视同仁，不依赖任何魔法电压。
 * ========================================================================== */
#define SENSOR_STABLE_WIN     8     /* 稳定性窗口：最近 8 次采样（8×5ms=40ms） */
#define SENSOR_STABLE_SPREAD_MV 100 /* 窗口内最大最小差超过此值 = 读数不稳 */

static int s_mv_win[BOARD_CHANNEL_COUNT][SENSOR_STABLE_WIN];
static int s_mv_win_idx[BOARD_CHANNEL_COUNT];
static int s_mv_win_cnt[BOARD_CHANNEL_COUNT];

/* 窗口推入一个新值，返回本次是否稳定（填满且极差足够小） */
static bool sensor_stable_push(int i, int mv)
{
    if (mv < 0) return false;  /* 读不到（ADC2 被 WiFi 占用等）→ 不稳 */
    int *w = s_mv_win[i];
    w[s_mv_win_idx[i]] = mv;
    s_mv_win_idx[i] = (s_mv_win_idx[i] + 1) % SENSOR_STABLE_WIN;
    if (s_mv_win_cnt[i] < SENSOR_STABLE_WIN) s_mv_win_cnt[i]++;

    if (s_mv_win_cnt[i] < SENSOR_STABLE_WIN) return false;  /* 没填满先不判 */

    int mn = w[0], mx = w[0];
    for (int k = 1; k < SENSOR_STABLE_WIN; k++) {
        if (w[k] < mn) mn = w[k];
        if (w[k] > mx) mx = w[k];
    }
    return (mx - mn) <= SENSOR_STABLE_SPREAD_MV;
}

static int sensor_classify(int mv)
{
    if (mv < 0) return COMBO_MID;  /* 读不到时保持中间 */
    /* ★ 无信号门限：微动未接/悬空时 ADC 读 ~371mV 噪声，正好落 START 区
       （371 < 433）导致 A1/A2 间隔误触发。低于 BOARD_SENSOR_MV_NOISE
       视为无动作（中间态），只有读到稳定的 START/STOP 分压才认。 */
    if (mv < BOARD_SENSOR_MV_NOISE) return COMBO_MID;
    if (mv < BOARD_SENSOR_MV_START) return COMBO_START;
    if (mv > BOARD_SENSOR_MV_STOP) return COMBO_STOP;
    return COMBO_MID;
}

/* ==========================================================================
 * 轮询回调
 * ========================================================================== */

/* ==========================================================================
 * 临时直连微动模式（SENSOR_TEMP_DIGITAL）
 *
 * 8 个微动 IO 各接一只普通微动，两两配对成 4 个通道的"开始位 / 停止位"。
 * 上层（AMCU / 缓冲补料 / 网页）看到的仍是 sensor_triggered(ch, START|STOP)，
 * 一行没改。
 * ========================================================================== */
#if SENSOR_TEMP_DIGITAL

/* 单个微动的去抖 + 边沿计数。raw=true 表示"压下"。 */
static void sensor_dbg_apply(int ch, sensor_kind_t kind, bool raw)
{
    int r = raw ? 1 : 0;
    if (r == s_dbg_last[ch][kind]) {
        if (s_dbg_counts[ch][kind] < SENSOR_DEBOUNCE_N) s_dbg_counts[ch][kind]++;
    } else {
        s_dbg_counts[ch][kind] = 0;
        s_dbg_last[ch][kind] = r;
    }

    /* 连续 N 次一致才认 */
    if (s_dbg_counts[ch][kind] < SENSOR_DEBOUNCE_N) return;
    if (raw == s_triggered[ch][kind]) return;

    s_triggered[ch][kind] = raw;
    if (raw) {
        s_trigger_seq[ch][kind]++;
        s_last_trigger_ms[ch][kind] = millis();
    } else {
        s_release_seq[ch][kind]++;
        s_last_release_ms[ch][kind] = millis();
    }
}

static void sensor_poll_digital(void)
{
    const int half = BOARD_CHANNEL_COUNT / 2;   /* 4：前一半是开始位，后一半是停止位 */
    bool pressed[BOARD_CHANNEL_COUNT];

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int lv = digitalRead(s_pins[i]);
#if SENSOR_TEMP_ACTIVE_LOW
        pressed[i] = (lv == LOW);
#else
        pressed[i] = (lv == HIGH);
#endif
        /* 给网页 /status 一个能看的数：压下 0mV，松开 3.3V。
           这不是真实 ADC 读数，只是让「微动」那一栏有意义。 */
        s_mv[i] = pressed[i] ? 0 : BOARD_SENSOR_VDD_MV;
        s_degraded[i] = false;   /* digitalRead 不受 ADC2/WiFi 影响 */
    }

    /* 通道 c：开始位 = 微动 c，停止位 = 微动 c + half */
    for (int c = 0; c < half; c++) {
        bool st = pressed[c];
        bool sp = pressed[c + half];
        sensor_dbg_apply(c, SENSOR_START, st);
        sensor_dbg_apply(c, SENSOR_STOP,  sp);
        /* combo 只用于网页显示 */
        s_combo[c] = st ? COMBO_START : (sp ? COMBO_STOP : COMBO_MID);
    }

    /* 后一半通道没有独立微动（它们的 IO 已被当成上面通道的停止位用了），
       固定清成"不触发"，避免误补料。 */
    for (int c = half; c < BOARD_CHANNEL_COUNT; c++) {
        sensor_dbg_apply(c, SENSOR_START, false);
        sensor_dbg_apply(c, SENSOR_STOP,  false);
        s_combo[c] = COMBO_MID;
    }
}

#endif /* SENSOR_TEMP_DIGITAL */

static void sensor_poll_cb(void *arg)
{
#if SENSOR_TEMP_DIGITAL
    (void)arg;
    sensor_poll_digital();
    return;
#else
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (s_adc_unit[i] == 0) continue;

        int mv = sensor_read_mv(i);
        s_mv[i] = mv;

        /* 稳定性滤波：读数在窗口内跳变过大（悬空/接触不良）→ 判中间态。
           全通道生效，不依赖具体噪声电压（其他通道悬空值跟通道 1 不同）。 */
        bool stable = sensor_stable_push(i, mv);
        int combo = stable ? sensor_classify(mv) : COMBO_MID;
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
#endif /* SENSOR_TEMP_DIGITAL */
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

bool sensor_init(void)
{
#if SENSOR_TEMP_DIGITAL
    /* 临时直连模式：8 个脚都当普通数字输入，开内部上拉。
       不碰 ADC —— GPIO11/12/13 是 ADC2，WiFi 开着时 ADC 读不到。 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        pinMode(s_pins[i], INPUT_PULLUP);
        s_adc_unit[i] = 1;      /* 标记"可读"，让降级判断不挡 */
    }
    /* 后 4 个 IO 已经被当成前 4 通道的停止位用掉了，它们不是独立料盘位。
       关掉这 4 个"通道"的缓冲补料，免得真去驱动不存在的通道。 */
    s_enabled_mask &= (uint8_t)((1u << (BOARD_CHANNEL_COUNT / 2)) - 1u);
#else
    sensor_adc_init();
#endif

    /* 清零状态 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        s_mv[i] = -1;
        s_combo[i] = COMBO_MID;
        s_counts[i] = 0;
        s_last_raw_combo[i] = COMBO_MID;
        s_degraded[i] = false;
        s_mv_win_idx[i] = 0;
        s_mv_win_cnt[i] = 0;
        for (int k = 0; k < SENSOR_STABLE_WIN; k++) s_mv_win[i][k] = 0;
        for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
            s_triggered[i][k] = false;
            s_trigger_seq[i][k] = 0;
            s_release_seq[i][k] = 0;
            s_last_trigger_ms[i][k] = UINT32_MAX;
            s_last_release_ms[i][k] = UINT32_MAX;
            s_dbg_counts[i][k] = 0;
            s_dbg_last[i][k] = -1;
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

#if SENSOR_TEMP_DIGITAL
    log_w("微动: ★临时直连模式★ 8 只独立微动 → 4 通道（后 4 通道已禁用）");
    log_w("  极性=%s", SENSOR_TEMP_ACTIVE_LOW ? "低电平触发(微动接 GND)" : "高电平触发(微动接 3.3V)");
    for (int c = 0; c < BOARD_CHANNEL_COUNT / 2; c++) {
        log_w("  通道%d: 开始=微动%d(GPIO%d)  停止=微动%d(GPIO%d)",
              c + 1, c + 1, s_pins[c], c + 5, s_pins[c + BOARD_CHANNEL_COUNT / 2]);
    }
    log_w("  电阻编码微动到货后把 SENSOR_TEMP_DIGITAL 改回 0 重新编译");
#else
    log_i("%d 路微动已初始化（单IO双微动电阻编码 + ADC）", BOARD_CHANNEL_COUNT);
#endif
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

/* ★ v1.1.24：原始电平诊断。
 *
 * 现场（2026-10-31）：通道 2~4 的微动在 WEB 上完全不变，只有通道 1 会动。
 * 光看 /status 的 ch[][] 分不清是「引脚没信号」还是「有信号但被掩码/去抖挡」，
 * 所以直接把 digitalRead 的原始值暴露出来：
 *   raw=1 且按下不变  → 接线/微动本身问题（常开没接通、GND 没共）
 *   raw=0 且松开不变  → 引脚被拉死（常闭微动、短路、或被别处占用）
 *   raw 会变但 ch 不变 → 软件问题（掩码 / 去抖 / 通道映射） */
int sensor_raw_level(int idx)
{
    if (idx < 0 || idx >= BOARD_CHANNEL_COUNT) return -1;
    return (digitalRead(s_pins[idx]) == HIGH) ? 1 : 0;
}

int sensor_pin_of(int idx)
{
    if (idx < 0 || idx >= BOARD_CHANNEL_COUNT) return -1;
    return s_pins[idx];
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
