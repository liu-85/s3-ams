/**
 * current_sense.cpp —— 电机电流采样 + 堵转保护（AT8236 ISEN · GPIO3）
 *
 * 实现要点：
 *   - ADC1_CHANNEL_2（GPIO3），12bit / 11dB 衰减，esp_adc_cal 校准
 *   - esp_timer 10ms 周期采样（ESP_TIMER_TASK，不占主循环）
 *   - IIR 滤波（系数 1/8）去 PWM 纹波；1s（100 点）滑动窗口记峰值
 *   - 堵转判定只在"电机确实在转"时进行；起步 blank_ms 内不判
 *     （直流电机起动电流是稳态 3~8 倍，不屏蔽必然误跳）
 *   - 跳闸动作通过回调执行（AMCU 侧停电机 + 异步断离合），
 *     本模块不直接依赖 clutch/AMCU，避免循环包含
 */

#include "current_sense.h"
#include "board_pins.h"
#include "motor.h"

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>

/* ==========================================================================
 * 参数
 * ========================================================================== */
#define CURRENT_POLL_MS          10      /* 采样周期 */
#define CURRENT_FILTER_SHIFT     3       /* IIR：avg += (mv-avg)/8 */
#define CURRENT_PEAK_WINDOW      100     /* 峰值窗口点数（10ms × 100 = 1s） */
#define CURRENT_ADC_VREF_MV      1100    /* 校准用默认 Vref */

/* ==========================================================================
 * 状态
 * ========================================================================== */
static bool     s_ready = false;
static bool     s_init_done = false;

/* 配置（NVS 持久化） */
static int      s_r_mohm   = CURRENT_DEFAULT_R_MOHM;
static int      s_stall_ma = CURRENT_DEFAULT_STALL_MA;
static int      s_stall_ms = CURRENT_DEFAULT_STALL_MS;
static int      s_blank_ms = CURRENT_DEFAULT_BLANK_MS;

/* 采样值 */
static int      s_mv_raw = -1;            /* 最近一次原始 mV（未滤波） */
static int      s_mv_filt = 0;            /* IIR 滤波 mV */
static bool     s_filt_primed = false;    /* IIR 是否已入驻 */
static int      s_peak_ring[CURRENT_PEAK_WINDOW];
static int      s_peak_idx = 0;
static int      s_peak_mv = 0;            /* 窗口内峰值 mV */

/* 堵转 */
static bool     s_tripped = false;
static uint32_t s_trips = 0;
static uint32_t s_over_ms = 0;            /* 当前连续超阈时长 */
static uint32_t s_run_started_ms = 0;     /* 本次电机启动时刻（0 = 未运行） */
static motor_dir_t s_last_dir = MOTOR_DIR_STOP;
static void (*s_stall_cb)(void) = NULL;

/* 定时器 */
static esp_timer_handle_t s_timer = NULL;

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* mV → mA：V = I × R  →  I(mA) = V(mV) / R(mΩ) × 1000 */
static int mv_to_ma(int mv)
{
    if (mv < 0 || s_r_mohm <= 0) return -1;
    return (int)((int64_t)mv * 1000 / s_r_mohm);
}

static void current_config_load(void)
{
    Preferences prefs;
    prefs.begin("s3ams-cur", true);
    s_r_mohm   = prefs.getInt("r_mohm",   CURRENT_DEFAULT_R_MOHM);
    s_stall_ma = prefs.getInt("stall_ma", CURRENT_DEFAULT_STALL_MA);
    s_stall_ms = prefs.getInt("stall_ms", CURRENT_DEFAULT_STALL_MS);
    s_blank_ms = prefs.getInt("blank_ms", CURRENT_DEFAULT_BLANK_MS);
    prefs.end();

    s_r_mohm   = clamp_int(s_r_mohm,   CURRENT_R_MOHM_MIN,   CURRENT_R_MOHM_MAX);
    s_stall_ma = clamp_int(s_stall_ma, CURRENT_STALL_MA_MIN, CURRENT_STALL_MA_MAX);
    s_stall_ms = clamp_int(s_stall_ms, CURRENT_STALL_MS_MIN, CURRENT_STALL_MS_MAX);
    s_blank_ms = clamp_int(s_blank_ms, CURRENT_BLANK_MS_MIN, CURRENT_BLANK_MS_MAX);
}

static void current_config_save(void)
{
    Preferences prefs;
    prefs.begin("s3ams-cur", false);
    prefs.putInt("r_mohm",   s_r_mohm);
    prefs.putInt("stall_ma", s_stall_ma);
    prefs.putInt("stall_ms", s_stall_ms);
    prefs.putInt("blank_ms", s_blank_ms);
    prefs.end();
}

/* 校准句柄 */
static esp_adc_cal_characteristics_t s_adc_chars;

/* ==========================================================================
 * 采样回调（10ms）
 * ========================================================================== */
static void current_poll_cb(void *arg)
{
    if (!s_ready) return;

    /* 1) 读 ADC（ADC1，WiFi 开启也不受影响） */
    int raw = adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_2);
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &s_adc_chars);
    s_mv_raw = (int)mv;

    /* 2) IIR 滤波（首拍直接入驻，避免从 0 爬升） */
    if (!s_filt_primed) {
        s_mv_filt = (int)mv;
        s_filt_primed = true;
    } else {
        s_mv_filt += ((int)mv - s_mv_filt) >> CURRENT_FILTER_SHIFT;
    }

    /* 3) 1s 滑动峰值窗口 */
    s_peak_ring[s_peak_idx] = s_mv_filt;
    s_peak_idx = (s_peak_idx + 1) % CURRENT_PEAK_WINDOW;
    int peak = 0;
    for (int i = 0; i < CURRENT_PEAK_WINDOW; i++) {
        if (s_peak_ring[i] > peak) peak = s_peak_ring[i];
    }
    s_peak_mv = peak;

    /* 4) 堵转判定 */
    motor_dir_t dir = motor_get_dir();
    uint32_t now = millis();

    if (dir == MOTOR_DIR_STOP) {
        /* 电机没转：清零运行期状态 */
        s_run_started_ms = 0;
        s_over_ms = 0;
        s_last_dir = MOTOR_DIR_STOP;
        return;
    }

    if (s_last_dir == MOTOR_DIR_STOP || s_run_started_ms == 0) {
        /* 上升沿：记录启动时刻，进入起步屏蔽 */
        s_run_started_ms = now;
        s_over_ms = 0;
    }
    s_last_dir = dir;

    if (s_stall_ma <= 0) return;               /* 0 = 只显示、不保护 */
    if (s_tripped) return;                     /* 已 latch，等清除 */
    if (now - s_run_started_ms < (uint32_t)s_blank_ms) return;  /* 起步屏蔽 */

    int ma = mv_to_ma(s_mv_filt);
    if (ma >= s_stall_ma) {
        s_over_ms += CURRENT_POLL_MS;
        if (s_over_ms >= (uint32_t)s_stall_ms) {
            /* 跳闸：latch + 计数 + 日志 + 回调（停电机/断离合） */
            s_tripped = true;
            s_trips++;
            log_e("堵转保护跳闸：电流 %dmA ≥ 阈值 %dmA 持续 %ums（第 %u 次）→ 已停电机",
                  ma, s_stall_ma, (unsigned)s_over_ms, (unsigned)s_trips);
            s_over_ms = 0;
            if (s_stall_cb) s_stall_cb();
        }
    } else {
        s_over_ms = 0;
    }
}

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

bool current_sense_init(void)
{
    if (s_init_done) return s_ready;
    s_init_done = true;

    if (!BOARD_CURRENT_SENSE_ENABLED || BOARD_PIN_CURRENT_SENSE < 0) {
        log_w("电流采样：本板型未启用（BOARD_CURRENT_SENSE_ENABLED=0）");
        return false;
    }

    /* GPIO3 = ADC1_CHANNEL_2 */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                             CURRENT_ADC_VREF_MV, &s_adc_chars);

    current_config_load();

    for (int i = 0; i < CURRENT_PEAK_WINDOW; i++) s_peak_ring[i] = 0;

    esp_timer_create_args_t args = {
        .callback = current_poll_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "current_sense"
    };
    esp_err_t err = esp_timer_create(&args, &s_timer);
    if (err != ESP_OK) {
        log_e("电流采样定时器创建失败: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_timer_start_periodic(s_timer, CURRENT_POLL_MS * 1000);
    if (err != ESP_OK) {
        log_e("电流采样定时器启动失败: %s", esp_err_to_name(err));
        return false;
    }

    s_ready = true;
    log_i("电机电流采样已启用: ISEN=GPIO%d (ADC1_CH2) · R=%dmΩ (%dmV/A) · "
          "堵转阈值 %dmA/%ums · 起步屏蔽 %ums%s",
          BOARD_PIN_CURRENT_SENSE, s_r_mohm, s_r_mohm,
          s_stall_ma, (unsigned)s_stall_ms, (unsigned)s_blank_ms,
          s_stall_ma <= 0 ? "（阈值 0 = 只显示不保护，实测后再填）" : "");
    return true;
}

bool current_sense_ready(void)   { return s_ready; }
bool current_sense_enabled(void) { return s_ready; }

int current_sense_mv(void)  { return s_ready ? s_mv_filt : -1; }
int current_sense_ma(void)  { return s_ready ? mv_to_ma(s_mv_filt) : -1; }
int current_sense_peak_mv(void) { return s_ready ? s_peak_mv : -1; }
int current_sense_peak_ma(void) { return s_ready ? mv_to_ma(s_peak_mv) : -1; }

bool current_sense_tripped(void)    { return s_tripped; }
uint32_t current_sense_trips(void)  { return s_trips; }

int current_sense_r_mohm(void)   { return s_r_mohm; }
int current_sense_stall_ma(void) { return s_stall_ma; }
int current_sense_stall_ms(void) { return s_stall_ms; }
int current_sense_blank_ms(void) { return s_blank_ms; }

void current_sense_set_config(int r_mohm, int stall_ma, int stall_ms, int blank_ms)
{
    s_r_mohm   = clamp_int(r_mohm,   CURRENT_R_MOHM_MIN,   CURRENT_R_MOHM_MAX);
    s_stall_ma = clamp_int(stall_ma, CURRENT_STALL_MA_MIN, CURRENT_STALL_MA_MAX);
    s_stall_ms = clamp_int(stall_ms, CURRENT_STALL_MS_MIN, CURRENT_STALL_MS_MAX);
    s_blank_ms = clamp_int(blank_ms, CURRENT_BLANK_MS_MIN, CURRENT_BLANK_MS_MAX);
    current_config_save();
    s_over_ms = 0;
    log_i("电流采样配置已更新: R=%dmΩ 阈值=%dmA/%ums 屏蔽=%ums",
          s_r_mohm, s_stall_ma, (unsigned)s_stall_ms, (unsigned)s_blank_ms);
}

void current_sense_reset_defaults(void)
{
    current_sense_set_config(CURRENT_DEFAULT_R_MOHM, CURRENT_DEFAULT_STALL_MA,
                             CURRENT_DEFAULT_STALL_MS, CURRENT_DEFAULT_BLANK_MS);
}

void current_sense_clear(void)
{
    s_tripped = false;
    s_over_ms = 0;
    for (int i = 0; i < CURRENT_PEAK_WINDOW; i++) s_peak_ring[i] = 0;
    s_peak_mv = 0;
    log_i("电流采样：已清除跳闸 latch 与峰值");
}

void current_sense_set_stall_cb(void (*cb)(void))
{
    s_stall_cb = cb;
}
