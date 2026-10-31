/**
 * motor.cpp —— 共享直流电机驱动（LEDC PWM 两路输出）
 *
 * 从 esp32s3-485/main/motor.c 移植到 Arduino 框架。
 * 用两个 LEDC 通道分别驱动 H 桥的 IN1 / IN2：
 *     方向       IN1 占空比        IN2 占空比
 *     进料       speed            0
 *     退料       0                speed
 *     停止       0                0
 */

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
#include "motor.h"
#include "board_pins.h"
#include "current_sense.h"

#include <driver/ledc.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* ==========================================================================
 * 硬件参数
 * ========================================================================== */
#define MOTOR_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER      LEDC_TIMER_0
#define MOTOR_LEDC_CH_IN1     LEDC_CHANNEL_0
#define MOTOR_LEDC_CH_IN2     LEDC_CHANNEL_1
#define MOTOR_LEDC_RES        LEDC_TIMER_8_BIT   /* 占空比 0~255 */
#define MOTOR_LEDC_FREQ_HZ    20000              /* 20kHz，超出人耳听阈 */

#define MOTOR_DUTY_MAX        255
#define MOTOR_DEFAULT_DEAD_MS 30

/* ==========================================================================
 * 运行状态
 * ========================================================================== */
static motor_dir_t s_dir = MOTOR_DIR_STOP;
static int         s_speed_pct = MOTOR_SPEED_FULL;
static uint32_t    s_dead_time_ms = MOTOR_DEFAULT_DEAD_MS;
static int64_t     s_dir_started_us = 0;

/* ==========================================================================
 * v1.1.26：单次运行电流剖析
 * ==========================================================================
 * 目的：给"堵转阈值"提供实测依据。一次运行结束后打印
 *   启动峰值 / 稳态均值 / 稳态峰值 / 全程最大
 * 现场用法：
 *   1) 正常送料一次 → 记下"稳态均值"和"稳态峰值"
 *   2) 人为堵住料（手捏住耗材）→ 记下"稳态峰值"
 *   3) 堵转阈值取 (2) 的 70% 左右，且明显高于 (1) 的稳态峰值
 *
 * 采样周期 20ms：起动电流峰值通常持续 50~200ms，20ms 能捕捉到。
 * 只对 duration_ms >= MOTOR_PROFILE_MIN_MS 的运行打日志 —— 跟随送料是
 * 250ms 脉冲，不打印，避免把网页日志刷爆。 */
#define MOTOR_PROFILE_PERIOD_US   20000   /* 20ms 采样一次 */
#define MOTOR_PROFILE_MIN_MS      500     /* 短于此的运行不打日志 */
#define MOTOR_PROFILE_START_MS    800     /* 启动段时长（与堵转保护的 blank_ms 同量级） */

typedef struct {
    bool     active;        /* 正在统计 */
    motor_dir_t dir;
    int      pct;
    int64_t  start_us;
    int      start_peak_ma;
    int      max_ma;
    int64_t  run_sum_ma;
    uint32_t run_n;
    int      run_peak_ma;
} motor_profile_run_t;

static motor_profile_run_t s_prof = {0};
static motor_current_stat_t s_last_stat = {0};
static esp_timer_handle_t s_prof_timer = NULL;

/* 20ms 采样：分「启动段」和「稳态段」两段统计
 *
 * 峰值用**原始值**（current_sense_raw_ma），稳态均值用**滤波值**：
 *   - 启动电流尖峰只持续 50~200ms，IIR(shift=3) 时间常数约 80ms，
 *     用滤波值会把峰值削掉一大截 → 堵转阈值会被定得过低 → 误跳闸
 *   - 稳态均值要的是"典型值"，用滤波值才不被噪声带偏 */
static void motor_profile_cb(void *arg)
{
    (void)arg;
    if (!s_prof.active) return;

    const int ma_raw  = current_sense_raw_ma();   /* 峰值用 */
    const int ma_filt = current_sense_ma();       /* 均值/稳态峰值用 */

    if (ma_raw > s_prof.max_ma) s_prof.max_ma = ma_raw;

    const int64_t elapsed_us = esp_timer_get_time() - s_prof.start_us;
    const int64_t start_us_limit = (int64_t)MOTOR_PROFILE_START_MS * 1000;

    if (elapsed_us < start_us_limit) {
        if (ma_raw > s_prof.start_peak_ma) s_prof.start_peak_ma = ma_raw;
    } else if (ma_filt >= 0) {          /* -1 = 采样不可用，别污染均值 */
        s_prof.run_sum_ma += ma_filt;
        s_prof.run_n++;
        if (ma_filt > s_prof.run_peak_ma) s_prof.run_peak_ma = ma_filt;
    }
}

static void motor_profile_start(motor_dir_t dir, int pct)
{
    if (s_prof_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = motor_profile_cb,
            .arg      = NULL,
            .name     = "motor_profile",
        };
        if (esp_timer_create(&args, &s_prof_timer) != ESP_OK) {
            s_prof_timer = NULL;
            return;
        }
        esp_timer_start_periodic(s_prof_timer, MOTOR_PROFILE_PERIOD_US);
    }

    s_prof.active        = true;
    s_prof.dir           = dir;
    s_prof.pct           = pct;
    s_prof.start_us      = esp_timer_get_time();
    s_prof.start_peak_ma = 0;
    s_prof.max_ma        = 0;
    s_prof.run_sum_ma    = 0;
    s_prof.run_n         = 0;
    s_prof.run_peak_ma   = 0;
}

static void motor_profile_stop(void)
{
    if (!s_prof.active) return;
    s_prof.active = false;

    const uint32_t dur_ms = (uint32_t)((esp_timer_get_time() - s_prof.start_us) / 1000);
    const int run_avg = (s_prof.run_n > 0) ? (int)(s_prof.run_sum_ma / (int64_t)s_prof.run_n) : 0;

    /* 太短的运行（跟随送料的 250ms 脉冲）既不写日志、也不覆盖 /status 的
       "最近一次统计" —— 否则网页上永远只看到一串 250ms 的碎片数据。 */
    if (dur_ms < MOTOR_PROFILE_MIN_MS) return;

    s_last_stat.valid         = true;
    s_last_stat.dir           = s_prof.dir;
    s_last_stat.pct           = s_prof.pct;
    s_last_stat.duration_ms   = dur_ms;
    s_last_stat.start_peak_ma = s_prof.start_peak_ma;
    s_last_stat.run_avg_ma    = run_avg;
    s_last_stat.run_peak_ma   = s_prof.run_peak_ma;
    s_last_stat.max_ma        = s_prof.max_ma;

    log_i("电流统计: %s PWM=%d%% 时长=%lums 启动峰值=%dmA "
          "稳态均值=%dmA 稳态峰值=%dmA 全程最大=%dmA",
          (s_prof.dir == MOTOR_DIR_FEED) ? "进料" : "退料",
          s_prof.pct, (unsigned long)dur_ms,
          s_prof.start_peak_ma, run_avg, s_prof.run_peak_ma, s_prof.max_ma);
}

bool motor_get_current_stat(motor_current_stat_t *out)
{
    if (out == NULL || !s_last_stat.valid) return false;
    *out = s_last_stat;
    return true;
}

void motor_reset_current_stat(void)
{
    s_last_stat.valid = false;
    s_prof.active = false;
}

/* ==========================================================================
 * 内部函数
 * ========================================================================== */

uint32_t motor_pct_to_duty(int pct)
{
    if (pct <= 0) return 0;
    if (pct > 100) pct = 100;
    return (uint32_t)((pct * MOTOR_DUTY_MAX + 50) / 100);
}

/**
 * 直接写两路占空比。唯一碰硬件的地方。
 * 先双路清零（防 H 桥直通），再给目标通道赋值。
 */
static void motor_apply(int in1_pct, int in2_pct)
{
    /* 兜底：两路都非 0 属于逻辑错误，一律按刹车处理 */
    if (in1_pct != 0 && in2_pct != 0) {
        in1_pct = 0;
        in2_pct = 0;
    }

    const uint32_t d1 = motor_pct_to_duty(in1_pct);
    const uint32_t d2 = motor_pct_to_duty(in2_pct);

    /* 第一步：双路清零 */
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);

    /* 第二步：给目标通道赋值 */
    if (d1) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, d1);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    } else if (d2) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, d2);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    }
}

/* ==========================================================================
 * 公开接口
 * ========================================================================== */

bool motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_LEDC_RES,
        .timer_num       = MOTOR_LEDC_TIMER,
        .freq_hz         = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        log_e("电机 PWM 定时器配置失败: %s", esp_err_to_name(err));
        return false;
    }

    ledc_channel_config_t ch1 = {
        .gpio_num   = (gpio_num_t)BOARD_PIN_MOTOR_IN1,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEDC_CH_IN1,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch1);
    if (err != ESP_OK) {
        log_e("电机 IN1(GPIO%d) PWM 通道配置失败: %s", BOARD_PIN_MOTOR_IN1, esp_err_to_name(err));
        return false;
    }

    ledc_channel_config_t ch2 = {
        .gpio_num   = (gpio_num_t)BOARD_PIN_MOTOR_IN2,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEDC_CH_IN2,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch2);
    if (err != ESP_OK) {
        log_e("电机 IN2(GPIO%d) PWM 通道配置失败: %s", BOARD_PIN_MOTOR_IN2, esp_err_to_name(err));
        return false;
    }

    s_dir = MOTOR_DIR_STOP;
    s_speed_pct = MOTOR_SPEED_FULL;
    s_dir_started_us = 0;
    motor_apply(0, 0);

    log_i("共享电机已初始化: IN1=GPIO%d IN2=GPIO%d  PWM %dHz/%d位  换向死区=%ums",
          BOARD_PIN_MOTOR_IN1, BOARD_PIN_MOTOR_IN2,
          MOTOR_LEDC_FREQ_HZ, 8, (unsigned)s_dead_time_ms);
    return true;
}

void motor_stop(void)
{
    if (s_dir == MOTOR_DIR_STOP) {
        motor_apply(0, 0);
        return;
    }
    motor_apply(0, 0);
    motor_profile_stop();   /* v1.1.26：收尾电流统计（会打印一条日志） */
    s_dir = MOTOR_DIR_STOP;
    s_dir_started_us = 0;
}

bool motor_set_dir_speed(motor_dir_t dir, int speed_pct)
{
    if (dir != MOTOR_DIR_FEED && dir != MOTOR_DIR_RETRACT && dir != MOTOR_DIR_STOP) {
        return false;
    }
    if (speed_pct < 0) speed_pct = 0;
    if (speed_pct > 100) speed_pct = 100;

    if (dir == MOTOR_DIR_STOP) {
        motor_stop();
        return true;
    }

    if (speed_pct == 0) {
        motor_stop();
        return true;
    }

    bool same_dir = (s_dir == dir);
    if (same_dir && s_speed_pct == speed_pct && s_dir_started_us != 0) {
        return true;  /* 幂等 */
    }

    if (!same_dir && s_dir != MOTOR_DIR_STOP) {
        motor_apply(0, 0);
        if (s_dead_time_ms) {
            vTaskDelay(pdMS_TO_TICKS(s_dead_time_ms));
        }
    }

    if (dir == MOTOR_DIR_FEED) {
        motor_apply(speed_pct, 0);
    } else {
        motor_apply(0, speed_pct);
    }

    /* v1.1.26：开始/重启一次电流统计（换向或换速都算新的一段） */
    motor_profile_start(dir, speed_pct);

    if (!same_dir || s_dir_started_us == 0) {
        s_dir_started_us = esp_timer_get_time();
    }
    s_dir = dir;
    s_speed_pct = speed_pct;
    return true;
}

bool motor_set_dir(motor_dir_t dir)
{
    return motor_set_dir_speed(dir, MOTOR_SPEED_FULL);
}

motor_dir_t motor_get_dir(void)
{
    return s_dir;
}

int motor_get_speed_pct(void)
{
    return s_speed_pct;
}
