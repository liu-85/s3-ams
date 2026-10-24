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

#include "motor.h"
#include "board_pins.h"

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
