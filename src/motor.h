/**
 * motor.h —— 共享直流电机（H 桥 IN1/IN2）
 *
 * 从 esp32s3-485/main/motor.h 移植到 Arduino 框架。
 * 方向语义：
 *   direction =  1  → 进料（正转）
 *   direction = -1  → 退料（反转）
 *   direction =  0  → 停止
 *
 * 使用 LEDC PWM 调速，20kHz，8 位分辨率。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/* 速度百分比（0~100） */
#define MOTOR_SPEED_FULL 100
/* 485 进料占空比：实测电机需 ≥80 才转得动，留足裕量取满占空比 */
#define MOTOR_SPEED_485_FEED 100

/* 方向 */
typedef enum {
    MOTOR_DIR_RETRACT = -1,  /* 退料（反转） */
    MOTOR_DIR_STOP    =  0,  /* 停止 */
    MOTOR_DIR_FEED    =  1,  /* 进料（正转） */
} motor_dir_t;

/* 初始化电机：配置两个 LEDC 通道，输出清零 */
bool motor_init(void);

/* 立即停止（两路 PWM 都写 0） */
void motor_stop(void);

/* 设置方向并立即启动，不阻塞。速度用上一次设定值（默认全速） */
bool motor_set_dir(motor_dir_t dir);

/* 设置方向 + 速度并立即启动，不阻塞 */
bool motor_set_dir_speed(motor_dir_t dir, int speed_pct);

/* 当前方向（0 = 停） */
motor_dir_t motor_get_dir(void);

/* 当前速度百分比 */
int motor_get_speed_pct(void);

/* 把百分比换算成 LEDC 的 duty 原值（0~255） */
uint32_t motor_pct_to_duty(int pct);

/* ==========================================================================
 * v1.1.26：单次运行电流剖析（启动值 / 稳态值 / 峰值）
 * ==========================================================================
 * 目的：标定堵转阈值。一次运行结束后给出
 *   启动峰值（前 blank_ms，起动电流是稳态的 3~8 倍）
 *   稳态均值 + 稳态峰值（正常送料时的典型值）
 *   全程最大（接近堵转时这里会明显抬高）
 * 依据：堵转阈值建议取「稳态均值 × 2」，且明显低于启动峰值。
 *
 * 采样：20ms 周期 esp_timer，调 current_sense_ma()（已 IIR 滤波）。
 * 只有 duration_ms >= MOTOR_PROFILE_MIN_MS 的运行才写日志，避免
 * 跟随送料（250ms 脉冲）把日志刷爆。 */
typedef struct {
    bool        valid;          /* 至少完成过一次有效统计 */
    motor_dir_t dir;            /* 这次运行的方向 */
    int         pct;            /* 这次运行的 PWM 百分比 */
    uint32_t    duration_ms;    /* 运行时长 */
    int         start_peak_ma;  /* 启动段（前 blank_ms）峰值 */
    int         run_avg_ma;     /* 稳态段均值 */
    int         run_peak_ma;    /* 稳态段峰值 */
    int         max_ma;         /* 全程最大 */
} motor_current_stat_t;

/* 取最近一次运行的电流统计（返回 false 表示还没有有效样本） */
bool motor_get_current_stat(motor_current_stat_t *out);

/* 清空统计（/current_clear 或换电机后重新标定用） */
void motor_reset_current_stat(void);

#endif /* MOTOR_H */
