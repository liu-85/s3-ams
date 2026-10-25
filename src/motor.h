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

#endif /* MOTOR_H */
