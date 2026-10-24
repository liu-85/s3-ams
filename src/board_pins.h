/**
 * board_pins.h —— ESP32-S3 硬件引脚定义（合并项目 s3-ams）
 * ============================================================================
 *
 * 本文件整合了 esp32s3-485 项目的硬件配置，并适配 Arduino 框架。
 * 所有引脚定义都集中在这里，改接线只改这一个文件。
 *
 * 硬件架构：
 *   - 1 个共享直流电机（H 桥驱动 8 路料盘位，通过电磁离合选通）
 *   - 8 个电磁离合（ULN2803 驱动，高电平吸合）
 *   - 8 路缓冲器，每路 2 个微动（开始位 + 停止位，电阻编码 + ADC 单脚读取）
 *   - RS-485 直连打印机（BambuBus 协议）
 *   - 状态 LED
 *   - 电机电流采样（ISEN）
 *
 * 参考：esp32s3-485/main/board_pins_s3.h（按已打样 PCB 原理图核对）
 */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <Arduino.h>

/* ==========================================================================
 * 一、通道数
 * ========================================================================== */
#define BOARD_CHANNEL_COUNT 8

/* ==========================================================================
 * 二、执行机构（输出）
 * ========================================================================== */

/* 共享电机 H 桥（AT8236）
 * IN1 = GPIO4, IN2 = GPIO5
 * 注意：原理图 G5→IN1, G4→IN2，固件沿用历史命名 4/5
 *   进/退料方向反了就把这两行对调 */
#define BOARD_PIN_MOTOR_IN1 4
#define BOARD_PIN_MOTOR_IN2 5

/* 电磁离合 1~8（ULN2803 反相驱动，高电平吸合）
 * 按 PCB 实际走线（ULN2803A 反推）：
 *   In1..In8 = G47,G21,G18,G17,G16,G15,G7,G6
 *   Out18..Out11 = O8..O1 → 电磁8..电磁1
 *   电磁1=G6 电磁2=G7 电磁3=G15 电磁4=G16
 *   电磁5=G17 电磁6=G18 电磁7=G21 电磁8=G47 */
#define BOARD_PIN_CLUTCH_1 6
#define BOARD_PIN_CLUTCH_2 7
#define BOARD_PIN_CLUTCH_3 15
#define BOARD_PIN_CLUTCH_4 16
#define BOARD_PIN_CLUTCH_5 17
#define BOARD_PIN_CLUTCH_6 18
#define BOARD_PIN_CLUTCH_7 21
#define BOARD_PIN_CLUTCH_8 47

/* 状态 LED */
#define BOARD_PIN_LED 42

/* ==========================================================================
 * 三、电机电流采样（ISEN / AT8236）
 * ========================================================================== */
#define BOARD_PIN_CURRENT_SENSE 3
#define BOARD_CURRENT_SENSE_ENABLED 1
#define BOARD_CURRENT_SENSE_R_MOHM 200  /* 采样电阻（毫欧） */

/* ==========================================================================
 * 四、微动开关（输入）：每路 2 只，合占 1 个 ADC 脚
 * ========================================================================== */
/* 每路一个 ADC 脚：G1,G2,G9,G8,G10,G11,G12,G13 */
#define BOARD_PIN_SENSOR_1 1
#define BOARD_PIN_SENSOR_2 2
#define BOARD_PIN_SENSOR_3 9
#define BOARD_PIN_SENSOR_4 8
#define BOARD_PIN_SENSOR_5 10
#define BOARD_PIN_SENSOR_6 11
#define BOARD_PIN_SENSOR_7 12
#define BOARD_PIN_SENSOR_8 13

/* 电阻编码参数 */
#define BOARD_SENSOR_R_START_OHM 4700    /* 开始位微动串联电阻（常开那只） */
#define BOARD_SENSOR_R_STOP_OHM 10000    /* 停止位微动串联电阻（常闭那只） */
#define BOARD_SENSOR_RPU_OHM    45000    /* 芯片内部上拉典型值 */
#define BOARD_SENSOR_VDD_MV     3300     /* 供电电压 */

/* 三档判定阈值（mV） */
#define BOARD_SENSOR_MV_START 433
#define BOARD_SENSOR_MV_STOP  1050

/* ==========================================================================
 * 五、RS-485（BambuBus 直连打印机）
 * ========================================================================== */
#define BOARD_PIN_485_TX 14
#define BOARD_PIN_485_RX 48
#define BOARD_PIN_485_DE 39
#define BOARD_485_ENABLED 1
#define BOARD_485_BAUDRATE 1228800  /* 1.2288 Mbps，8E1 */

/* ==========================================================================
 * 六、电平约定
 * ========================================================================== */
#define BOARD_CLUTCH_ACTIVE_LEVEL 1   /* 高电平吸合 */
#define BOARD_LED_ACTIVE_LEVEL 1      /* 高电平点亮 */

/* ==========================================================================
 * 七、辅助宏：引脚数组
 * ========================================================================== */
#define BOARD_PIN_CLUTCH_ARRAY {BOARD_PIN_CLUTCH_1, BOARD_PIN_CLUTCH_2, BOARD_PIN_CLUTCH_3, BOARD_PIN_CLUTCH_4, BOARD_PIN_CLUTCH_5, BOARD_PIN_CLUTCH_6, BOARD_PIN_CLUTCH_7, BOARD_PIN_CLUTCH_8}
#define BOARD_PIN_SENSOR_ARRAY {BOARD_PIN_SENSOR_1, BOARD_PIN_SENSOR_2, BOARD_PIN_SENSOR_3, BOARD_PIN_SENSOR_4, BOARD_PIN_SENSOR_5, BOARD_PIN_SENSOR_6, BOARD_PIN_SENSOR_7, BOARD_PIN_SENSOR_8}

#endif /* BOARD_PINS_H */
