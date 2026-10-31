/**
 * filament_sensor.h —— 微动开关
 *
 * 从 esp32s3-485/main/filament_sensor.h 移植到 Arduino 框架。
 *
 * ============================================================================
 * 两种模式（只影响"怎么读、怎么映射"，上层 API 完全一样）
 * ============================================================================
 *
 * 【正式模式】SENSOR_TEMP_DIGITAL = 0 —— 电阻编码，1 个 ADC 脚带 2 只微动：
 *     开始位 微动(NO) ──4.7K──┐
 *                             ├── GPIO（ADC）── 内部上拉
 *     停止位 微动(NC) ──10K ──┘
 *
 *     < MV_START (433mV)  → 开始位压下（要补料）
 *     MV_START~MV_STOP    → 中间（保持上一决策）
 *     > MV_STOP (1050mV)  → 停止位顶开（缓冲器满）
 *
 * 【临时直连模式】SENSOR_TEMP_DIGITAL = 1 —— 电阻编码微动还没到货时的过渡方案。
 *   8 个微动 IO 各接一只普通微动（一端 GPIO，一端 GND，内部上拉）：
 *
 *     微动1..4 → 通道 1..4 的「开始送料」微动（压下 = 缓冲器空 = 要补料）
 *     微动5..8 → 通道 1..4 的「停止送料」微动（压下 = 缓冲器满 = 停料）
 *
 *   即：通道 c 的开始位 = 微动 c，停止位 = 微动 c+4。
 *
 *   ★ 注意 GPIO 编号不是连续的 0..7（沿用正式模式的引脚表）：
 *
 *     通道 | 开始位微动 | GPIO | 停止位微动 | GPIO
 *     -----+------------+------+------------+------
 *       1  |   微动 1   |  1   |   微动 5   |  10
 *       2  |   微动 2   |  2   |   微动 6   |  11
 *       3  |   微动 3   |  9   |   微动 7   |  12
 *       4  |   微动 4   |  8   |   微动 8   |  13
 *
 *   例：微动 1(GPIO1) = 通道 1 开始送料，微动 5(GPIO10) = 通道 1 停止送料。
 *
 *   ★ 这里用 digitalRead 而不是 ADC，是有原因的：GPIO11/12/13 属于 ADC2，
 *     WiFi 开着时 ADC 读不到（正式模式 6/7/8 路"降级"就是这个原因）。
 *     digitalRead 不受 ADC2 限制，正好躲开 —— 4 通道测试全程 WiFi 开着也没问题。
 *
 *   极性由 SENSOR_TEMP_ACTIVE_LOW 决定：1 = 微动另一端接 GND（压下读到低电平）。
 *   如果接线是"微动接 3.3V、引脚加下拉"，把它改成 0。
 *
 *   切回正式模式：把 SENSOR_TEMP_DIGITAL 改回 0，重新编译即可，其他一行不用动。
 */

#ifndef FILAMENT_SENSOR_H
#define FILAMENT_SENSOR_H

#include <Arduino.h>

/* ===== 临时直连微动模式（v1.1.14）===== */
#ifndef SENSOR_TEMP_DIGITAL
#define SENSOR_TEMP_DIGITAL      1   /* 1=临时直连微动（当前）；0=正式电阻编码 */
#endif
#ifndef SENSOR_TEMP_ACTIVE_LOW
#define SENSOR_TEMP_ACTIVE_LOW   1   /* 1=微动接 GND，压下为低电平 */
#endif

/* 微动类型 */
typedef enum {
    SENSOR_STOP     = 0,  /* 停止送料微动 */
    SENSOR_START    = 1,  /* 开始送料微动 */
    SENSOR_KIND_COUNT = 2,
} sensor_kind_t;

/* 去抖参数 */
#define SENSOR_POLL_MS      5    /* 轮询周期 */
#define SENSOR_DEBOUNCE_N   3    /* 连续多少次一致才认可 */

/* 初始化：配置 ADC，启动轮询定时器 */
bool sensor_init(void);

/* 去抖后状态：微动是否触发 */
bool sensor_triggered(int channel, sensor_kind_t kind);

/* 原始电压（mV） */
int sensor_combo_mv(int channel);

/* ADC 单元：1=ADC1, 2=ADC2, 0=无 */
int sensor_combo_adc_unit(int channel);

/* 是否降级（ADC2 + WiFi 占用） */
bool sensor_combo_degraded(int channel);

/* 该通道缓冲送料是否启用 */
bool sensor_channel_enabled(int channel);

/* 设置通道启用状态 */
bool sensor_set_channel_enabled(int channel, bool enabled);

/* 启用掩码（bit0 = 料盘位1） */
uint8_t sensor_enabled_mask(void);

/* ★ v1.1.24：诊断用 —— 第 idx 只微动脚的**原始** digitalRead 电平
 *   （1 = HIGH 松开，0 = LOW 压下；不受去抖 / 掩码 / 通道映射影响）
 * 返回 -1 表示 idx 越界。
 *
 * 为什么需要：现场"某几路微动 WEB 上完全不变"时，必须先把
 *   「引脚没信号（硬件/接线）」和「有信号但被判成别的状态（软件）」
 *   分开，否则只能瞎猜。 */
int sensor_raw_level(int idx);

/* ★ v1.1.24：第 idx 只微动脚的 GPIO 编号（-1 越界） */
int sensor_pin_of(int idx);

/* 边沿序号 */
uint32_t sensor_trigger_seq(int channel, sensor_kind_t kind);
uint32_t sensor_release_seq(int channel, sensor_kind_t kind);

/* 等待触发/释放 */
bool sensor_wait_trigger(int channel, sensor_kind_t kind, uint32_t timeout_ms);
bool sensor_wait_release(int channel, sensor_kind_t kind, uint32_t timeout_ms);

/* JSON 状态描述 */
int sensor_describe_json(char *buf, size_t buflen);

#endif /* FILAMENT_SENSOR_H */
