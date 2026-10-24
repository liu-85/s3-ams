/**
 * filament_sensor.h —— 微动开关（每路 2 个，电阻编码 + ADC 单脚读取）
 *
 * 从 esp32s3-485/main/filament_sensor.h 移植到 Arduino 框架。
 *
 * 每路（每个料盘位）两只微动合占 1 个 ADC 脚：
 *     开始位 微动(NO) ──4.7K──┐
 *                             ├── GPIO（ADC）── 内部上拉
 *     停止位 微动(NC) ──10K ──┘
 *
 * 三档电压：
 *     < MV_START (433mV)  → 开始位压下（要补料）
 *     MV_START~MV_STOP    → 中间（保持上一决策）
 *     > MV_STOP (1050mV)  → 停止位顶开（缓冲器满）
 */

#ifndef FILAMENT_SENSOR_H
#define FILAMENT_SENSOR_H

#include <Arduino.h>

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

/* 边沿序号 */
uint32_t sensor_trigger_seq(int channel, sensor_kind_t kind);
uint32_t sensor_release_seq(int channel, sensor_kind_t kind);

/* 等待触发/释放 */
bool sensor_wait_trigger(int channel, sensor_kind_t kind, uint32_t timeout_ms);
bool sensor_wait_release(int channel, sensor_kind_t kind, uint32_t timeout_ms);

/* JSON 状态描述 */
int sensor_describe_json(char *buf, size_t buflen);

#endif /* FILAMENT_SENSOR_H */
