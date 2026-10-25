/**
 * current_sense.h —— 电机电流采样 + 堵转保护（AT8236 ISEN · GPIO3）
 *
 * 硬件：
 *   - AT8236 的 ISEN 脚输出电压 = 电机电流 × 采样电阻 R11
 *   - 本板 R11 = 200mΩ → 200mV/A；芯片自身硬限流约 2.5A
 *     （VREF=5V ÷ (10 × 200mΩ)），超过后电流读数会停在平顶
 *   - ISEN 接 GPIO3 = ADC1_CHANNEL_2（ADC1，WiFi 开启也能读）
 *
 * 采样：esp_timer 10ms 周期，IIR 滤波 + 1s 滑动峰值窗口。
 *
 * 堵转保护：
 *   - stall_ma = 0 → 只显示、不保护（默认，阈值需实测后填写）
 *   - 电机运行且过了「起步屏蔽」(blank_ms) 后，滤波电流持续
 *     超过 stall_ma 达 stall_ms → 跳闸：停电机 + 断离合（回调），
 *      latch 住直到 /current_clear
 *
 * 配置持久化：Preferences "s3ams-cur"（改 /current_set 立即生效并保存）
 */

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <Arduino.h>

/* 默认配置（前端"恢复默认"用的就是这套） */
#define CURRENT_DEFAULT_R_MOHM     200   /* 采样电阻 mΩ */
#define CURRENT_DEFAULT_STALL_MA   0     /* 0 = 不保护（阈值待实测） */
#define CURRENT_DEFAULT_STALL_MS   300   /* 持续多久算堵转 */
#define CURRENT_DEFAULT_BLANK_MS   800   /* 起步屏蔽（起动电流是稳态 3~8 倍） */

/* 夹取范围（与前端输入框 min/max 一致） */
#define CURRENT_R_MOHM_MIN     10
#define CURRENT_R_MOHM_MAX     2000
#define CURRENT_STALL_MA_MIN   0
#define CURRENT_STALL_MA_MAX   20000
#define CURRENT_STALL_MS_MIN   20
#define CURRENT_STALL_MS_MAX   5000
#define CURRENT_BLANK_MS_MIN   0
#define CURRENT_BLANK_MS_MAX   5000

/* 初始化（ADC + 定时器 + 读 NVS 配置）。失败时 ready=false，其余接口安全返回 */
bool current_sense_init(void);

/* ADC 是否可用 */
bool current_sense_ready(void);

/* 采样是否开启（ready 即开启；本板型电流采样常开） */
bool current_sense_enabled(void);

/* 滤波后瞬时值 */
int current_sense_mv(void);
int current_sense_ma(void);

/* 1s 滑动窗口峰值 */
int current_sense_peak_mv(void);
int current_sense_peak_ma(void);

/* 堵转状态 */
bool current_sense_tripped(void);      /* latch：已跳闸，未清除 */
uint32_t current_sense_trips(void);    /* 累计跳闸次数 */

/* 配置读取 */
int current_sense_r_mohm(void);
int current_sense_stall_ma(void);
int current_sense_stall_ms(void);
int current_sense_blank_ms(void);

/* 一次性设置全部配置（各自夹取到合法范围 + 持久化） */
void current_sense_set_config(int r_mohm, int stall_ma, int stall_ms, int blank_ms);

/* 恢复默认（200mΩ / 不保护 / 300ms / 800ms）并持久化 */
void current_sense_reset_defaults(void);

/* 清除跳闸 latch + 峰值窗口（/current_clear） */
void current_sense_clear(void);

/* 注册堵转动作回调（AMCU 注册 AMCU_drive_stop：停电机+断离合） */
void current_sense_set_stall_cb(void (*cb)(void));

#endif /* CURRENT_SENSE_H */
