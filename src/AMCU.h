#pragma once

#include "main.h"
#include "BambuBus.h"
#include "motor.h"
#include "clutch.h"
#include "filament_sensor.h"
#include "board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 微动类型重新映射（缓冲器双微动语义）====================
// 原 Tianshu 的 1 微动（耗材在线检测）已废弃
// 新架构：每路 2 微动 = 缓冲器开始位(SENSOR_START) + 停止位(SENSOR_STOP)
//   开始位压下 → 缓冲器空，需要补料
//   停止位顶开 → 缓冲器满，停止补料
//   中间档      → 保持上一决策

// ==================== 硬件参数定义 ====================
#define MAX_FILAMENT_NUM 16        // 最大耗材数量
#define MAX_AMS_NUM 4              // 最大AMS数量
#define WHEEL_D 4.5                // 轮子直径
#define SERVO_DELAY 100            // 伺服电机延迟时间(ms)
#define PULL_BACK_METER 200        // 退料距离
#define LED_BLINK_INTERVAL 500     // LED闪烁间隔
#define SERVO_COUNT 4              // 伺服电机数量

// ==================== RGB LED 配置（状态指示灯）====================
// 用板载 LED (GPIO42) 替代原 NeoPixel
typedef enum {
    LED_STATE_BLUE = 0,    // 待机/正常
    LED_STATE_GREEN = 1,   // 进料/退料动作中
    LED_STATE_RED = 2,     // 停止/故障/取消
} led_state_t;

void set_led_state(led_state_t state);

// ==================== 核心功能函数 ====================
void AMCU_init();
void AMCU_run();
void AMCU_motion();

// 电机+离合复合动作（核心硬件控制）
// 吸合指定通道离合 + 设定电机方向（异步，不阻塞主循环/Web），返回是否本周期可驱动
bool AMCU_drive_channel(int channel, motor_dir_t dir, int speed_pct);
// 停止电机 + 异步释放所有离合
void AMCU_drive_stop();
// 结算上一笔挂起的离合请求（每循环在驱动电机前调；吸合完成后才给电机）
void AMCU_clutch_pump();

// 送料距离跟踪（兼容 Tianshu 的 last_pullback_meters + target 语义）
// 进入进料前：last_pullback_meters[f] = get_filament_meters(f)；
// 每次驱动周期：add_filament_meters(f, ±Δ)
// 停止条件：last_pullback_meters[f] - get_filament_meters(f) > target_meters
extern float last_pullback_meters[MAX_FILAMENT_NUM];
extern float target_meters;

// 送料距离跟踪（Tianshu 的 last_pullback_meters + target 语义）
// 进入进料前：last_pullback_meters[f] = get_filament_meters(f)；
// 每次驱动周期：add_filament_meters(f, ±Δ)
// 停止条件：last_pullback_meters[f] - get_filament_meters(f) > target
extern float last_pullback_meters[MAX_FILAMENT_NUM];
extern float target_meters;

// 485 运动执行开关（Web 驱动）
void amcu_set_ams_enable(bool en);
extern bool ams_enable;

// v1.1.12：before_on_use 跟随送料是否处于"送"的相位（诊断用）
bool amcu_follow_feed_active(void);

// ==================== 手动点动（Web /hardware_test，测电流/占空比用）====================
// 非阻塞：吸合离合 → 按 dir/pct 驱动电机 → 到时限自动停并断离合
bool amcu_jog_start(int channel, int dir, int ms, int pct);
bool amcu_jog_busy(void);
extern int jog_time_ms;      // 点动时长（ms），默认 1000
extern int jog_speed_pct;    // 点动 PWM 占空比（%），默认 100
void jog_config_set(int ms, int pct);  // 夹取 + NVS 持久化

// 缓冲器辅助送料（打印时由微动驱动）
void AMCU_buffer_assist_feed(int channel);

// 对外接口（保持与 Tianshu 兼容，供 BambuBus 调用）
void motorCmd(int motorNum, int cmd);  // 兼容接口：motorNum=通道(0起), cmd: 1=进料 -1=退料 0=停
void init_io_pins();

// 微动状态查询（兼容 Tianshu 的 check_microswitch_status 语义）
// 注意：新架构中微动不参与 AMS 进料，只在打印/流量校准时参与
void check_buffer_status();

extern int PULLBACK_TIMEOUT[16];  // 退料超时时间数组（来自 settings_storage）

#ifdef __cplusplus
}
#endif
