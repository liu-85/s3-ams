#pragma once
#include <Arduino.h>

// ============================
// 设置存储管理器 - 使用 ESP32 Preferences
// 替代 RP2040 Flash 直写方案
// ============================

// 通道状态模式定义
typedef enum {
    CHANNEL_MODE_OFFLINE = 0,    // 离线
    CHANNEL_MODE_ONLINE = 1,     // 在线
    CHANNEL_MODE_AUTO = 2         // 自动（根据微动开关）
} ChannelMode;

// 设置数据结构 - 支持16个电机独立的退料时间
struct SettingsData {
    int32_t pullback_timeout[16];    // 16个电机的退料时间（秒）
    int32_t sleep_timeout_min;      // 息屏时间（分钟）
    int32_t screen_always_on;       // 常亮标记（0或1）
    int32_t channel_mode[16];       // 16个通道的状态模式（0=离线, 1=在线, 2=自动）
    int32_t ams_count;             // AMS数量（1-4）
    uint32_t checksum;              // 校验和
    uint32_t magic;                 // 魔数
};

// 魔数定义 - 版本5
#define SETTINGS_MAGIC_NUMBER    0x53415425  // "STA%" = Settings Active V5

// 默认值定义
#define DEFAULT_PULLBACK_TIMEOUT    4
#define DEFAULT_SLEEP_TIMEOUT_MIN   15
#define DEFAULT_SCREEN_ALWAYS_ON    0
/* ★ v1.1.21：AMS 台数上限 2（AMS-A + AMS-B，共 8 料位）。
 *
 * 本板 8 个物理通道 ÷ 每台 4 料位 = 2 台。原来写 4 会让打印机看到
 * AMS-C / AMS-D 两台"幽灵 AMS"（没有任何物理通道），耗材页出现
 * 重复卡片。改成 2 后 0x05 注册、0x402 序列号、0x103 版本都只报 A/B。 */
#define MAX_AMS_COUNT             2
#define DEFAULT_AMS_COUNT         2

/* ★ v1.1.26：自动送料（微动缓冲送料）的电机 PWM 百分比，WEB 可调。
 *
 * 为什么单独存：SettingsData 带 checksum，往里加字段会让老配置整包失效
 * （用户设过的退料时间等会被重置）。所以用独立的 NVS 命名空间 "s3ams-feed"。
 *
 * 下限为什么是 40 而不是 1：本硬件实测电机 PWM 低于 80 起不来
 * （见项目铁律），40 只是留给"轻载/空转"的余量，实际送料建议 80~100。 */
#define DEFAULT_FEED_PWM          100
#define FEED_PWM_MIN              40
#define FEED_PWM_MAX              100

// 外部变量（供其他模块访问）
extern int PULLBACK_TIMEOUT[16];
extern int SLEEP_TIMEOUT_MIN;
extern bool SCREEN_ALWAYS_ON;
extern int CHANNEL_MODE[16];
extern int AMS_COUNT;
extern int FEED_PWM;   /* v1.1.26：自动送料 PWM 百分比（40~100） */

// 初始化设置存储（在setup中调用）
void settings_storage_init();

// 从存储加载设置到变量
void settings_load_from_flash();

// 保存当前设置到存储
void settings_save_to_flash();

// 重置为默认值（不自动保存，需手动调用save）
void settings_reset_to_defaults();

// 检查设置是否有效（校验和验证）
bool settings_is_valid();

// 获取指定电机的退料超时时间
int get_motor_pullback_timeout(int motor_index);

// 设置全局退料超时（所有通道统一），返回实际保存的秒数；非法返回 -1
int settings_set_pullback_timeout_all(int sec);

// 读取全局退料超时（所有通道相同时返回该值，否则返回通道0的值）
int settings_get_pullback_timeout_common(void);

/* ★ v1.1.26：自动送料 PWM（微动缓冲送料的电机速度）
 *   set: 40~100 合法，返回实际保存值；越界返回 -1 且不改动
 *   get: 当前生效值（启动时已从 NVS 载入） */
int settings_set_feed_pwm(int pct);
int settings_get_feed_pwm(void);
