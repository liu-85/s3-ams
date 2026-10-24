/**
 * AMCU.cpp - ESP32-S3 主控单元（合并项目 s3-ams）
 *
 * 架构变更（对比原 Tianshu RP2040 版）：
 *   - 8 路独立电机 → 1 个共享电机 + 8 个电磁离合（互斥吸合）
 *   - 每路 1 微动（耗材在线）→ 每路 2 微动（缓冲器开始位/停止位，电阻编码 ADC）
 *   - 微动只在打印/流量校准时参与送料，AMS 进料时不参与
 *   - 离合只在送料/退料时吸合，且同一时刻只有 1 路吸合
 */

#include "AMCU.h"
#include <Arduino.h>
#include <driver/gpio.h>

// ============================
//  硬件参数定义
// ============================
#define MAX_FILAMENT_NUM 16
#define MAX_AMS_NUM 4
#define PULL_BACK_METER 200
#define LED_BLINK_INTERVAL 500
#define BUFFER_FEED_TIMEOUT_MS 15000  /* 缓冲送料无反馈超时（15秒回退停止） */

// ============================
//  全局变量
// ============================

// 退料超时时间数组（来自 settings_storage.cpp）
extern int PULLBACK_TIMEOUT[16];

// 当前动作状态
_filament_motion_state_set act = waiting;

// 电机/离合状态
bool ams_enable = true;
float target = 0;

// 退料状态（兼容原逻辑）
bool is_need_pull_back = false;
int now_filament_num_need_pull_back = 255;
unsigned long last_pullback_time = 0;

// LED 状态
static led_state_t s_led_state = LED_STATE_BLUE;

// 当前吸合的通道（0 = 无）
static int s_current_channel = 0;

// 缓冲送料状态跟踪（每通道上次决策）
static int s_buffer_feed_decision[BOARD_CHANNEL_COUNT] = {0};  // 0=停, 1=送料
static uint32_t s_buffer_last_feedback[BOARD_CHANNEL_COUNT] = {0};

// last_action 跟踪（简化版）
static uint8_t s_last_action[MAX_FILAMENT_NUM] = {0};
static bool s_last_need_pull_back[MAX_FILAMENT_NUM] = {false};

// ============================
//  RGB LED 控制（板载 LED 替代 NeoPixel）
// ============================
void set_led_state(led_state_t state)
{
    s_led_state = state;
    int pin = BOARD_PIN_LED;
    if (pin < 0) return;

    switch (state) {
    case LED_STATE_RED:
    case LED_STATE_GREEN:
        gpio_set_level((gpio_num_t)pin, BOARD_LED_ACTIVE_LEVEL);
        break;
    case LED_STATE_BLUE:
    default:
        gpio_set_level((gpio_num_t)pin, 1 - BOARD_LED_ACTIVE_LEVEL);
        break;
    }
}

#define RGB_RED    set_led_state(LED_STATE_RED)
#define RGB_GREEN  set_led_state(LED_STATE_GREEN)
#define RGB_BLUE   set_led_state(LED_STATE_BLUE)

// ============================
//  辅助函数（提前声明）
// ============================
static bool last_action_need_pull_back_set(int f);
static void last_action_set(int f, _filament_motion_state_set m);

// ============================
//  核心硬件动作：电机 + 离合
// ============================

/**
 * 吸合指定通道离合 + 设定电机方向
 * @param channel 通道号 1~8（对应料盘位）
 * @param dir 方向：MOTOR_DIR_FEED=进料, MOTOR_DIR_RETRACT=退料
 * @return true=成功
 */
bool AMCU_drive_channel(int channel, motor_dir_t dir, int speed_pct)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        log_e("AMCU_drive_channel: 通道 %d 越界", channel);
        return false;
    }

    /* 1) 吸合离合（内部保证互斥：先全断再吸合目标） */
    if (!clutch_engage(channel)) {
        log_e("AMCU_drive_channel: 通道 %d 离合吸合失败", channel);
        motor_stop();
        return false;
    }

    s_current_channel = channel;

    /* 2) 启动电机 */
    if (dir == MOTOR_DIR_STOP) {
        motor_stop();
    } else {
        motor_set_dir_speed(dir, speed_pct);
    }

    RGB_GREEN;
    return true;
}

/**
 * 停止电机 + 释放所有离合
 */
void AMCU_drive_stop()
{
    motor_stop();
    clutch_release_all();
    s_current_channel = 0;
    RGB_BLUE;
}

// ============================
//  缓冲器辅助送料（打印时由微动驱动）
// ============================
void AMCU_buffer_assist_feed(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) return;
    if (!sensor_channel_enabled(channel)) return;

    bool start_trig = sensor_triggered(channel, SENSOR_START);  // 开始位压下=需补料
    bool stop_trig  = sensor_triggered(channel, SENSOR_STOP);   // 停止位顶开=满

    uint32_t now = millis();

    if (start_trig) {
        /* 需要补料：吸合离合 + 进料 */
        if (!clutch_is_engaged(channel + 1)) {
            if (AMCU_drive_channel(channel + 1, MOTOR_DIR_FEED, MOTOR_SPEED_FULL)) {
                s_buffer_feed_decision[channel] = 1;
                s_buffer_last_feedback[channel] = now;
            }
        } else {
            motor_set_dir(MOTOR_DIR_FEED);
            s_buffer_last_feedback[channel] = now;
        }
    } else if (stop_trig) {
        /* 缓冲器满：停电机 + 释放离合 */
        AMCU_drive_stop();
        s_buffer_feed_decision[channel] = 0;
        s_buffer_last_feedback[channel] = now;
    } else {
        /* 中间档：保持上一决策 */
        if (s_buffer_feed_decision[channel] == 1) {
            if (now - s_buffer_last_feedback[channel] > BUFFER_FEED_TIMEOUT_MS) {
                AMCU_drive_stop();
                s_buffer_feed_decision[channel] = 0;
            }
            /* 否则保持送料 */
        }
    }
}

// ============================
//  初始化 IO 引脚
// ============================
void init_io_pins() {
    if (BOARD_PIN_LED >= 0) {
        gpio_reset_pin((gpio_num_t)BOARD_PIN_LED);
        gpio_set_direction((gpio_num_t)BOARD_PIN_LED, GPIO_MODE_OUTPUT);
    }
    if (BOARD_485_ENABLED) {
        gpio_reset_pin((gpio_num_t)BOARD_PIN_485_DE);
        gpio_set_direction((gpio_num_t)BOARD_PIN_485_DE, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)BOARD_PIN_485_DE, 0);
    }
    if (BOARD_CURRENT_SENSE_ENABLED && BOARD_PIN_CURRENT_SENSE >= 0) {
        gpio_reset_pin((gpio_num_t)BOARD_PIN_CURRENT_SENSE);
        gpio_set_direction((gpio_num_t)BOARD_PIN_CURRENT_SENSE, GPIO_MODE_INPUT);
    }
}

// ============================
//  兼容接口：motorCmd
// ============================
void motorCmd(int motorNum, int cmd) {
    if (motorNum < 0 || motorNum >= BOARD_CHANNEL_COUNT) return;
    int channel = motorNum + 1;

    if (cmd == 0) {
        AMCU_drive_stop();
    } else if (cmd > 0) {
        AMCU_drive_channel(channel, MOTOR_DIR_FEED, MOTOR_SPEED_FULL);
    } else {
        AMCU_drive_channel(channel, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
    }
}

// ============================
//  微动状态检查（兼容 Tianshu 的 check_microswitch_status）
// ============================
void check_buffer_status() {
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int filament_num = i;
        _filament_motion_state_set m = get_filament_motion(filament_num);
        if (m == need_send_out || m == need_pull_back) {
            continue;  // AMS 进料/退料：微动不参与
        }
        if (is_channel_printing(filament_num)) {
            AMCU_buffer_assist_feed(i);
        }
    }
}

// ============================
//  AMCU 初始化
// ============================
void AMCU_init() {
    init_io_pins();
    sensor_init();
    motor_init();
    clutch_init();
    AMCU_drive_stop();
    RGB_BLUE;

    BambuBus_init();
    delay(500);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            set_filament_online(i, j, true);
        }
    }
}

// ============================
//  AMCU 主运行逻辑
// ============================
void AMCU_run() {
    static int now_f = 255;
    int x = get_now_filament_num();
    if (x < 0 || x >= MAX_FILAMENT_NUM) x = 0;  // 防止 0xFF 越界

    if (now_f != x) {
        now_f = x;
        Bambubus_set_need_to_save();
    }

    auto state = get_filament_motion(now_f);
    if (state == need_pull_back) {
        if (!s_last_need_pull_back[now_f]) {
            s_last_need_pull_back[now_f] = true;
            is_need_pull_back = true;
            now_filament_num_need_pull_back = now_f;
            last_pullback_time = millis();
            RGB_GREEN;
        }
    } else if (state == need_send_out) {
        s_last_action[now_f] = (uint8_t)need_send_out;
        RGB_GREEN;
    } else if (state == waiting) {
        s_last_action[now_f] = (uint8_t)waiting;
        RGB_BLUE;
    }

    // 运行 BambuBus 通信（含 UART 接收）
    int stu = -1;
    if (ams_enable) stu = BambuBus_run();

    // 缓冲器微动检查（打印时驱动辅助送料）
    check_buffer_status();

    AMCU_motion();
}

// ============================
//  电机运动逻辑（带离合/缓冲）
// ============================
void AMCU_motion() {
    int cur = get_now_filament_num();
    if (cur < 0 || cur >= MAX_FILAMENT_NUM) cur = 0;  // 防止 0xFF 越界

    // 退料超时处理
    if (is_need_pull_back) {
        int timeout_seconds = 7;
        if (now_filament_num_need_pull_back >= 0 &&
            now_filament_num_need_pull_back < 4 * MAX_AMS_NUM) {
            timeout_seconds = PULLBACK_TIMEOUT[now_filament_num_need_pull_back];
        }

        if (millis() - last_pullback_time >= (timeout_seconds * 1000)) {
            int pull_back_channel = now_filament_num_need_pull_back;
            AMCU_drive_stop();
            is_need_pull_back = false;
            now_filament_num_need_pull_back = 255;

            if (ams_enable && pull_back_channel >= 0) {
                set_filament_motion(pull_back_channel, waiting);
            }
            RGB_BLUE;
        } else {
            int ch = now_filament_num_need_pull_back + 1;
            AMCU_drive_channel(ch, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
            RGB_GREEN;
        }
    }

    if (ams_enable) act = get_filament_motion(cur);

    // 打印中：由缓冲微动驱动，AMCU_motion 不覆盖（check_buffer_status 已处理）
    if (is_channel_printing(cur)) {
        // 保持绿灯（动作中），不调用 AMCU_drive_stop
        RGB_GREEN;
        clutch_assert_single(true);
        return;
    }

    switch (act) {
    case need_pull_back:
        RGB_GREEN;
        break;
    case need_send_out: {
        int ch = cur + 1;
        if (is_filament_online(cur)) {
            AMCU_drive_channel(ch, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
            RGB_GREEN;
        } else {
            AMCU_drive_stop();
            RGB_BLUE;
        }
        break;
    }
    case waiting:
        AMCU_drive_stop();
        RGB_BLUE;
        break;
    case act_send_mm:
        AMCU_drive_channel(cur + 1, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
        RGB_GREEN;
        break;
    case act_pull_mm:
        AMCU_drive_channel(cur + 1, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
        RGB_GREEN;
        break;
    case cancel:
    case release_all:
        AMCU_drive_stop();
        RGB_RED;
        act = waiting;
        break;
    default:
        AMCU_drive_stop();
        RGB_BLUE;
        break;
    }

    // 运行期离合体检
    clutch_assert_single(true);
}

// ============================
//  辅助函数实现
// ============================
static bool last_action_need_pull_back_set(int f) {
    if (f < 0 || f >= MAX_FILAMENT_NUM) return true;
    if (!s_last_need_pull_back[f]) {
        s_last_need_pull_back[f] = true;
        return false;
    }
    return true;
}

static void last_action_set(int f, _filament_motion_state_set m) {
    if (f < 0 || f >= MAX_FILAMENT_NUM) return;
    s_last_action[f] = (uint8_t)m;
    if (m != need_pull_back) s_last_need_pull_back[f] = false;
}
