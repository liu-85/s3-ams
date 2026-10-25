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
#include <Preferences.h>
#include "current_sense.h"

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

/* Web "485 运动执行" 开关的 setter：
   打开 → 打印机点进料/退料会真的驱动本机电机（立即生效）
   关闭 → 只走协议层，不驱动电机
   之前 /act485_set 走 handle_ok 占位，根本没碰这个变量。 */
void amcu_set_ams_enable(bool en)
{
    ams_enable = en;
    log_i("485 运动执行开关 -> %s", en ? "ON（驱动电机）" : "OFF（仅协议）");
}

/* ============================
 *  手动点动（Web 硬件调试 / 堵转阈值实测）
 * ============================
 * 流程：jog_start 记录参数 → 每循环 jog_pump：
 *   吸合目标离合（异步）→ 到时限自动 AMCU_drive_stop
 * 点动期间 AMCU_motion 让出总线（否则 waiting 状态会把它立刻停掉）。
 * 堵转保护跳闸时也立即结束点动（见 jog_pump 内的 tripped 检查）。 */

/* 虚拟计米状态（时间法，无 AS5600；声明提前供点动复用） */
static uint32_t s_last_drive_ms = 0;
static bool     s_driving_now   = false;

static bool      s_jog_active = false;
static uint32_t  s_jog_end_ms = 0;
static int       s_jog_channel = 0;
static motor_dir_t s_jog_dir = MOTOR_DIR_STOP;
static int       s_jog_pct = MOTOR_SPEED_FULL;

int  jog_time_ms = 1000;     /* 点动时长默认 1s */
int  jog_speed_pct = 100;    /* 点动 PWM 默认 100%（实测电机启动阈值用） */

#define JOG_TIME_MS_MIN 200
#define JOG_TIME_MS_MAX 60000
#define JOG_PCT_MIN     5
#define JOG_PCT_MAX     100

static void jog_config_load(void)
{
    Preferences prefs;
    prefs.begin("s3ams-jog", true);
    jog_time_ms   = prefs.getInt("time_ms", 1000);
    jog_speed_pct = prefs.getInt("speed_pct", 100);
    prefs.end();
    if (jog_time_ms < JOG_TIME_MS_MIN) jog_time_ms = JOG_TIME_MS_MIN;
    if (jog_time_ms > JOG_TIME_MS_MAX) jog_time_ms = JOG_TIME_MS_MAX;
    if (jog_speed_pct < JOG_PCT_MIN) jog_speed_pct = JOG_PCT_MIN;
    if (jog_speed_pct > JOG_PCT_MAX) jog_speed_pct = JOG_PCT_MAX;
}

void jog_config_set(int ms, int pct)
{
    if (ms < JOG_TIME_MS_MIN) ms = JOG_TIME_MS_MIN;
    if (ms > JOG_TIME_MS_MAX) ms = JOG_TIME_MS_MAX;
    if (pct < JOG_PCT_MIN) pct = JOG_PCT_MIN;
    if (pct > JOG_PCT_MAX) pct = JOG_PCT_MAX;
    jog_time_ms = ms;
    jog_speed_pct = pct;
    Preferences prefs;
    prefs.begin("s3ams-jog", false);
    prefs.putInt("time_ms", jog_time_ms);
    prefs.putInt("speed_pct", jog_speed_pct);
    prefs.end();
    log_i("点动参数已更新: %ums @ %d%%（duty %u/255）",
          (unsigned)jog_time_ms, jog_speed_pct,
          (unsigned)motor_pct_to_duty(jog_speed_pct));
}

bool amcu_jog_start(int channel, int dir, int ms, int pct)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) return false;
    if (dir != MOTOR_DIR_FEED && dir != MOTOR_DIR_RETRACT) return false;
    if (ms < JOG_TIME_MS_MIN) ms = JOG_TIME_MS_MIN;
    if (ms > JOG_TIME_MS_MAX) ms = JOG_TIME_MS_MAX;
    if (pct < JOG_PCT_MIN) pct = JOG_PCT_MIN;
    if (pct > JOG_PCT_MAX) pct = JOG_PCT_MAX;

    s_jog_channel = channel;
    s_jog_dir = (motor_dir_t)dir;
    s_jog_pct = pct;
    s_jog_end_ms = millis() + ms;
    s_jog_active = true;
    log_i("点动开始: 通道%d %s %ums @ %d%%（duty %u/255）",
          channel, dir == MOTOR_DIR_FEED ? "进料" : "退料",
          (unsigned)ms, pct, (unsigned)motor_pct_to_duty(pct));
    return true;
}

static void amcu_jog_pump(void)
{
    if (!s_jog_active) return;

    /* 堵转保护跳闸 → 立即结束点动（电机/离合已由保护回调停掉） */
    if (current_sense_tripped()) {
        s_jog_active = false;
        return;
    }

    if ((int32_t)(millis() - s_jog_end_ms) >= 0) {
        AMCU_drive_stop();
        s_jog_active = false;
        log_i("点动结束（到时自动停止）");
        return;
    }

    if (clutch_is_engaged(s_jog_channel)) {
        /* 离合已咬合：按点动速度驱动（幂等，同方向同速度不重写） */
        motor_set_dir_speed(s_jog_dir, s_jog_pct);
        s_driving_now = true;
    } else if (!clutch_is_busy()) {
        /* 发起离合吸合请求（异步）；吸合后下一循环本函数给电机 */
        AMCU_drive_channel(s_jog_channel, s_jog_dir, s_jog_pct);
    }
}

bool amcu_jog_busy(void)
{
    return s_jog_active;
}

// 退料状态（兼容原逻辑）
bool is_need_pull_back = false;
int now_filament_num_need_pull_back = 255;
unsigned long last_pullback_time = 0;

/* 送料距离跟踪（兼容 Tianshu 的 last_pullback_meters 语义）
 * 进入进料时记 last_pullback_meters[f] = get_filament_meters(f)；
 * 每次驱动周期把本周期推进的米数累加到 filament_meters；
 *
 * ★ 重要：本硬件无 AS5600，送料结束不靠本地测距，而靠
 *   BambuBus 协议帧把状态切到 waiting。target_meters 默认 0，
 *   此时"按距离停料"的逻辑被完全跳过（见 AMCU_motion 判断），
 *   电机由 BambuBus 状态机驱动到结束。
 *   若以后想"送固定 N 米就停"，把 target_meters 设为 N（单位米）即可。 */
float last_pullback_meters[MAX_FILAMENT_NUM] = {0};
float target_meters = 0.0f;        /* 目标送料距离（米）；0 = 不限距离，由 BambuBus 切到 waiting 结束 */

/* 虚拟计米速率（时间法，无 AS5600）：
 * AMS 送料实际速度约 30~60 mm/s，取 50 mm/s = 0.05 m/s。
 * 仅用于 get_filament_meters() 数值累加（显示/记录），
 * 当 target_meters == 0 时不参与停料判断。 */
#define FILAMENT_PUSH_M_PER_SEC     0.05f
/* 累计到 filament_meters 的推进（虚拟计米；本硬件无 AS5600，用时间法） */
static void accumulate_feed_meters(int f)
{
    if (!s_driving_now) return;
    uint32_t now = millis();
    uint32_t dt = now - s_last_drive_ms;
    s_last_drive_ms = now;
    if (dt == 0) return;
    float meters = (float)dt * FILAMENT_PUSH_M_PER_SEC;
    add_filament_meters(f, meters);
}

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

/* 当前挂起的离合请求（非阻塞；后台任务执行完前主循环不卡） */
static clutch_req_t *s_pending_clutch = NULL;

/* ★ 挂起的电机参数：离合是异步吸合的，从"发起请求"到"机械咬合"要 80ms，
   这期间必须把 dir/speed 存住 —— 否则咬合后 pump 里 motor_get_dir()
   拿到的还是 STOP（电机压根没启过），"启动电机"变成 motor_stop()，
   现场症状就是"该送料时离合响了但电机没电没转"（2026-10-25 实测）。 */
static motor_dir_t s_pending_dir = MOTOR_DIR_STOP;
static int         s_pending_speed = MOTOR_SPEED_FULL;

/**
 * 吸合指定通道离合 + 设定电机方向（异步，不阻塞主循环/Web）
 * @param channel 通道号 1~8（对应料盘位）
 * @param dir 方向：MOTOR_DIR_FEED=进料, MOTOR_DIR_RETRACT=退料
 * @return true=已发起且本周期可驱动电机
 *
 * 要点：
 *   - 若目标离合已吸合 → 立即启动电机（不重复请求、不卡 80ms）
 *   - 若尚未吸合 → 异步请求（后台任务吸收时序），本周期不驱动电机
 */
bool AMCU_drive_channel(int channel, motor_dir_t dir, int speed_pct)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        log_e("AMCU_drive_channel: 通道 %d 越界", channel);
        return false;
    }

    /* 1) 离合（互斥由后台任务保证：吸合前全断） */
    if (dir == MOTOR_DIR_STOP) {
        /* 停止：不必吸合，直接全断 */
        AMCU_drive_stop();
        return true;
    }

    if (clutch_is_engaged(channel)) {
        /* 已吸合目标通道：直接给电机，不重复请求 */
        motor_set_dir_speed(dir, speed_pct);
        s_current_channel = channel;
        s_pending_clutch = NULL;
        s_driving_now = true;
        s_last_drive_ms = millis();
    } else {
        /* 未吸合：异步发起，等后台任务完成（下一循环 poll 成功再驱动电机） */
        s_pending_clutch = clutch_request_engage(channel);
        s_current_channel = channel;
        /* 记住这次要跑的方向/速度，咬合后 pump 按它启动电机 */
        s_pending_dir = dir;
        s_pending_speed = speed_pct;
        /* 本周期不启动电机：要等离合机械咬合后才给电，避免打滑 */
        s_driving_now = false;
        RGB_GREEN;
        return false;
    }

    RGB_GREEN;
    return true;
}

/* 在每循环"驱动电机"前调用：把上一笔离合请求结算掉。
 * 若已吸合则立刻启动电机；否则本周期不驱动（等吸合完成）。 */
void AMCU_clutch_pump()
{
    if (s_pending_clutch == NULL) return;
    int want = s_current_channel;
    if (want < 1 || want > BOARD_CHANNEL_COUNT) {
        s_pending_clutch = NULL;
        s_driving_now = false;
        return;
    }
    if (clutch_is_engaged(want)) {
        /* 离合已咬合 → 按挂起时记下的方向/速度启动电机
           （不能用 motor_get_dir()：电机还没启过，拿到的是 STOP） */
        motor_set_dir_speed(s_pending_dir, s_pending_speed);
        s_pending_clutch = NULL;
        s_pending_dir = MOTOR_DIR_STOP;
        s_driving_now = true;
        s_last_drive_ms = millis();
    } else if (clutch_result_poll(s_pending_clutch)) {
        /* 后台已处理完；再确认吸合电平 */
        s_pending_clutch = NULL;
        if (clutch_is_engaged(want)) {
            motor_set_dir_speed(s_pending_dir, s_pending_speed);
            s_pending_dir = MOTOR_DIR_STOP;
            s_driving_now = true;
            s_last_drive_ms = millis();
        }
    }
    /* 未完成：什么都不做，等下一循环（非阻塞，Web 不受影响） */
}

/**
 * 停止电机 + 异步释放所有离合
 */
void AMCU_drive_stop()
{
    motor_stop();
    s_pending_clutch = clutch_request_release_all();  /* 非阻塞，后台吸收时序 */
    s_current_channel = 0;
    s_driving_now = false;
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
    clutch_task_start();  /* 启动非阻塞离合后台任务（修 Web 卡顿） */
    jog_config_load();

    /* 电机电流采样（AT8236 ISEN / GPIO3）+ 堵转保护 */
    current_sense_init();
    /* 堵转跳闸动作：停电机 + 异步断离合（电流模块不直接依赖离合，走回调） */
    current_sense_set_stall_cb([]() {
        log_w("堵转保护：执行停机（电机停 + 离合断）");
        AMCU_drive_stop();
    });

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
//  电机运动逻辑（带离合/缓冲 + Tianshu 到位检测）
// ============================
void AMCU_motion() {
    int cur = get_now_filament_num();
    if (cur < 0 || cur >= MAX_FILAMENT_NUM) cur = 0;  // 防止 0xFF 越界

    /* ★ 关键：结算上一笔挂起的离合请求（非阻塞，Web 不卡） */
    AMCU_clutch_pump();

    /* 手动点动进行中：总线归点动用，状态机让出
       （否则 waiting 分支每循环 AMCU_drive_stop 会把点动立刻停掉） */
    if (s_jog_active) {
        amcu_jog_pump();
        return;
    }

    /* 堵转保护 latch 期间禁止状态机再次驱动电机
       （否则 驱动→跳闸→停止→再驱动 无限循环；清除用 /current_clear） */
    if (current_sense_tripped()) {
        AMCU_drive_stop();
        RGB_RED;
        return;
    }

    /* 累计虚拟计米（时间法，本硬件无 AS5600） */
    if (s_driving_now && cur >= 0 && cur < MAX_FILAMENT_NUM) {
        accumulate_feed_meters(cur);
    }

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
        RGB_GREEN;
        clutch_assert_single(true);
        return;
    }

    switch (act) {
    case need_pull_back:
        RGB_GREEN;
        break;

    case need_send_out: {
        /* 进入送料前，记录"当前基准"（Tianshu 语义） */
        if (s_last_action[cur] != (uint8_t)need_send_out) {
            last_pullback_meters[cur] = get_filament_meters(cur);
            s_last_action[cur] = (uint8_t)need_send_out;
        }
        int ch = cur + 1;
        /* Tianshu 到位检测：若 target_meters > 0，检查是否已送到位 */
        if (target_meters > 0.0f &&
            (get_filament_meters(cur) - last_pullback_meters[cur]) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
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

    case act_send_mm: {
        /* 进入前记录基准 */
        if (s_last_action[cur] != (uint8_t)act_send_mm) {
            last_pullback_meters[cur] = get_filament_meters(cur);
            s_last_action[cur] = (uint8_t)act_send_mm;
        }
        /* Tianshu 到位检测 */
        if (target_meters > 0.0f &&
            (get_filament_meters(cur) - last_pullback_meters[cur]) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
        AMCU_drive_channel(cur + 1, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
        RGB_GREEN;
        break;
    }

    case act_pull_mm: {
        /* 进入前记录基准 */
        if (s_last_action[cur] != (uint8_t)act_pull_mm) {
            last_pullback_meters[cur] = get_filament_meters(cur);
            s_last_action[cur] = (uint8_t)act_pull_mm;
        }
        /* Tianshu 到位检测（退料方向） */
        if (target_meters > 0.0f &&
            (last_pullback_meters[cur] - get_filament_meters(cur)) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
        AMCU_drive_channel(cur + 1, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
        RGB_GREEN;
        break;
    }

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

    /* 离开当前动作时清除 last_action 标记 */
    if (act == waiting) {
        s_last_action[cur] = 0;
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
