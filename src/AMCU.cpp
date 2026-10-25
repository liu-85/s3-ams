/**
 * AMCU.cpp - ESP32-S3 主控单元（合并项目 s3-ams）
 *
 * 架构变更（对比原 Tianshu RP2040 版）：
 *   - 8 路独立电机 → 1 个共享电机 + 8 个电磁离合（互斥吸合）
 *   - 每路 1 微动（耗材在线）→ 每路 2 微动（缓冲器开始位/停止位，电阻编码 ADC）
 *   - 微动只在打印/流量校准时参与送料，AMS 进料时不参与
 *   - 离合只在送料/退料时吸合，且同一时刻只有 1 路吸合
 */

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
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

    /* ★★ v1.1.11 关键修复：单位错了 1000 倍 ★★
     *
     * 现场症状（2026-10-25 第二次实测）：
     *   送料 3 秒后打印机就发 before_on_use（09/A5）让挤出机拉料，
     *   拉一点就停 → 报「挤出失败」→ 立刻退料重试。
     *   抓包看我方 Dxx 应答里的 meters：3 秒就报 141.1 米，
     *   第二次更是 818.6 米（真实应约 0.15 米）。
     *
     * 根因：dt 是毫秒，FILAMENT_PUSH_M_PER_SEC 是「米/秒」，
     *   原写法 (float)dt * 0.05f 等于把 50ms 算成 2.5 米。
     *   打印机靠 meters 判断「AMS 送了多少料」，看到 141 米
     *   自然认为早就到位了 → 提前发 before_on_use → 挤出机拉空。
     *   （用户直觉「485 的消息给早了」完全正确。）
     *
     * 修正：先把毫秒换成秒，再乘米/秒。
     *   50ms → 0.0025 米 = 2.5mm，3 秒 → 0.15 米，符合 AMS lite 实际送料速度。 */
    if (dt > 500) return;  /* 主循环卡顿/暂停后不要一次补一大段 */

    float meters = ((float)dt / 1000.0f) * FILAMENT_PUSH_M_PER_SEC;
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

/* v1.1.9：挂起请求的类型/通道。
 *
 * 之前每次调用都无条件 clutch_request_*()，而 AMCU_drive_channel() 在
 * need_send_out 分支里是**每循环**调用的（主循环几百 Hz）：
 *   - 离合队列（8 深）瞬间塞满 → clutch_queue_push 走兜底分支，
 *     退化成在主循环里同步执行 clutch_engage()（内含 80ms+60ms vTaskDelay）
 *     → 主循环被卡、WEB 变慢、485 接收延迟
 *   - req_slot 只有 4 个槽，epoch 每请求 ++ → 句柄被反复回收，
 *     clutch_result_poll() 永远等不到完成
 * 修法：同一目标通道已有挂起请求时不再重复入队；换目标才重新请求。 */
static int         s_pending_kind = 0;   /* 0=无, 1=吸合, 3=全断 */
static int         s_pending_ch   = 0;

/* v1.1.15 诊断计数器（暴露到 /status 的 clutch.diag）：
 *   s_clutch_switch —— 挂起的吸合目标通道被改成别的通道的次数
 *   s_clutch_abort  —— 挂起的吸合请求被"停机"打断的次数
 *
 * 正常打印时这两个数应该很慢地涨（只在真正换料时各加 1）。
 * 如果"咔咔响"时每秒涨几十，就是两个模块在互抢同一个离合槽位 ——
 * 详见 find_motion_channel() 上方的注释。 */
static uint32_t    s_clutch_switch = 0;
static uint32_t    s_clutch_abort  = 0;

uint32_t amcu_clutch_switch_count(void) { return s_clutch_switch; }
uint32_t amcu_clutch_abort_count(void)  { return s_clutch_abort;  }
void     amcu_clutch_reset_diag(void)   { s_clutch_switch = 0; s_clutch_abort = 0; }

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
        s_pending_kind = 0;
        s_driving_now = true;
        s_last_drive_ms = millis();
    } else {
        /* 未吸合：异步发起，等后台任务完成（下一循环 poll 成功再驱动电机）
           ★ 同一通道已有挂起请求时不重复入队（否则队列被塞满 → 主循环被同步
             clutch_engage() 卡住 80ms+，WEB 卡顿、485 收包延迟） */
        if (s_pending_clutch == NULL || s_pending_kind != 1 || s_pending_ch != channel) {
            /* v1.1.15 诊断：本来在等另一个通道吸合，现在被改成这个通道 */
            if (s_pending_kind == 1 && s_pending_ch != 0 && s_pending_ch != channel) {
                s_clutch_switch++;
            }
            s_pending_clutch = clutch_request_engage(channel);
            s_pending_kind = 1;
            s_pending_ch = channel;
            /* 记住这次要跑的方向/速度，咬合后 pump 按它启动电机 */
            s_pending_dir = dir;
            s_pending_speed = speed_pct;
        }
        s_current_channel = channel;
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

    const bool done = clutch_result_poll(s_pending_clutch);

    if (s_pending_kind == 1) {
        /* ---- 吸合请求 ---- */
        int want = s_pending_ch;
        if (want < 1 || want > BOARD_CHANNEL_COUNT) {
            s_pending_clutch = NULL;
            s_pending_kind = 0;
            s_driving_now = false;
            return;
        }
        if (clutch_is_engaged(want)) {
            /* 离合已咬合 → 按挂起时记下的方向/速度启动电机
               （不能用 motor_get_dir()：电机还没启过，拿到的是 STOP） */
            motor_set_dir_speed(s_pending_dir, s_pending_speed);
            s_pending_clutch = NULL;
            s_pending_kind = 0;
            s_pending_dir = MOTOR_DIR_STOP;
            s_driving_now = true;
            s_last_drive_ms = millis();
        } else if (done) {
            /* 后台已处理完但电平仍不对（吸合失败）：放弃本笔，
               下一循环 AMCU_drive_channel() 会重新发起，避免死等 */
            s_pending_clutch = NULL;
            s_pending_kind = 0;
            s_driving_now = false;
        }
        /* 未完成：什么都不做，等下一循环（非阻塞，Web 不受影响） */
        return;
    }

    /* ---- 断开请求：完成或已确认全断就清掉挂起 ---- */
    if (done || clutch_engaged_mask() == 0) {
        s_pending_clutch = NULL;
        s_pending_kind = 0;
    }
    s_driving_now = false;
}

/**
 * 停止电机 + 异步释放所有离合
 *
 * ★ 幂等：本函数在 case waiting/default 里是**每循环**调用的。
 *   之前无条件 clutch_request_release_all()，队列瞬间塞满后
 *   clutch_queue_push 退化成主循环内同步 clutch_release_all()（60ms 延时）
 *   → 主循环被卡、WEB 卡、485 掉包。现在只有"确实还没断"时才入队。
 */
void AMCU_drive_stop()
{
    motor_stop();
    s_driving_now = false;
    s_current_channel = 0;
    s_pending_dir = MOTOR_DIR_STOP;

    const bool already_releasing = (s_pending_kind == 3 && s_pending_clutch != NULL);
    const bool already_off = (s_pending_clutch == NULL && clutch_engaged_mask() == 0);

    if (!already_releasing && !already_off) {
        /* v1.1.15 诊断：吸合请求还没完成就被停机打断 */
        if (s_pending_kind == 1) s_clutch_abort++;
        s_pending_clutch = clutch_request_release_all();
        s_pending_kind = 3;
        s_pending_ch = 0;
    }
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
//  before_on_use 跟随送料（v1.1.12）
// ============================
/* 现场（2026-10-25 第四次实测）：
 *
 *   送料到位 → 打印机发 09/A5（before_on_use），让挤出机咬住耗材往里拉
 *   → 我们这时把电机关了 → 打印机只拉得动一点点 → 5 秒后判「挤出失败」
 *   → 07/00 (stop_on_use) → 09/3F (before_pull_back) → 退料。
 *
 * 为什么"断开离合"不够：
 *   日志证明 09/A5 期间离合本来就是 0x00（已全断）、电机=停止、电流 0mA，
 *   耗材照样拉不动 —— 说明阻力来自 AMS 侧的压轮与主动轮/齿轮组，
 *   光断离合解决不了（离合只断开"电机↔主动轮"，压轮始终压着耗材）。
 *
 * 所以改成"电机跟随"：在 before_on_use 期间按脉冲送料，
 * 让 AMS 与挤出机之间的耗材保持松弛，打印机就能顺畅把料拉进喷嘴。
 *
 * 为什么不用缓冲微动来驱动：
 *   微动是 ADC 分压三态（START/MID/STOP），现场信号不稳（悬空 371mV 就落在
 *   START 区），而且 before_on_use 只持续几秒，等微动触发太慢。
 *   这里直接按固定节奏送，节奏与打印机拉料速度匹配即可。
 *
 * 脉冲参数：送 250ms / 停 350ms。
 *   100% PWM ≈ 50mm/s → 平均 50 × 250/600 ≈ 21mm/s，
 *   略高于打印机拉料速度（约 10~20mm/s），保证耗材不会被拉紧；
 *   停的 350ms 让缓冲器把多送的料吸收掉，不会顶进喷嘴。
 */
#define FOLLOW_FEED_ON_MS   250
#define FOLLOW_FEED_OFF_MS  350

static uint32_t s_follow_last_ms = 0;
static bool     s_follow_on      = false;
static int      s_follow_ch      = 0;

/* 跟随送料泵：每循环调用，内部按时间切换"送/停" */
static void follow_feed_pump(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) return;

    const uint32_t now = millis();

    /* 首次进入或换通道：从"送"开始，并重置相位 */
    if (s_follow_ch != channel) {
        s_follow_ch = channel;
        s_follow_on = true;
        s_follow_last_ms = now;
        AMCU_drive_channel(channel, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
        return;
    }

    const uint32_t span = s_follow_on ? FOLLOW_FEED_ON_MS : FOLLOW_FEED_OFF_MS;
    if (now - s_follow_last_ms < span) return;

    s_follow_last_ms = now;
    s_follow_on = !s_follow_on;

    if (s_follow_on) {
        AMCU_drive_channel(channel, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
    } else {
        /* 只停电机、保留离合（下一拍还要接着送，反复吸合会磨离合） */
        motor_stop();
        s_driving_now = false;
    }
}

/* 退出跟随送料。
 * stop_motor=false 用于"退料进行中"：此时电机正被退料逻辑独占反转，
 * 绝不能在这里 motor_stop()，否则就是现场那个"退料有时电机没动作"。 */
static void follow_feed_reset_ex(bool stop_motor)
{
    if (stop_motor && s_follow_ch != 0) {
        /* 退出跟随时如果最后停在"送"相位，电机还在转 —— 必须停掉。
           否则交回 on_use 的微动逻辑时，微动若处于 MID 且 decision=0，
           电机会一直转着不放，把耗材顶进喷嘴。 */
        motor_stop();
        s_driving_now = false;
    }
    s_follow_on      = false;
    s_follow_ch      = 0;
    s_follow_last_ms = 0;
}

/* 供 /status 诊断：跟随送料当前是否在"送"的相位 */
bool amcu_follow_feed_active(void) { return s_follow_ch != 0 && s_follow_on; }

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
//  通道仲裁（v1.1.15）
// ============================
/* 本机是「1 台电机 + 8 路电磁离合」，同一时刻只能服务 1 个通道。
 *
 * ★ 现场症状（2026-10-25 用户实测）：
 *   打印中换料，电磁离合"咔咔响"，料既退不出来也送不进去；
 *   停止打印后再点动该通道，离合也吸不上。
 *
 * ★ 根因：打印中换料时，两个模块同时抢同一个离合槽位 ——
 *   ① check_buffer_status() 看到旧通道还是 on_use，按缓冲微动
 *      AMCU_drive_channel(旧, FEED) → clutch_request_engage(旧)
 *   ② AMCU_motion() 看到新通道 need_send_out，
 *      switch 里又 AMCU_drive_channel(新, FEED) → clutch_request_engage(新)
 *   两者每循环交替，而 s_pending_clutch 只有**一个槽**（s_pending_ch 单值），
 *   于是目标通道每循环被改一次 → 后台任务反复"全断 60ms + 吸合 80ms"，
 *   听到的就是"咔咔响"；电机永远等不到稳定吸合，两个方向都不转。
 *
 * 另外 is_channel_printing(cur) 分支在 switch 之前就 return，
 * 若 cur 还指着旧通道，新通道的 need_send_out **永远执行不到**。
 *
 * ★ 解法：统一仲裁，优先级
 *      退料(need_pull_back / act_pull_mm)
 *    > 进料(need_send_out / act_send_mm)
 *    > 打印中缓冲补料
 *    > before_on_use 跟随泵
 *   两个调用点都先问一次 find_motion_channel()，有进/退料就整体让出。 */

/* 找出正在请求「进料 / 退料」的通道（0 起）。没有则 -1。
 * 退料优先于进料 —— 打印机换料一定是先退旧料再送新料，
 * 退料没走完就送新料会把耗材扯断。 */
static int find_motion_channel(void)
{
    int send = -1, pull = -1, send_mm = -1, pull_mm = -1;
    for (int i = 0; i < MAX_FILAMENT_NUM; i++) {
        _filament_motion_state_set m = get_filament_motion(i);
        if (m == need_pull_back)      { if (pull    < 0) pull    = i; }
        else if (m == need_send_out)  { if (send    < 0) send    = i; }
        else if (m == act_pull_mm)    { if (pull_mm < 0) pull_mm = i; }
        else if (m == act_send_mm)    { if (send_mm < 0) send_mm = i; }
    }
    if (pull    >= 0) return pull;
    if (send    >= 0) return send;
    if (pull_mm >= 0) return pull_mm;
    if (send_mm >= 0) return send_mm;
    return -1;
}

// ============================
//  微动状态检查（兼容 Tianshu 的 check_microswitch_status）
// ============================
void check_buffer_status() {
    /* ★★ v1.1.12：退料进行中，缓冲补料必须完全让出 ★★
     *
     * 现场症状：退料"有时候电机没动作"。
     * 根因：AMCU_buffer_assist_feed() 在缓冲器满（STOP 微动）时会调
     *   AMCU_drive_stop()，而它内部会 clutch_request_release_all()，
     *   **把正在挂起的"退料吸合"请求直接覆盖掉**（s_pending_clutch 只有一个槽），
     *   于是 AMCU_drive_channel() 的退料请求永远等不到吸合完成 → 电机不转。
     *   打印机在退料途中切换料位时（cur 指向正在打印的通道）最容易撞上。
     *
     * 退料优先级高于一切缓冲动作，这里直接整段跳过。 */
    if (is_need_pull_back) return;

    /* ★★ v1.1.15：只要有通道在「进料 / 退料」，缓冲补料整体让出 ★★
     *
     * 单电机 + 单离合槽位，缓冲补料和 485 进/退料**不能同时要电机**。
     * 之前只挡了 is_need_pull_back（全局退料标志），没挡 need_send_out，
     * 于是打印中换料时这里和 AMCU_motion() 的 switch 互抢离合槽位，
     * 离合反复吸放"咔咔响"，料退不出也送不进。 */
    if (find_motion_channel() >= 0) return;

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int filament_num = i;
        _filament_motion_state_set m = get_filament_motion(filament_num);
        if (m == need_send_out || m == need_pull_back) {
            continue;  // AMS 进料/退料：微动不参与
        }
        /* ★ v1.1.12：before_on_use 由 AMCU_motion() 的 follow_feed_pump()
           接管（固定脉冲跟随）。这里必须让出，否则两边交替驱动同一台电机，
           一边送一边停，现场就是"抖一下、抖一下"。 */
        if (is_channel_before_on_use(filament_num)) {
            continue;
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
            log_i("退料开始: 料位%d→通道%d PWM=%d%%",
                  now_f, now_f + 1, MOTOR_SPEED_FULL);
        }
    } else {
        /* ★★ v1.1.10 关键修复：离开退料状态必须清 latch ★★
         *
         * 现场症状（2026-10-25 实测）：
         *   第一次退料电机正常反转把料退出来；
         *   第二次退料 —— 状态机明明切到了 need_pull_back（/status 的
         *   feed.motion_text=need_pull_back），但电机=停止、离合=0x00、电流=0mA。
         *
         * 根因：s_last_need_pull_back[] 只在触发时置 true，
         *   全项目**没有任何地方把它清回 false**
         *   （本意是让 last_action_set() 去清，但那个函数定义了从未被调用）。
         *   于是第二次进 need_pull_back 时 !s_last_need_pull_back[now_f] 为假
         *   → is_need_pull_back 不置位 → AMCU_motion() 里那段
         *   "if (is_need_pull_back) { ...退料驱动... }" 整块被跳过 → 电机不动。
         *
         * 参考 Tianshu 原版：它判断的是 last_action[now_f] != need_pull_back，
         *   而 last_action[now_f] 在进入 waiting 时会被改写（第 419 行），
         *   所以天然可以反复触发。移植时换成独立布尔 latch 却漏了复位。 */
        if (s_last_need_pull_back[now_f]) {
            s_last_need_pull_back[now_f] = false;
        }

        if (state == need_send_out) {
            /* ★ 不要在这里写 s_last_action[now_f]：
               AMCU_motion() 的 case need_send_out 靠 "s_last_action != need_send_out"
               来判断"刚进入送料"，用它记录 last_pullback_meters 基准。
               这里抢先写成 need_send_out 会让基准永远不记录（恒 0），
               一旦 target_meters>0 就会立刻判"已到位"→ 送一点就停。 */
            RGB_GREEN;
        } else if (state == waiting) {
            s_last_action[now_f] = (uint8_t)waiting;
            RGB_BLUE;
        }
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

    /* ★★★ v1.1.15：服务通道仲裁 ★★★
     *
     * 不能盲目用 cur（打印机上报的"当前料位"）：
     *   打印中换料时 cur 往往还指着**旧通道**（它还是 on_use），
     *   而真正要动的电机是**新通道**的 need_send_out。
     *   原代码在 is_channel_printing(cur) 分支直接 return，
     *   新通道的进料永远执行不到 → "料送不进去"。
     *
     * 所以先问一次 find_motion_channel()：只要有通道在进/退料，
     * 本循环就服务它，打印态/缓冲补料全部让位。
     * 详见 find_motion_channel() 上方的长注释。 */
    int svc = cur;                                   /* 本循环真正服务的通道 */
    {
        int mc = find_motion_channel();
        if (mc >= 0) svc = mc;
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

            /* ★ 本轮退料结束 → 清该通道的 latch，允许它以后再退一次。
               只靠 AMCU_run 里"离开 need_pull_back 状态"来清是不够的：
               退料途中打印机可能已经切到别的料位（now_f 变了），
               那时 AMCU_run 清的是新料位的 latch，原通道的会一直留着 true。 */
            if (pull_back_channel >= 0 && pull_back_channel < MAX_FILAMENT_NUM) {
                s_last_need_pull_back[pull_back_channel] = false;
            }

            if (ams_enable && pull_back_channel >= 0) {
                set_filament_motion(pull_back_channel, waiting);
            }
            log_i("退料结束: 料位%d 已反转 %d 秒，停电机", pull_back_channel, timeout_seconds);
            RGB_BLUE;
        } else {
            int ch = now_filament_num_need_pull_back + 1;
            AMCU_drive_channel(ch, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
            RGB_GREEN;
        }
    }

    if (ams_enable) act = get_filament_motion(svc);

    /* ★★★ v1.1.12：退料优先级最高，跟随泵必须完全让出 ★★★
     *
     * 现场症状：退料"有时候电机没动作"（不是每次）。
     * 触发条件：打印机退料途中切换料位 → cur 指向另一个正在打印的通道 →
     *   走进下面的"打印中"分支 → 跟随泵 reset 里 motor_stop() 把
     *   正在反转的退料电机掐掉；或者 check_buffer_status 的 drive_stop()
     *   把挂起的退料吸合请求覆盖掉。
     * 所以这里先判 is_need_pull_back，退料期间只清跟随泵状态、绝不动电机。
     *
     * ★ v1.1.15：判断对象从 cur 改成 svc（仲裁后的服务通道）。
     *   否则打印中换料时 cur 还指着旧通道（on_use），
     *   会走 "打印中" 分支 return，新通道的 need_send_out 永远执行不到。 */
    if (is_need_pull_back) {
        follow_feed_reset_ex(false);   /* 只清状态，不停电机 */
    } else if (is_channel_printing(svc)) {
        RGB_GREEN;
        clutch_assert_single(true);

        /* ★ v1.1.12：before_on_use 阶段必须"跟随送料"。
         *
         * 打印机发 09/A5 后，挤出机已经咬住耗材在往里拉，
         * 这时如果我们把电机停死，耗材会被 AMS 侧压轮+齿轮组拽住，
         * 打印机只拉得动一点点就判「挤出失败」→ 退料。
         * 现场实测：09/A5 期间离合本来就是 0x00、电机停止、电流 0mA，
         * 说明光"断开离合"没用（离合只管电机↔主动轮，压轮始终压着耗材）。
         *
         * 所以这里按固定节奏脉冲送料，让耗材保持松弛。
         * 详见 follow_feed_pump() 的注释。 */
        if (is_channel_before_on_use(svc)) {
            follow_feed_pump(svc + 1);
        } else {
            /* on_use / stop_on_use：交回 check_buffer_status 的微动逻辑 */
            follow_feed_reset_ex(true);
        }
        return;
    } else {
        follow_feed_reset_ex(true);
    }

    switch (act) {
    case need_pull_back:
        RGB_GREEN;
        break;

    case need_send_out: {
        /* 进入送料前，记录"当前基准"（Tianshu 语义）。
           entered 只在"刚进入送料"这一个周期为真 → 下面的日志只打一次，
           不会刷屏，但能在网页「设备日志」里直接看到这一轮送料是启动还是被挡。 */
        const bool entered = (s_last_action[svc] != (uint8_t)need_send_out);
        if (entered) {
            last_pullback_meters[svc] = get_filament_meters(svc);
            s_last_action[svc] = (uint8_t)need_send_out;
        }
        int ch = svc + 1;
        /* Tianshu 到位检测：若 target_meters > 0，检查是否已送到位 */
        if (target_meters > 0.0f &&
            (get_filament_meters(svc) - last_pullback_meters[svc]) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
        if (is_filament_online(svc)) {
            bool driving = AMCU_drive_channel(ch, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
            if (entered) {
                log_i("送料开始: 料位%d→通道%d PWM=%d%% 电机%s",
                      svc, ch, MOTOR_SPEED_485_FEED,
                      driving ? "已启动" : "等离合吸合后启动");
            }
            RGB_GREEN;
        } else {
            if (entered) {
                log_w("送料被挡: 料位%d 被判为离线(online=0)，不驱动电机", svc);
            }
            AMCU_drive_stop();
            RGB_BLUE;
        }
        break;
    }

    case waiting:
        /* ★ 退料进行中不要停电机（对应 Tianshu 原版
           `if (!is_need_pull_back) switchInterrupt();`）：
           打印机在退料途中切换当前料位时 cur 会变成别的通道，
           那个通道的 act 是 waiting → 这里 AMCU_drive_stop() 会把
           正在反转的退料电机掐掉，表现为"退料只转了一下就停"。 */
        if (!is_need_pull_back) {
            AMCU_drive_stop();
        }
        RGB_BLUE;
        break;

    case act_send_mm: {
        /* 进入前记录基准 */
        if (s_last_action[svc] != (uint8_t)act_send_mm) {
            last_pullback_meters[svc] = get_filament_meters(svc);
            s_last_action[svc] = (uint8_t)act_send_mm;
        }
        /* Tianshu 到位检测 */
        if (target_meters > 0.0f &&
            (get_filament_meters(svc) - last_pullback_meters[svc]) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
        AMCU_drive_channel(svc + 1, MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
        RGB_GREEN;
        break;
    }

    case act_pull_mm: {
        /* 进入前记录基准 */
        if (s_last_action[svc] != (uint8_t)act_pull_mm) {
            last_pullback_meters[svc] = get_filament_meters(svc);
            s_last_action[svc] = (uint8_t)act_pull_mm;
        }
        /* Tianshu 到位检测（退料方向） */
        if (target_meters > 0.0f &&
            (last_pullback_meters[svc] - get_filament_meters(svc)) > target_meters) {
            AMCU_drive_stop();
            RGB_BLUE;
            act = waiting;
            break;
        }
        AMCU_drive_channel(svc + 1, MOTOR_DIR_RETRACT, MOTOR_SPEED_FULL);
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
        if (!is_need_pull_back) {
            AMCU_drive_stop();
        }
        RGB_BLUE;
        break;
    }

    /* 离开当前动作时清除 last_action 标记 */
    if (act == waiting) {
        s_last_action[svc] = 0;
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
