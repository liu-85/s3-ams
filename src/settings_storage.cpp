/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
#include "settings_storage.h"
#include <Preferences.h>

// 外部变量定义
int PULLBACK_TIMEOUT[16];
int SLEEP_TIMEOUT_MIN;
bool SCREEN_ALWAYS_ON;
int CHANNEL_MODE[16];
int AMS_COUNT;

/* ★ v1.1.26：自动送料 PWM（独立 NVS 命名空间，见 settings_storage.h 注释） */
int FEED_PWM = DEFAULT_FEED_PWM;
#define FEED_PWM_NVS_NS  "s3ams-feed"
#define FEED_PWM_NVS_KEY "feed_pwm"

static Preferences s_settings_prefs;
#define SETTINGS_KEY "settings"

// 计算校验和
static uint32_t calculate_checksum(const SettingsData* data) {
    uint32_t checksum = 0;
    for (int i = 0; i < 16; i++) {
        checksum += (uint32_t)data->pullback_timeout[i] * (i + 1);
        checksum += (uint32_t)data->channel_mode[i] * (i + 17);
    }
    checksum += (uint32_t)data->sleep_timeout_min * 100;
    checksum += (uint32_t)data->screen_always_on * 10000;
    checksum += (uint32_t)data->ams_count * 1000;
    return checksum;
}

/* ★ v1.1.26：自动送料 PWM 的读写（独立命名空间，不参与 SettingsData 校验） */
static void feed_pwm_load(void) {
    Preferences p;
    if (p.begin(FEED_PWM_NVS_NS, true)) {
        int v = p.getInt(FEED_PWM_NVS_KEY, DEFAULT_FEED_PWM);
        p.end();
        FEED_PWM = (v >= FEED_PWM_MIN && v <= FEED_PWM_MAX) ? v : DEFAULT_FEED_PWM;
    } else {
        FEED_PWM = DEFAULT_FEED_PWM;
    }
}

int settings_set_feed_pwm(int pct) {
    if (pct < FEED_PWM_MIN || pct > FEED_PWM_MAX) return -1;
    FEED_PWM = pct;
    Preferences p;
    if (p.begin(FEED_PWM_NVS_NS, false)) {
        p.putInt(FEED_PWM_NVS_KEY, pct);
        p.end();
    }
    return pct;
}

int settings_get_feed_pwm(void) {
    return FEED_PWM;
}

void settings_storage_init() {
    /* 独立命名空间，必须最先加载 —— 下面 !valid 分支会 return，
       放在那里就永远轮不到它了。 */
    feed_pwm_load();

    SettingsData data;

    s_settings_prefs.begin("s3ams", true);  // 只读
    size_t len = s_settings_prefs.getBytesLength(SETTINGS_KEY);

    bool valid = false;
    if (len == sizeof(SettingsData)) {
        s_settings_prefs.getBytes(SETTINGS_KEY, &data, sizeof(SettingsData));
        if (data.magic == SETTINGS_MAGIC_NUMBER &&
            data.checksum == calculate_checksum(&data)) {
            valid = true;
        }
    }
    s_settings_prefs.end();

    if (!valid) {
        settings_reset_to_defaults();
        settings_save_to_flash();
        return;
    }

    // 加载值（带范围检查）
    for (int i = 0; i < 16; i++) {
        if (data.pullback_timeout[i] >= 3 && data.pullback_timeout[i] <= 30) {
            PULLBACK_TIMEOUT[i] = data.pullback_timeout[i];
        } else {
            PULLBACK_TIMEOUT[i] = DEFAULT_PULLBACK_TIMEOUT;
        }

        if (data.channel_mode[i] >= 0 && data.channel_mode[i] <= 2) {
            CHANNEL_MODE[i] = data.channel_mode[i];
        } else {
            CHANNEL_MODE[i] = CHANNEL_MODE_OFFLINE;
        }
    }

    if (data.sleep_timeout_min >= 1 && data.sleep_timeout_min <= 16) {
        SLEEP_TIMEOUT_MIN = data.sleep_timeout_min;
    } else {
        SLEEP_TIMEOUT_MIN = DEFAULT_SLEEP_TIMEOUT_MIN;
    }

    SCREEN_ALWAYS_ON = (data.screen_always_on != 0);

    /* ★ v1.1.21：NVS 里可能还存着旧值 4（幽灵 AMS-C/D 的来源）。
       上限收到 MAX_AMS_COUNT(2)，旧值不合法 → 回落默认 2。 */
    if (data.ams_count >= 1 && data.ams_count <= MAX_AMS_COUNT) {
        AMS_COUNT = data.ams_count;
    } else {
        AMS_COUNT = DEFAULT_AMS_COUNT;
    }
}

void settings_load_from_flash() {
    settings_storage_init();
}

void settings_save_to_flash() {
    SettingsData data;

    for (int i = 0; i < 16; i++) {
        data.pullback_timeout[i] = PULLBACK_TIMEOUT[i];
        data.channel_mode[i] = CHANNEL_MODE[i];
    }
    data.sleep_timeout_min = SLEEP_TIMEOUT_MIN;
    data.screen_always_on = SCREEN_ALWAYS_ON ? 1 : 0;
    data.ams_count = AMS_COUNT;
    data.checksum = calculate_checksum(&data);
    data.magic = SETTINGS_MAGIC_NUMBER;

    s_settings_prefs.begin("s3ams", false);  // 读写
    s_settings_prefs.putBytes(SETTINGS_KEY, &data, sizeof(SettingsData));
    s_settings_prefs.end();
}

void settings_reset_to_defaults() {
    for (int i = 0; i < 16; i++) {
        PULLBACK_TIMEOUT[i] = DEFAULT_PULLBACK_TIMEOUT;
        CHANNEL_MODE[i] = CHANNEL_MODE_OFFLINE;
    }
    SLEEP_TIMEOUT_MIN = DEFAULT_SLEEP_TIMEOUT_MIN;
    SCREEN_ALWAYS_ON = DEFAULT_SCREEN_ALWAYS_ON;
    AMS_COUNT = DEFAULT_AMS_COUNT;
}

bool settings_is_valid() {
    SettingsData data;
    s_settings_prefs.begin("s3ams", true);
    size_t len = s_settings_prefs.getBytesLength(SETTINGS_KEY);
    bool valid = false;
    if (len == sizeof(SettingsData)) {
        s_settings_prefs.getBytes(SETTINGS_KEY, &data, sizeof(SettingsData));
        if (data.magic == SETTINGS_MAGIC_NUMBER &&
            data.checksum == calculate_checksum(&data)) {
            valid = true;
        }
    }
    s_settings_prefs.end();
    return valid;
}

int get_motor_pullback_timeout(int motor_index) {
    if (motor_index >= 0 && motor_index < 16) {
        return PULLBACK_TIMEOUT[motor_index];
    }
    return DEFAULT_PULLBACK_TIMEOUT;
}

int settings_set_pullback_timeout_all(int sec) {
    /* 合法范围与加载校验一致（settings_storage.cpp:54 的 3~30）。
       本硬件是单电机 + 8 离合，全局一个值即可，这里统一写 16 路。 */
    if (sec < 3 || sec > 30) return -1;
    for (int i = 0; i < 16; i++) {
        PULLBACK_TIMEOUT[i] = sec;
    }
    settings_save_to_flash();
    return sec;
}

int settings_get_pullback_timeout_common(void) {
    /* 各通道通常一致（由 settings_set_pullback_timeout_all 统一写过）。
       返回通道 0 的值作为代表；万一不一致也安全（不崩溃）。 */
    return PULLBACK_TIMEOUT[0];
}
