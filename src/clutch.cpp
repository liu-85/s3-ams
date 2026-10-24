/**
 * clutch.cpp —— 电磁离合的实现（通道数由 BOARD_CHANNEL_COUNT 决定）
 *
 * 从 esp32s3-485/main/clutch.c 移植到 Arduino 框架。
 * 核心是 clutch_engage()，把"吸合前先全部断开"做成无条件。
 */

#include "clutch.h"
#include "board_pins.h"
#include "motor.h"

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

/* ==========================================================================
 * 内部状态
 * ========================================================================== */
static const int s_pins[BOARD_CHANNEL_COUNT] = BOARD_PIN_CLUTCH_ARRAY;
static const int s_active_level   = BOARD_CLUTCH_ACTIVE_LEVEL;
static const int s_inactive_level = 1 - BOARD_CLUTCH_ACTIVE_LEVEL;

static SemaphoreHandle_t s_lock = NULL;

static int      s_active = 0;          /* 软件记账：当前吸合通道（0=全断） */
static const char *s_owner = NULL;     /* 当前占用者名 */
static bool     s_busy = false;        /* 是否处于复合动作中 */
static uint32_t s_conflicts = 0;       /* 累计违规次数 */

/* ==========================================================================
 * 底层：唯一碰引脚的地方
 * ========================================================================== */

static inline void clutch_apply_one(int index, bool on, bool settle)
{
    gpio_set_level((gpio_num_t)s_pins[index],
                   on ? s_active_level : s_inactive_level);
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(on ? CLUTCH_ENGAGE_MS : CLUTCH_RELEASE_MS));
    }
}

static void clutch_apply_all_off(bool settle)
{
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        clutch_apply_one(i, false, false);
    }
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(CLUTCH_RELEASE_MS));
    }
    s_active = 0;
    s_owner = NULL;
}

/* ==========================================================================
 * 查询
 * ========================================================================== */

uint8_t clutch_engaged_mask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (gpio_get_level((gpio_num_t)s_pins[i]) == s_active_level) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

bool clutch_is_engaged(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) return false;
    return (clutch_engaged_mask() & (1u << (channel - 1))) != 0;
}

int clutch_active_channel(void)
{
    return s_active;
}

void clutch_set_owner(const char *owner)
{
    s_owner = owner;
}

const char *clutch_get_owner(void)
{
    return s_owner ? s_owner : "";
}

void clutch_set_busy(bool busy)
{
    s_busy = busy;
}

bool clutch_is_busy(void)
{
    return s_busy;
}

uint32_t clutch_conflict_count(void)
{
    return s_conflicts;
}

void clutch_reset_conflict_count(void)
{
    s_conflicts = 0;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

bool clutch_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }

    int bad = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) {
            log_e("电磁离合%d(GPIO%d) 配置失败: %s —— 该路跳过", i + 1, s_pins[i], esp_err_to_name(err));
            bad++;
            continue;
        }
        gpio_set_level((gpio_num_t)s_pins[i], s_inactive_level);
    }

    s_active = 0;
    s_owner = NULL;
    s_busy = false;
    s_conflicts = 0;
    clutch_apply_all_off(false);

    /* 上电回读自检 */
    {
        uint8_t stuck = clutch_engaged_mask();
        if (stuck) {
            char det[64] = {0};
            int o = 0;
            for (int i = 0; i < BOARD_CHANNEL_COUNT && o < 52; i++) {
                if (stuck & (1u << i)) {
                    o += snprintf(det + o, sizeof(det) - o, "%s%d(GPIO%d)",
                                  o ? "," : "", i + 1, s_pins[i]);
                }
            }
            log_e("★ 离合上电自检异常：通道 %s 写入「断开」后仍读到「吸合」电平", det);
        }
    }

    log_i("%d 路电磁离合已初始化（高电平吸合，吸合等待%ums）%s",
          BOARD_CHANNEL_COUNT, (unsigned)CLUTCH_ENGAGE_MS,
          bad ? "  ★ 有脚配置失败" : "");
    return (bad < BOARD_CHANNEL_COUNT);
}

/* ==========================================================================
 * 核心：互斥吸合
 * ========================================================================== */

bool clutch_engage(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        log_e("离合通道 %d 不存在，可用 1~%d", channel, BOARD_CHANNEL_COUNT);
        return false;
    }
    if (s_lock == NULL) return false;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        log_e("切换离合超时：有别的动作正占着总线");
        return false;
    }

    const int index = channel - 1;

    /* 同通道重复吸合是幂等的 */
    if (s_active == channel &&
        gpio_get_level((gpio_num_t)s_pins[index]) == s_active_level) {
        xSemaphoreGive(s_lock);
        return true;
    }

    /* 1) 电机必须先停 */
    motor_stop();

    /* 2) 物理层：先全部断开 */
    bool had_engaged = (clutch_engaged_mask() != 0);
    clutch_apply_all_off(false);

    /* 3) 等上一路彻底脱开 */
    if (had_engaged) {
        vTaskDelay(pdMS_TO_TICKS(CLUTCH_RELEASE_MS + CLUTCH_SETTLE_MS));
    }

    /* 4) 复核 */
    uint8_t still = clutch_engaged_mask();
    if (still != 0) {
        s_conflicts++;
        clutch_apply_all_off(false);
        xSemaphoreGive(s_lock);
        log_e("吸合通道 %d 前检测到残留吸合，已强制全断开并取消（累计第 %u 次）", channel, (unsigned)s_conflicts);
        return false;
    }

    /* 5) 吸合目标 */
    clutch_apply_one(index, true, true);
    s_active = channel;

    xSemaphoreGive(s_lock);
    return true;
}

bool clutch_release(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) return false;
    if (!clutch_is_engaged(channel)) return true;
    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        return false;
    }

    if (s_active != channel) {
        s_conflicts++;
        log_e("通道 %d 吸合但未被总线记录，已强制全断开", channel);
    }
    motor_stop();
    clutch_apply_all_off(true);

    xSemaphoreGive(s_lock);
    return true;
}

void clutch_release_all(void)
{
    motor_stop();
    if (s_lock != NULL && xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) == pdTRUE) {
        clutch_apply_all_off(false);
        s_busy = false;
        xSemaphoreGive(s_lock);
    } else {
        clutch_apply_all_off(false);
        s_busy = false;
    }
}

void clutch_release_all_settled(void)
{
    motor_stop();
    if (s_lock != NULL && xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) == pdTRUE) {
        clutch_apply_all_off(true);
        s_busy = false;
        xSemaphoreGive(s_lock);
    } else {
        clutch_apply_all_off(true);
        s_busy = false;
    }
}

/* ==========================================================================
 * 运行期体检
 * ========================================================================== */

bool clutch_assert_single(bool force_release)
{
    uint8_t mask = clutch_engaged_mask();
    int count = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (mask & (1u << i)) count++;
    }

    if (count > 1) {
        s_conflicts++;
        log_e("体检发现 %d 路电磁离合同时吸合，已处置", count);
        if (force_release) clutch_release_all();
        return false;
    }

    int actual = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (mask & (1u << i)) {
            actual = i + 1;
            break;
        }
    }
    if (s_active != actual) {
        s_active = actual;
        if (actual == 0) s_owner = NULL;
    }
    return true;
}
