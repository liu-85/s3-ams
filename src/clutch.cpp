/**
 * clutch.cpp —— 电磁离合的实现（通道数由 BOARD_CHANNEL_COUNT 决定）
 *
 * 从 esp32s3-485/main/clutch.c 移植到 Arduino 框架。
 * 核心是 clutch_engage()，把"吸合前先全部断开"做成无条件。
 */

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
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

/* ★ v1.1.9：引脚回读能力标记
 *
 * 踩过的坑：ESP-IDF 的 gpio_config(GPIO_MODE_OUTPUT) 只打开输出驱动，
 * 会把输入缓冲关掉（IO_MUX 的 FUN_IE=0）。此时 gpio_get_level() 读回来的
 * 恒为 0 —— 于是 clutch_engaged_mask() 永远返回 0：
 *   - clutch_is_engaged() 永远 false → AMCU_drive_channel() 永远走
 *     "未吸合，异步发起"分支 → 电机永远不被 motor_set_dir_speed() 启动
 *   - 现场症状：送料时离合"咔"一声响了，电机却一点不动、电流 0mA
 * 修法：配置成 GPIO_MODE_INPUT_OUTPUT（输出 + 输入缓冲都开），
 *      并在 init 里做一次"写吸合电平读回"自检；
 *      万一自检失败（不同 IDF 版本行为差异），退回软件记账 s_active。 */
static bool     s_readback_ok = true;

static inline uint8_t s_sw_mask(void)
{
    if (s_active >= 1 && s_active <= BOARD_CHANNEL_COUNT) {
        return (uint8_t)(1u << (s_active - 1));
    }
    return 0;
}

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
    /* 回读不可信时用软件记账兜底（否则整条送料链会因 mask=0 而永不启动电机） */
    if (!s_readback_ok && mask == 0) {
        return s_sw_mask();
    }
    return mask;
}

bool clutch_readback_ok(void)
{
    return s_readback_ok;
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
            /* ★ 必须是 INPUT_OUTPUT：OUTPUT-only 会关掉输入缓冲，
               gpio_get_level() 恒 0，导致离合状态永远读不到（详见文件头注释） */
            .mode         = GPIO_MODE_INPUT_OUTPUT,
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

    /* 引脚回读自检：写一次"吸合"电平立刻读回（只加电 ~1ms，不会真的吸合） */
    if (bad < BOARD_CHANNEL_COUNT) {
        gpio_set_level((gpio_num_t)s_pins[0], s_active_level);
        vTaskDelay(pdMS_TO_TICKS(1));
        s_readback_ok = (gpio_get_level((gpio_num_t)s_pins[0]) == s_active_level);
        gpio_set_level((gpio_num_t)s_pins[0], s_inactive_level);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!s_readback_ok) {
        log_e("★ 离合引脚回读不可信（GPIO_MODE_INPUT_OUTPUT 仍读不到吸合电平）"
              " —— 改用软件记账判断吸合状态");
    }

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

    log_i("%d 路电磁离合已初始化（高电平吸合，吸合等待%ums，回读%s）%s",
          BOARD_CHANNEL_COUNT, (unsigned)CLUTCH_ENGAGE_MS,
          s_readback_ok ? "正常" : "不可信(用软件记账)",
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

    /* 5) 吸合目标（先记账再等机械稳定：回读兜底模式要靠 s_active 判状态） */
    s_active = channel;
    clutch_apply_one(index, true, true);

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

/* ==========================================================================
 * 异步离合引擎
 *
 * 设计动机（修 "网页超级慢"）：
 *   原 clutch_engage() 里 vTaskDelay(80ms/60ms) 是**阻塞主任务**的。
 *   Arduino 框架下 setup/loop 就是主任务，而 web_server_handle()
 *   （HTTP 请求处理）也在主循环里跑。每切换一次离合主循环卡 80ms，
 *   叠加"每个循环都重新 engage" → Web 请求处理被饿死，页面卡死。
 *
 * 解法：
 *   - 主循环只调 clutch_request_*()：入队 + 立即返回句柄（微秒级，不阻塞）
 *   - 独立 FreeRTOS 任务 clutch_task 在后台串行执行离合动作，
 *     时序延迟（vTaskDelay）发生在它身上，不占主任务
 *   - 主循环用 clutch_result_poll() 非阻塞查询是否完成
 *
 * 一致性：同一时刻仍只允许 1 路吸合（后台任务串行 + 4 层保障不变）。
 * ========================================================================== */

/* 单个挂起请求（clutch_task 串行消费） */
struct clutch_pending {
    int kind;   /* 1=engage, 2=release, 3=release_all */
    int channel;/* engage/release 用，1~8 */
    uint32_t epoch; /* 对应的主循环请求 epoch（clutch_task 完成后精确标记该槽） */
};

static SemaphoreHandle_t s_req_lock = NULL;
static clutch_pending s_pending[8];
static int s_pending_head = 0, s_pending_tail = 0;  /* 环形 */
static SemaphoreHandle_t s_req_notify = NULL;        /* 任务等待/唤醒 */
static TaskHandle_t s_clutch_task = NULL;

/* 给请求句柄用的池子（主循环侧，避免栈上指针失效） */
#define CLUTCH_REQ_SLOTS 4
static clutch_req_t s_req_slots[CLUTCH_REQ_SLOTS];
static uint32_t s_req_epoch = 0;   /* 单调递增序号：每次请求 ++ */

/* 取一个新序号槽位：用单调 epoch 保证每个请求 token 唯一，
 * 后台任务只会把"恰好等于该 epoch"的槽标记完成，避免误标别人的请求。 */
static clutch_req_t *req_slot_alloc(void)
{
    s_req_epoch++;
    int idx = (int)(s_req_epoch % CLUTCH_REQ_SLOTS);
    s_req_slots[idx].request = 0;
    s_req_slots[idx].seq = s_req_epoch;
    s_req_slots[idx].status = 1;  /* 进行中 */
    return &s_req_slots[idx];
}

/* 推一个请求进环形队列（非阻塞；满则直接后台执行，兜底） */
static bool clutch_queue_push(int kind, int channel)
{
    if (s_req_lock != NULL) {
        if (xSemaphoreTake(s_req_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
            /* 极端竞争：直接后台执行（可能阻塞一次，仅在锁竞争时） */
            if (kind == 1) clutch_engage(channel);
            else if (kind == 2) clutch_release(channel);
            else clutch_release_all();
            return false;
        }
    }
    int next = (s_pending_head + 1) % 8;
    if (next == s_pending_tail) {
        if (s_req_lock != NULL) xSemaphoreGive(s_req_lock);
        /* 队列满：兜底直接执行（会阻塞，仅发生在高负载时） */
        if (kind == 1) clutch_engage(channel);
        else if (kind == 2) clutch_release(channel);
        else clutch_release_all();
        return false;
    }
    s_pending[s_pending_head].kind = kind;
    s_pending[s_pending_head].channel = channel;
    s_pending[s_pending_head].epoch = s_req_epoch;  /* 当前最新请求 */
    s_pending_head = next;
    if (s_req_lock != NULL) xSemaphoreGive(s_req_lock);

    if (s_clutch_task != NULL && s_req_notify != NULL) {
        xSemaphoreGive(s_req_notify);
    }
    return true;
}

/* 请求入口（立即返回句柄） */
clutch_req_t *clutch_request_engage(int channel)
{
    clutch_req_t *r = req_slot_alloc();
    r->request = (channel >= 1 && channel <= BOARD_CHANNEL_COUNT) ? channel : 0;
    clutch_queue_push(1, channel);
    return r;
}

clutch_req_t *clutch_request_release(int channel)
{
    clutch_req_t *r = req_slot_alloc();
    r->request = (channel >= 1 && channel <= BOARD_CHANNEL_COUNT) ? channel : 0;
    clutch_queue_push(2, channel);
    return r;
}

clutch_req_t *clutch_request_release_all(void)
{
    clutch_req_t *r = req_slot_alloc();
    r->request = 0;
    clutch_queue_push(3, 0);
    return r;
}

/* 非阻塞查询：看本请求(epoch)是否已被后台任务处理完成 */
bool clutch_result_poll(const clutch_req_t *req)
{
    if (!req) return false;
    /* 任务完成会把 status 置 2/3（且仅对该 epoch 的槽） */
    return (req->status == 2 || req->status == 3);
}

/* 后台离合任务：串行消费队列，时序延迟发生在这里（不影响主任务） */
static void clutch_task_fn(void *arg)
{
    (void)arg;
    /* 低优先级 + 独立任务，吸收所有 vTaskDelay */
    for (;;) {
        if (s_req_notify != NULL) {
            xSemaphoreTake(s_req_notify, portMAX_DELAY);
        }

        /* 取一个待处理请求（注意：可能为 0 个，空转唤醒时） */
        for (;;) {
            int kind, channel;
            uint32_t epoch;
            if (s_req_lock != NULL &&
                xSemaphoreTake(s_req_lock, 0) != pdTRUE) break;

            if (s_pending_head == s_pending_tail) {
                if (s_req_lock != NULL) xSemaphoreGive(s_req_lock);
                break;  /* 队列空，回到阻塞等待 */
            }
            kind = s_pending[s_pending_tail].kind;
            channel = s_pending[s_pending_tail].channel;
            epoch = s_pending[s_pending_tail].epoch;
            s_pending_tail = (s_pending_tail + 1) % 8;
            if (s_req_lock != NULL) xSemaphoreGive(s_req_lock);

            /* 执行（内部 vTaskDelay 在本任务上，不卡主循环） */
            bool ok = true;
            switch (kind) {
            case 1: ok = clutch_engage(channel); break;
            case 2: ok = clutch_release(channel); break;
            case 3: clutch_release_all(); ok = true; break;
            default: ok = true; break;
            }

            /* 精确标记该 epoch 对应的槽完成（避免误标别人的请求） */
            for (int i = 0; i < CLUTCH_REQ_SLOTS; i++) {
                if (s_req_slots[i].seq == epoch && s_req_slots[i].status == 1) {
                    s_req_slots[i].status = ok ? 2 : 3;
                }
            }
        }
    }
}

void clutch_task_start(void)
{
    if (s_clutch_task != NULL) return;  /* 幂等 */

    if (s_req_lock == NULL) s_req_lock = xSemaphoreCreateMutex();
    if (s_req_notify == NULL) s_req_notify = xSemaphoreCreateBinary();

    xTaskCreate(
        clutch_task_fn, "clutch_task", 2048, NULL,
        2,  /* 略低于主任务(通常5)：主任务/Web 优先，离合后台执行时序 */
        &s_clutch_task);

    log_i("离合后台任务已启动（优先级 2，非阻塞主循环）");
}
