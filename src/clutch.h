/**
 * clutch.h —— 电磁离合（带互斥仲裁）
 *
 * 从 esp32s3-485/main/clutch.h 移植到 Arduino 框架。
 *
 * 核心硬约束：任何时刻最多只能有 1 路电磁离合吸合。
 * 四重保障：
 *   1. 物理层前置断开：吸合前无条件全断开
 *   2. 吸合前状态复核：写完全断后回读，有残留则取消
 *   3. 直接调用拦截：写引脚只通过内部函数，上层只能走 clutch_engage()
 *   4. 运行期体检：clutch_assert_single() 随时检查
 */

#ifndef CLUTCH_H
#define CLUTCH_H

#include <Arduino.h>

/* 时序参数 */
#define CLUTCH_ENGAGE_MS   80   /* 吸合后机械稳定等待 */
#define CLUTCH_RELEASE_MS  60   /* 断开后机械稳定等待 */
#define CLUTCH_SETTLE_MS   20   /* 切换通道额外静默时间 */

/* 错误码（避免与 ESP-IDF 的 ESP_ERR_* 宏冲突） */
#define ESP_ERR_CLUTCH_CHANNEL  0x7001
#define ESP_ERR_CLUTCH_CONFLICT 0x7002

/* 初始化 8 路离合 */
bool clutch_init(void);

/* 吸合指定通道（1 起），强制断开其他所有通道 */
bool clutch_engage(int channel);

/* 断开指定通道 */
bool clutch_release(int channel);

/* 断开全部离合（总是安全） */
void clutch_release_all(void);

/* 断开全部离合并等待机械脱开 */
void clutch_release_all_settled(void);

/* 当前吸合通道号；0 = 全部断开 */
int clutch_active_channel(void);

/* 实际吸合掩码（bit0 = 通道1），读引脚 */
uint8_t clutch_engaged_mask(void);

/* 引脚回读是否可信（v1.1.9 加：不可信时 clutch_engaged_mask 用软件记账兜底） */
bool clutch_readback_ok(void);

/* 某通道是否吸合（读引脚） */
bool clutch_is_engaged(int channel);

/* 累计违规次数 */
uint32_t clutch_conflict_count(void);

/* 清零违规计数 */
void clutch_reset_conflict_count(void);

/* 运行期体检：确认同时最多 1 路吸合 */
bool clutch_assert_single(bool force_release);

/* 设置/获取当前占用总线动作名 */
void clutch_set_owner(const char *owner);
const char *clutch_get_owner(void);

/* 设置/获取忙状态 */
void clutch_set_busy(bool busy);
bool clutch_is_busy(void);

/* ==========================================================================
 * 异步离合引擎（非阻塞，避免 vTaskDelay 卡死主任务 / Web 服务器）
 *
 * 背景：原 clutch_engage() 内的 vTaskDelay(80ms) 会阻塞 Arduino 主任务，
 *       而 Web 的 web_server_handle() 也在主循环里跑 → 每换一次离合卡 80ms，
 *       表现为 "网页超级慢"。
 * 解法：请求入队 → 独立任务 (clutch_task) 后台执行并吸收时序延迟；
 *       主循环只调 clutch_request_* 拿一个立即返回的句柄，稍后用
 *       clutch_result_poll() 非阻塞查询结果。
 * ========================================================================== */

typedef struct {
    volatile int request;   /* 目标通道(1~8)；0=断开全部；-1=无待执行 */
    volatile int seq;       /* 当前执行序号（clutch_task 完成后 ++） */
    volatile int status;    /* 0=空闲/无任务, 1=进行中, 2=完成, 3=失败 */
} clutch_req_t;

/* 请求吸合指定通道（非阻塞，立即返回句柄） */
clutch_req_t *clutch_request_engage(int channel);

/* 请求断开指定通道（非阻塞） */
clutch_req_t *clutch_request_release(int channel);

/* 请求断开全部离合（非阻塞） */
clutch_req_t *clutch_request_release_all(void);

/* 非阻塞轮询结果：true=本次请求完成，false=未完成/失败 */
bool clutch_result_poll(const clutch_req_t *req);

/* 启动后台离合任务（在 AMCU_init 里调一次即可，内部带幂等保护） */
void clutch_task_start(void);

#endif /* CLUTCH_H */
