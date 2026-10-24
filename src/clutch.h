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

/* 错误码 */
#define ESP_ERR_CLUTCH_CHANNEL  0x7001
#define ESP_ERR_CLUTCH_CONFLICT 0x7002
#define ESP_ERR_TIMEOUT         0x7003
#define ESP_ERR_INVALID_STATE   0x7004

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

#endif /* CLUTCH_H */
