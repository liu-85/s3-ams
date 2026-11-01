#pragma once

#include "board_pins.h"
#include <Arduino.h>
#ifdef __cplusplus
extern "C" {
#endif

/* RS-485 引脚（从 board_pins.h 引入） */
#define BambuBus_pin_tx BOARD_PIN_485_TX
#define BambuBus_pin_rx BOARD_PIN_485_RX
#define BambuBus_pin_de BOARD_PIN_485_DE

/* 使用 ESP32 的 HardwareSerial（UART1） */
#define BambuBus_uart_num UART_NUM_1

enum _filament_status
{
    offline,
    online,
    NFC_waiting
};

enum _filament_motion_state_set
{
    need_pull_back,
    need_send_out,
    waiting,
    act_send_mm,
    act_pull_mm,
    select_pos,
    release_all,
    cancel
};

enum package_type
{
    BambuBus_package_filament_motion_short,
    BambuBus_package_filament_motion_long,
    BambuBus_package_online_detect,
    BambuBus_package_REQx6,
    BambuBus_package_NFC_detect,
    BambuBus_package_set_filament,
    BambuBus_long_package_MC_online,
    BambuBus_longe_package_filament,
    BambuBus_long_package_version,
    BambuBus_package_heartbeat,
    BambuBus_package_ETC,
    BambuBus_package_ERROR,
    BambuBus_package_set_filament_type2,  // 新增：0x05 0x218 长包设置耗材
    /* ★ v1.1.35：打印机一直在发、我们一直没回的三种长包 */
    BambuBus_long_package_ext_411,        // 0x411 payload 4B，[0]=台号
    BambuBus_long_package_ext_23C,        // 0x23C payload 4B
    BambuBus_long_package_ext_237,        // 0x237 payload 3B
    __BambuBus_package_packge_type_size
};

extern void BambuBus_init();
extern int BambuBus_run();

extern bool Bambubus_read();
extern void Bambubus_set_need_to_save();
extern int get_now_filament_num();
extern int get_current_AMS_index();
extern void reset_filament_meters(int num);
extern void add_filament_meters(int num, float meters);
extern float get_filament_meters(int num);
extern void set_filament_online(int num, int count,bool if_online);
extern bool is_filament_online(int num);
extern uint8_t get_filament_color_A(int num);
extern uint8_t get_filament_color_R(int num);
extern uint8_t get_filament_color_G(int num);
extern uint8_t get_filament_color_B(int num);
extern int get_cmd_type();
extern unsigned char get_now_op_num();
extern void set_filament_motion(int num,_filament_motion_state_set motion);
extern void send_uart(const unsigned char *data, uint16_t length);

/* 新增：判断指定料位是否正在打印（内部状态 on_use / before_on_use / stop_on_use）。
 * 用于 AMCU 在打印阶段启用缓冲器微动辅助送料。 */
extern bool is_channel_printing(int num);

/* v1.1.23：比 is_channel_printing() 宽一级，多认 send_out（送料中）。
 * 缓冲微动（check_buffer_status）用它做准入 —— 流量校准阶段打印机会
 * 长时间停在 send_out，用 is_channel_printing() 会把微动整段挡掉。 */
extern bool is_channel_in_use(int num);

/* v1.1.23：该通道最近 window_ms 内是否被打印机当成"在用"（看 statu 位掩码，
 * 不看 motion）。流量校准阶段状态机会停在 idle，只能靠它判断。 */
extern bool is_channel_active_recently(int num, uint32_t window_ms);

/* ★ v1.1.27：该通道是否处于退料阶段（before_pull_back / pull_back）。
 * 缓冲补料必须整段让出 —— 退料 3 秒超时后 is_need_pull_back 已清，
 * 但打印机可能还在退料流程里，此时送料会把刚退出来的料又塞回挤出机。 */
extern bool is_channel_pulling_back(int num);

/* v1.1.23：诊断用，最近一次收到的 statu（0 = 还没收到过） */
extern uint8_t get_channel_last_statu(int num);

/* ★ v1.1.25：485 链路诊断计数（任一指针可传 NULL）
 * 用来把「打印机报 40A3」定性成：物理层断 / 干扰误码 / 没应答 / 应答内容错 */
extern void bambubus_485_stats(uint32_t *rx_bytes, uint32_t *rx_frames, uint32_t *crc_err,
                               uint32_t *overflow, uint32_t *tx_402, uint32_t *tx_103,
                               uint32_t *tx_06, uint32_t *last_rx_ms);
extern void bambubus_485_stats_reset(void);

/* ★ v1.1.31：RX 帧队列丢帧数（队列满 / 帧过长）
 * 正常应该恒为 0。持续增长说明打印机的突发比我们排空得快。 */
extern uint32_t bambubus_rx_dropped(void);

/* ★ v1.1.44：应答延迟统计（任一指针可传 NULL）
 *
 * 这是判断 40A3 最关键的度量：`qdrop` 只统计 Cxx/Dxx，长包查询
 * （0x402/0x103/0x06/0x21A/0x211）走的是"越界 clamp 到 0 继续应答"，
 * 从不计入 qdrop —— 所以 qdrop 全 0 并不等于"每问都答了"。
 *
 * 判读：
 *   max_ms < 10ms          → 应答及时，40A3 不是迟到引起的
 *   max_ms 几十~几百 ms     → 主循环卡顿（同步写 flash / ADC2 / WiFi 抢占）
 *   over100 持续增长        → 直接就是它，必须去掉同步写 flash */
extern void bambubus_485_lat_stats(uint32_t *max_ms, uint32_t *over5,
                                   uint32_t *over20, uint32_t *over100);

/* ★ v1.1.45：帧解析异常 + UART 积压峰值（任一指针可传 NULL）
 *
 *   bad_len  长度字段非法 → 复位解析器的次数
 *            **只要涨，就说明总线上真的丢过字节**（丢字节把 length 读坏了）。
 *            这是"偶发 40A3"唯一可靠的直接证据。
 *   peak     单次 poll 时 UART 环形缓冲里的最大积压字节数。
 *            接近缓冲大小（8192）= 主循环被堵住，字节正在被丢。 */
extern void bambubus_485_frame_stats(uint32_t *bad_len, uint32_t *peak);

/* ★ v1.1.33：打印机「按台号 × 包类型」的查询次数 + 未应答次数。
 *
 * 目的：一次说清"打印机到底会不会来问台 2/3（AMS-C/D）"。
 *   out_unit[台号][类型]，类型下标：
 *     0=0x402 序列号  1=0x103 版本   2=0x06 握手   3=0x21A 耗材信息
 *     4=0x211 MC在线  5=Cxx 送料查询 6=Dxx 送料命令 7=0x05 注册
 *     8=0x05 注册(0x80|台号)  9=0x411  10=0x23C  11=0x237
 *   out_drop[类型] = 问了但没应答的次数（**只要涨就一定会报 40A3**）
 *
 * 判读：
 *   · out_unit[2]/[3] 全 0  → 打印机只认 A/B，40A3 另有原因
 *   · out_unit[2]/[3] 有值  → 打印机侧还残留 4 台配置，需要在那里清掉 */
#define BAMBUBUS_QK_NUM 12
extern void bambubus_query_stats(uint16_t out_unit[4][BAMBUBUS_QK_NUM],
                                 uint16_t out_drop[BAMBUBUS_QK_NUM]);
extern void bambubus_query_stats_reset(void);

/* ★ v1.1.35：兼容开关（NVS 持久化，HTTP /compat_set 可改）
 *   bit0 = 应答 0x05 里 `0x80|台号` 那一组注册查询
 *   bit1 = 应答 0x411 长包
 *   bit2 = 应答 0x23C 长包
 *   bit3 = 应答 0x237 长包
 *
 *   ★ v1.1.39 新增三位（默认全关 = 完全退回 v1.1.29 的 485 收发行为）：
 *   bit4 = 每轮把 RX 队列排空（v1.1.31 起的行为）；关 = 每轮只处理一帧
 *   bit5 = UART RX 环形缓冲 2048（v1.1.31 起）；关 = Arduino 默认 256。改完要重启
 *   bit6 = 耗材信息由 core0 后台任务写 flash（v1.1.31 起）；关 = 就地同步写
 *
 * 现场已确认 v1.1.29（存档原厂固件）能打印、v1.1.38 不能，而两版
 * **应答内容逐字节一致**，所以差异只可能在这三处"什么时候发"上。
 *
 * ⚠️ v1.1.36：**默认值改为 0（全关）**。v1.1.35 全开时打印机报出
 * 「AMS-HT A/B/C/D 加热器短路/过热」「AMS A/B/C/D 加热器故障」一整屏 ——
 * 因为 `0x80|n` 是 **AMS-HT**（带烘干加热器）的地址，`0x411/0x23C/0x237`
 * 是**加热器状态查询**，本板没有加热器，回全 0 就等于报故障。
 * 对这四类保持静默才是正确语义（和 v1.1.20 能正常打印时一致）。 */
extern uint8_t bambubus_compat_get(void);
extern void bambubus_compat_set(uint8_t v);

/* ★ v1.1.34：按当前 AMS_COUNT 重写 16 个料位的 online/offline。
 * WEB 改「AMS 台数」后调用，立即生效（不需要重启）。 */
extern void bambubus_apply_ams_count(void);

/* ★ v1.1.31：耗材信息落盘统计（NVS 写已移到后台任务）
 *   saves    —— 真正写 flash 的次数
 *   skipped  —— 内容未变而跳过的次数
 *   max_ms   —— 单次写的最坏耗时（>100ms 说明 NVS 页在擦除）
 *   pending  —— 是否还有待写数据 */
extern void bambubus_nvs_stats(uint32_t *saves, uint32_t *skipped,
                               uint32_t *max_ms, uint32_t *total_ms, bool *pending);
extern void bambubus_nvs_task_start(void);

/* 打印阶段细分查询（v1.1.12）。
 *
 * 为什么需要：map_internal_to_api() 把 before_on_use / on_use / stop_on_use
 * 统统映射成 waiting，AMCU 用 get_filament_motion() 根本分不出当前是哪一步。
 * 但三步的电机策略完全不同：
 *   before_on_use —— 打印机已咬住耗材正在往里拉，AMS 必须"跟随送料"
 *   on_use        —— 正常打印，靠缓冲微动按需补料
 *   stop_on_use   —— 暂停，必须停料 + 断离合
 */
extern bool is_channel_before_on_use(int num);
extern bool is_channel_on_use(int num);
extern bool is_channel_stop_on_use(int num);

/* 上报给打印机的 AMS 固件版本号（v1.1.13）。
 *
 * 打印机会拿它跟白名单比对，编造的版本号（Tianshu 的 90.00.00.00）
 * 会被判成"不认识的 AMS"→ 弹窗「当前固件仅支持 AMS / AMS 2 Pro / AMS HT」。
 * 默认 0x01000657 = 01.00.06.87，网页可改，存 NVS "s3ams-bus"。
 */
extern uint32_t Bambubus_get_ams_fw_version(void);
extern void Bambubus_set_ams_fw_version(uint32_t v);

_filament_motion_state_set get_filament_motion(int num);
String get_now_filament_name(int num);

#ifdef BambuBus_use_forwarding_Serial
#define forwarding_Serial Serial
extern void BambuBus_run_forward();
#endif

#ifdef __cplusplus
}
#endif
