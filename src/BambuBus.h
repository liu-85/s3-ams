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

/* v1.1.23：诊断用，最近一次收到的 statu（0 = 还没收到过） */
extern uint8_t get_channel_last_statu(int num);

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
