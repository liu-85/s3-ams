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

_filament_motion_state_set get_filament_motion(int num);
String get_now_filament_name(int num);

#ifdef BambuBus_use_forwarding_Serial
#define forwarding_Serial Serial
extern void BambuBus_run_forward();
#endif

#ifdef __cplusplus
}
#endif
