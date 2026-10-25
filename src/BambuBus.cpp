// BambuBus.cpp - ESP32-S3 移植版本（保留 Tianshu 协议逻辑）
// 修改内容：
//   - 移除 RP2040 flash / uart / irq 依赖
//   - 使用 ESP32 HardwareSerial (UART1) 通信
//   - 使用 Preferences 存储设置
//   - 移除 SSD1306 OLED 依赖
//   - 移除 PCF8575 依赖

#include "BambuBus.h"
#include "CRC16.h"
#include "CRC8.h"
#include "time64.h"
#include "bus485_raw.h"
#include <cstring>

// ESP32 相关
#include <HardwareSerial.h>
#include <Preferences.h>
#include <driver/uart.h>

// ==================== 原有变量完全保留 ====================
CRC16 crc_16;
CRC8 crc_8;
uint8_t BambuBus_data_buf[1000];
int BambuBus_have_data = 0;

unsigned char now_op_num = 255;
int cmd = 0;

#define BambuBus_address_AMS08 ((uint16_t)0x700)
#define BambuBus_address_AMSLite ((uint16_t)0x1300)

#define SERSIAL_NUMBER long_packge_version_serial_number_AMS_lite
#define VERSION_NAME long_packge_version_version_and_name_AMS_lite

// ==================== 新协议内部状态机定义 ====================
enum class _filament_motion_internal : uint8_t
{
    idle = 0,
    send_out = 1,
    before_on_use = 2,
    on_use = 3,
    stop_on_use = 4,
    before_pull_back = 5,
    pull_back = 6
};

struct _filament_ext
{
    _filament_motion_internal motion = _filament_motion_internal::idle;
    uint32_t meters_virtual_count = 0;
    uint32_t last_time_ticks = 0;
    bool is_sendout_onuse_timing = false;
    uint32_t sendout_onuse_start_ticks = 0;
};

static _filament_ext filament_ext[4][4];
static uint8_t bambubus_ams_map[4] = {0, 1, 2, 3};
static uint16_t bambubus_ams_address = 0x0000;
static uint32_t time_sendout_onuse_ticks[4] = {0};
static bool have_registered[4] = {false, false, false, false};

/* AMS_COUNT 来自 settings_storage.cpp */
extern int AMS_COUNT;

// ==================== 原有_filament结构体完全保留 ====================
struct _filament
{
    char ID[8] = "GFG00";
    uint8_t color_R = 0xFF;
    uint8_t color_G = 0xFF;
    uint8_t color_B = 0xFF;
    uint8_t color_A = 0xFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";
    _filament_status statu = online;
    float meters = 0;
    _filament_motion_state_set motion_set = waiting;
};

// ==================== Flash操作API（ESP32 Preferences 替代 RP2040 Flash）====================
static Preferences s_prefs;
#define FLASH_KEY "bambubus"

struct flash_save_struct
{
    _filament filament[4][4];
    int BambuBus_now_filament_num = 0;
    uint32_t check = 0x40614061;
} data_save;

bool Bambubus_read()
{
    s_prefs.begin("s3ams", true);  // 只读模式
    size_t len = s_prefs.getBytesLength(FLASH_KEY);
    bool ok = false;
    if (len == sizeof(data_save)) {
        s_prefs.getBytes(FLASH_KEY, &data_save, sizeof(data_save));
        ok = (data_save.check == 0x40614061);
    }
    s_prefs.end();
    return ok;
}

bool Bambubus_need_to_save = false;
void Bambubus_set_need_to_save()
{
    Bambubus_need_to_save = true;
}

void Bambubus_save()
{
    s_prefs.begin("s3ams", false);  // 读写模式
    s_prefs.putBytes(FLASH_KEY, &data_save, sizeof(data_save));
    s_prefs.end();
}

// ==================== 数据访问API ====================
int get_now_filament_num()
{
    return data_save.BambuBus_now_filament_num;
}
int get_current_AMS_index()
{
    return data_save.BambuBus_now_filament_num / 4;
}

void reset_filament_meters(int num)
{
    data_save.filament[num / 4][num % 4].meters = 0;
    filament_ext[num / 4][num % 4].meters_virtual_count = 0;
}

void add_filament_meters(int num, float meters)
{
    data_save.filament[num / 4][num % 4].meters += meters;
}

float get_filament_meters(int num)
{
    return data_save.filament[num / 4][num % 4].meters;
}

bool is_filament_online(int num)
{
    return data_save.filament[num / 4][num % 4].statu == online;
}

void set_filament_online(int num,int count, bool if_online)
{
    if (if_online)
        data_save.filament[num][count].statu = online;
    else
        data_save.filament[num][count].statu = offline;
}

String get_now_filament_name(int num)
{
    return String(data_save.filament[num / 4][num % 4].name);
}

static _filament_motion_state_set map_internal_to_api(_filament_motion_internal m)
{
    switch (m)
    {
    case _filament_motion_internal::send_out:
        return need_send_out;
    case _filament_motion_internal::pull_back:
    case _filament_motion_internal::before_pull_back:
        return need_pull_back;
    default:
        return waiting;
    }
}

static void set_motion_internal(uint8_t ams_num, uint8_t ch, _filament_motion_internal new_motion)
{
    if (ams_num >= 4 || ch >= 4) return;
    filament_ext[ams_num][ch].motion = new_motion;
    data_save.filament[ams_num][ch].motion_set = map_internal_to_api(new_motion);
}

_filament_motion_state_set get_filament_motion(int num)
{
    return data_save.filament[num / 4][num % 4].motion_set;
}

void set_filament_motion(int num, _filament_motion_state_set motion)
{
    data_save.filament[num / 4][num % 4].motion_set = motion;
    uint8_t ams = num / 4;
    uint8_t ch = num % 4;
    switch (motion)
    {
    case need_send_out:
        filament_ext[ams][ch].motion = _filament_motion_internal::send_out;
        break;
    case need_pull_back:
        filament_ext[ams][ch].motion = _filament_motion_internal::pull_back;
        break;
    default:
        filament_ext[ams][ch].motion = _filament_motion_internal::idle;
        break;
    }
}

uint8_t get_filament_color_A(int num)
{
    return data_save.filament[num / 4][num % 4].color_A;
}
uint8_t get_filament_color_R(int num)
{
    return data_save.filament[num / 4][num % 4].color_R;
}
uint8_t get_filament_color_G(int num)
{
    return data_save.filament[num / 4][num % 4].color_G;
}
uint8_t get_filament_color_B(int num)
{
    return data_save.filament[num / 4][num % 4].color_B;
}

// ==================== 新协议状态机核心 ====================
static uint8_t get_loaded_filament(uint8_t ams_num)
{
    int now = data_save.BambuBus_now_filament_num;
    if (now >= ams_num * 4 && now < (ams_num + 1) * 4)
        return now % 4;
    return 0xFF;
}

static bool process_motion_state_machine(
    uint8_t ams_num,
    uint8_t filament_channel,
    uint8_t statu_flags,
    uint8_t motion_flag,
    uint32_t time_used_ms)
{
    if (ams_num >= AMS_COUNT) return false;

    _filament_ext *ext = &filament_ext[ams_num][filament_channel];
    _filament *fil = &data_save.filament[ams_num][filament_channel];

    ext->last_time_ticks = (uint32_t)get_time64();

    uint8_t loaded = get_loaded_filament(ams_num);
    bool allow_any = (loaded == 0xFF) || (loaded == filament_channel);
    bool allow_stop = (loaded == filament_channel);

    bool state_changed = false;

    {
        bool is_send_out = (statu_flags == 0x03) && (motion_flag == 0x00);
        bool is_before_on_use = (statu_flags == 0x09) && (motion_flag == 0xA5);
        bool is_stop_on_use = (statu_flags == 0x07) && (motion_flag == 0x00);
        bool is_on_use = (statu_flags == 0x07) && (motion_flag == 0x7F);
        bool is_before_pullb = (statu_flags == 0x09) && (motion_flag == 0x3F);

        const bool accept =
            (is_send_out) ||
            (is_before_on_use && allow_any) ||
            (is_on_use && allow_any) ||
            (is_stop_on_use && allow_stop) ||
            (is_before_pullb && allow_stop);

        if (accept)
        {
            if (data_save.BambuBus_now_filament_num != ams_num * 4 + filament_channel)
            {
                if (data_save.BambuBus_now_filament_num < 16)
                {
                    uint8_t prev_ams = data_save.BambuBus_now_filament_num / 4;
                    uint8_t prev_ch = data_save.BambuBus_now_filament_num % 4;
                    set_motion_internal(prev_ams, prev_ch, _filament_motion_internal::idle);
                    time_sendout_onuse_ticks[prev_ch] = 0;
                }
                data_save.BambuBus_now_filament_num = ams_num * 4 + filament_channel;
            }
        }

        if (is_send_out)
        {
            time_sendout_onuse_ticks[filament_channel] = 0;
            if (ext->motion != _filament_motion_internal::send_out && loaded != 0xFF)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (i != filament_channel)
                    {
                        set_motion_internal(ams_num, i, _filament_motion_internal::idle);
                    }
                }
            }
            set_motion_internal(ams_num, filament_channel, _filament_motion_internal::send_out);
            state_changed = true;
        }
        else if (is_before_on_use && allow_any)
        {
            time_sendout_onuse_ticks[filament_channel] = 0;
            set_motion_internal(ams_num, filament_channel, _filament_motion_internal::before_on_use);
            state_changed = true;
        }
        else if (is_on_use && allow_any)
        {
            if (ext->motion == _filament_motion_internal::send_out)
            {
                uint32_t now = (uint32_t)get_time64();
                if (time_sendout_onuse_ticks[filament_channel] == 0)
                    time_sendout_onuse_ticks[filament_channel] = now;

                uint32_t dt = now - time_sendout_onuse_ticks[filament_channel];
                if (dt < 5000)
                    return true;
                time_sendout_onuse_ticks[filament_channel] = 0;
            }
            set_motion_internal(ams_num, filament_channel, _filament_motion_internal::on_use);
            if (ext->meters_virtual_count < 10000)
            {
                fil->meters += (float)time_used_ms / 300000.0f;
                ext->meters_virtual_count += time_used_ms;
            }
            state_changed = true;
        }
        else if (is_stop_on_use && allow_stop)
        {
            time_sendout_onuse_ticks[filament_channel] = 0;
            if (ext->motion == _filament_motion_internal::on_use ||
                ext->motion == _filament_motion_internal::before_on_use)
            {
                set_motion_internal(ams_num, filament_channel, _filament_motion_internal::stop_on_use);
                state_changed = true;
            }
        }
        else if (is_before_pullb && allow_stop)
        {
            time_sendout_onuse_ticks[filament_channel] = 0;
            if (ext->motion == _filament_motion_internal::on_use ||
                ext->motion == _filament_motion_internal::before_on_use ||
                ext->motion == _filament_motion_internal::stop_on_use)
            {
                set_motion_internal(ams_num, filament_channel, _filament_motion_internal::before_pull_back);
                state_changed = true;
            }
        }
        else if (statu_flags == 0x09)
        {
            time_sendout_onuse_ticks[filament_channel] = 0;
        }
    }

    return state_changed;
}

// ==================== UART底层（ESP32 HardwareSerial）====================
HardwareSerial RS485(1);  // UART1

bool TX_IDLE = true;
bool need_debug = false;

void send_uart(const unsigned char *data, uint16_t length)
{
    TX_IDLE = false;
    digitalWrite(BambuBus_pin_de, 1);     // DE 高 = 发送
    RS485.write(data, length);
    RS485.flush();                        // 等待发送完成
    digitalWrite(BambuBus_pin_de, 0);     // DE 低 = 接收
    TX_IDLE = true;
    bus485raw_record(1, (const uint8_t *)data, length);  /* 原始报文记录：本机发出 */
}

uint8_t buf_X[1000];
CRC8 _RX_IRQ_crcx(0x39, 0x66, 0x00, false, false);

/* 单字节协议解析（原 RX_IRQ 逻辑，改为函数调用） */
static void RX_IRQ(unsigned char data)
{
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;

    if (_index == 0)
    {
        if (data == 0x3D)
        {
            BambuBus_data_buf[0] = 0x3D;
            _RX_IRQ_crcx.restart();
            _RX_IRQ_crcx.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    }
    else
    {
        BambuBus_data_buf[_index] = data;
        if (_index == 1)
        {
            if (data & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index)
        {
            length = data;
        }
        if (_index < data_CRC8_index)
        {
            _RX_IRQ_crcx.add(data);
        }
        else if (_index == data_CRC8_index)
        {
            if (data != _RX_IRQ_crcx.calc())
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length)
        {
            _index = 0;
            memcpy(buf_X, BambuBus_data_buf, length);
            BambuBus_have_data = length;
            bus485raw_record(0, BambuBus_data_buf, length);  /* 原始报文记录：打印机发来 */
        }
        if (_index >= 999)
        {
            _index = 0;
        }
    }
}

/* 从 UART 缓冲区读取并喂给协议解析器 */
static void BambuBus_UART_Poll(void)
{
    while (RS485.available() > 0)
    {
        unsigned char x = RS485.read();
        RX_IRQ(x);
    }
}

void BambuBUS_UART_Init(uint32_t baudrate)
{
    RS485.begin(baudrate, SERIAL_8E1, BambuBus_pin_rx, BambuBus_pin_tx);
    pinMode(BambuBus_pin_de, OUTPUT);
    digitalWrite(BambuBus_pin_de, 0);  // 默认接收
}

// ==================== 初始化 ====================
void BambuBus_init()
{
    bool _init_ready = Bambubus_read();
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            filament_ext[i][j] = _filament_ext();
        }
        time_sendout_onuse_ticks[i] = 0;
        have_registered[i] = false;
    }
    bambubus_ams_address = 0x0000;

    if (!_init_ready)
    {
        data_save.filament[0][0].color_R = 0xFF;
        data_save.filament[0][0].color_G = 0x00;
        data_save.filament[0][0].color_B = 0x00;
        data_save.filament[0][1].color_R = 0x00;
        data_save.filament[0][1].color_G = 0xFF;
        data_save.filament[0][1].color_B = 0x00;
        data_save.filament[0][2].color_R = 0x00;
        data_save.filament[0][2].color_G = 0x00;
        data_save.filament[0][2].color_B = 0xFF;
        data_save.filament[0][3].color_R = 0x88;
        data_save.filament[0][3].color_G = 0x88;
        data_save.filament[0][3].color_B = 0x88;

        data_save.filament[1][0].color_R = 0xC0;
        data_save.filament[1][0].color_G = 0x20;
        data_save.filament[1][0].color_B = 0x20;
        data_save.filament[1][1].color_R = 0x20;
        data_save.filament[1][1].color_G = 0xC0;
        data_save.filament[1][1].color_B = 0x20;
        data_save.filament[1][2].color_R = 0x20;
        data_save.filament[1][2].color_G = 0x20;
        data_save.filament[1][2].color_B = 0xC0;
        data_save.filament[1][3].color_R = 0x60;
        data_save.filament[1][3].color_G = 0x60;
        data_save.filament[1][3].color_B = 0x60;

        data_save.filament[2][0].color_R = 0x80;
        data_save.filament[2][0].color_G = 0x40;
        data_save.filament[2][0].color_B = 0x40;
        data_save.filament[2][1].color_R = 0x40;
        data_save.filament[2][1].color_G = 0x80;
        data_save.filament[2][1].color_B = 0x40;
        data_save.filament[2][2].color_R = 0x40;
        data_save.filament[2][2].color_G = 0x40;
        data_save.filament[2][2].color_B = 0x80;
        data_save.filament[2][3].color_R = 0x40;
        data_save.filament[2][3].color_G = 0x40;
        data_save.filament[2][3].color_B = 0x40;

        data_save.filament[3][0].color_R = 0x40;
        data_save.filament[3][0].color_G = 0x20;
        data_save.filament[3][0].color_B = 0x20;
        data_save.filament[3][1].color_R = 0x20;
        data_save.filament[3][1].color_G = 0x40;
        data_save.filament[3][1].color_B = 0x20;
        data_save.filament[3][2].color_R = 0x20;
        data_save.filament[3][2].color_G = 0x20;
        data_save.filament[3][2].color_B = 0x40;
        data_save.filament[3][3].color_R = 0x20;
        data_save.filament[3][3].color_G = 0x20;
        data_save.filament[3][3].color_B = 0x20;
    }

    for (auto &i : data_save.filament)
    {
        for (auto &j : i)
        {
#ifdef _Bambubus_DEBUG_mode_
            j.statu = online;
#else
            j.statu = offline;
#endif
            j.motion_set = waiting;
        }
    }

    BambuBUS_UART_Init(1228800);
}

// ==================== 包处理辅助函数 ====================
bool package_check_crc16(uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

void package_send_with_crc(uint8_t *data, int data_length)
{
    crc_8.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++)
        {
            crc_8.add(data[i]);
        }
        data[3] = crc_8.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++)
        {
            crc_8.add(data[i]);
        }
        data[6] = crc_8.calc();
    }
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    send_uart(data, data_length);
}

uint8_t packge_send_buf[1000];

struct long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas;
    uint16_t data_length;
} __attribute__((packed));

void Bambubus_long_package_send(long_packge_data *data)
{
    packge_send_buf[0] = 0x3D;
    packge_send_buf[1] = 0x00;
    data->package_length = data->data_length + 15;
    memcpy(packge_send_buf + 2, data, 11);
    memcpy(packge_send_buf + 13, data->datas, data->data_length);
    package_send_with_crc(packge_send_buf, data->data_length + 15);
}

void Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15;
}

long_packge_data printer_data_long;

// ==================== 动态压力值计算 ====================
static uint16_t get_pressure_value(uint8_t ams_num, uint8_t ch)
{
    if (ams_num >= AMS_COUNT || ch >= 4) return 0xFFFF;

    _filament_motion_internal m = filament_ext[ams_num][ch].motion;
    _filament_motion_internal prev_motion = m;

    switch (m)
    {
    case _filament_motion_internal::send_out:
        return 0x4700;
    case _filament_motion_internal::before_on_use:
        return (prev_motion == _filament_motion_internal::send_out) ? 0x4700 : 0x2B00;
    case _filament_motion_internal::on_use:
        return 0x2B00;
    case _filament_motion_internal::stop_on_use:
        return (prev_motion == _filament_motion_internal::send_out) ? 0x4700 : 0x2B00;
    case _filament_motion_internal::before_pull_back:
        return 0x2B00;
    case _filament_motion_internal::pull_back:
        return 0x4700;
    default:
        return 0xFFFF;
    }
}

static uint8_t get_filament_use_flag(uint8_t ams_num)
{
    if (ams_num >= AMS_COUNT) return 0x00;

    for (int i = 0; i < 4; i++)
    {
        _filament_motion_internal m = filament_ext[ams_num][i].motion;
        if (m != _filament_motion_internal::idle)
        {
            if (m == _filament_motion_internal::send_out ||
                m == _filament_motion_internal::pull_back)
                return 0x02;
            else
                return 0x04;
        }
    }
    return 0x00;
}

// ==================== 包类型解析 ====================
package_type get_packge_type(unsigned char *buf, int length)
{
    if (package_check_crc16(buf, length) == false)
    {
        return BambuBus_package_ERROR;
    }
    if (buf[1] == 0xC5)
    {
        switch (buf[4])
        {
        case 0x03:
            return BambuBus_package_filament_motion_short;
        case 0x04:
            return BambuBus_package_filament_motion_long;
        case 0x05:
            return BambuBus_package_online_detect;
        case 0x06:
            return BambuBus_package_REQx6;
        case 0x07:
            return BambuBus_package_NFC_detect;
        case 0x08:
            return BambuBus_package_set_filament;
        case 0x20:
            return BambuBus_package_heartbeat;
        default:
            return BambuBus_package_ETC;
        }
    }
    else if (buf[1] == 0x05)
    {
        Bambubus_long_package_analysis(buf, length, &printer_data_long);
        bambubus_ams_address = BambuBus_address_AMS08;

        switch (printer_data_long.type)
        {
        case 0x21A:
            return BambuBus_long_package_MC_online;
        case 0x211:
            return BambuBus_longe_package_filament;
        case 0x218:
            return BambuBus_package_set_filament_type2;
        case 0x103:
            return BambuBus_long_package_version;
        case 0x402:
            return BambuBus_long_package_version;
        default:
            return BambuBus_package_ETC;
        }
    }
    return BambuBus_package_ERROR;
}

uint8_t package_num = 0;

uint8_t get_filament_left_char(uint8_t AMS_num)
{
    uint8_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        if (data_save.filament[AMS_num][i].statu == online)
        {
            data |= (1 << i) << i;
            if (filament_ext[AMS_num][i].motion != _filament_motion_internal::idle &&
                bambubus_ams_address == BambuBus_address_AMS08)
            {
                data |= (2 << i) << i;
            }
        }
    }
    return data;
}

// ==================== 响应函数 ====================
#define C_test 0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x80, 0xBF, \
               0x00, 0x00, 0x00, 0xC0, \
               0x00, 0xC0, 0x5D, 0xFF, \
               0xFC, 0xFF, 0xFC, 0xFF, \
               0x00, 0x00, 0x44, 0x00, \
               0x55,                   \
               0xC1, 0xC3, 0xEC, 0xBC, \
               0x01, 0x01, 0x01, 0x01,

unsigned char Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0x90, 0xE4};

static inline uint16_t swap_bytes(uint16_t val) {
    return (val >> 8) | (val << 8);
}

void set_cxx_buf(unsigned char *set_buf, unsigned char *buf, int length)
{
    unsigned char AMS_num = buf[5];
    unsigned char read_num = buf[7];
    now_op_num = read_num;
    float meters = -1;

    if (AMS_num >= AMS_COUNT) return;

    meters = data_save.filament[AMS_num][read_num].meters;

    uint8_t flagx = get_filament_use_flag(AMS_num);
    if (flagx == 0x00) flagx = 0x02;

    set_buf[2] = flagx;
    set_buf[3] = read_num;

    memcpy(set_buf + 4, &meters, sizeof(meters));

    uint16_t pressure = get_pressure_value(AMS_num, read_num);
    memcpy(set_buf + 8, &pressure, sizeof(pressure));

    set_buf[24] = get_filament_left_char(AMS_num);
}

void send_for_Cxx(unsigned char *buf, int length)
{
    Cxx_res[1] = 0xC0 | (package_num << 3);
    set_cxx_buf(Cxx_res + 5, buf, length);
    package_send_with_crc(Cxx_res, sizeof(Cxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}


unsigned char Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, 0x75, 0x01, 0x11,
                           0x04, 0x04, 0x04, 0xFF,
                           0x00, 0x00, 0x00, 0x00,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x90, 0xE4};

bool need_res_for_06 = false;
uint8_t res_for_06_num = 0xFF;
int last_detect = 0;
uint8_t filament_flag_detected = 0;
void send_for_Dxx(unsigned char *buf, int length)
{
    unsigned char filament_flag_on = 0x00;
    unsigned char filament_flag_NFC = 0x00;
    unsigned char AMS_num = buf[5];
    unsigned char statu_flags = buf[6];
    unsigned char fliment_motion_flag = buf[7];
    unsigned char read_num = buf[9];
    now_op_num = read_num;
    float meters = -1;
    uint8_t flagx = 0x02;

    if (AMS_num >= AMS_COUNT) return;

    for (auto i = 0; i < 4; i++)
    {
        if (data_save.filament[AMS_num][i].statu == online)
        {
            filament_flag_on |= 1 << i;
        }
        else if (data_save.filament[AMS_num][i].statu == NFC_waiting)
        {
            filament_flag_on |= 1 << i;
            filament_flag_NFC |= 1 << i;
        }
    }

    static uint32_t last_ticks[4] = {0};
    uint32_t now_ticks = (uint32_t)get_time64();
    uint32_t dt = (last_ticks[AMS_num] == 0) ? 0 : (now_ticks - last_ticks[AMS_num]);
    last_ticks[AMS_num] = now_ticks;
    uint32_t time_used = dt > 1000 ? 0 : dt;

    if (read_num != 0xFF && read_num < 4)
    {
        process_motion_state_machine(AMS_num, read_num, statu_flags, fliment_motion_flag, time_used);
        meters = data_save.filament[AMS_num][read_num].meters;
    }
    else if (read_num == 0xFF && statu_flags == 0x03 && fliment_motion_flag == 0x00)
    {
        uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
        if (filament_ext[AMS_num][now_ch].motion == _filament_motion_internal::before_pull_back)
        {
            set_motion_internal(AMS_num, now_ch, _filament_motion_internal::pull_back);
        }
    }
    else if (read_num == 0xFF && statu_flags == 0x01)
    {
        uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
        if (now_ch < 4)
        {
            set_motion_internal(AMS_num, now_ch, _filament_motion_internal::idle);
            time_sendout_onuse_ticks[now_ch] = 0;
        }
    }
    else if (read_num == 0xFF)
    {
        if (data_save.BambuBus_now_filament_num < 16)
        {
            uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
            _filament_motion_internal m = filament_ext[AMS_num][now_ch].motion;
            if (m == _filament_motion_internal::on_use ||
                m == _filament_motion_internal::before_on_use ||
                m == _filament_motion_internal::stop_on_use)
            {
                return;
            }
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            set_motion_internal(AMS_num, i, _filament_motion_internal::idle);
            time_sendout_onuse_ticks[i] = 0;
        }

        data_save.BambuBus_now_filament_num = 0xFF;
    }

    uint16_t pressure = 0xFFFF;
    uint8_t use_flag = 0x00;
    if (read_num < 4)
    {
        pressure = get_pressure_value(AMS_num, read_num);
        use_flag = get_filament_use_flag(AMS_num);
    }

    Dxx_res[1] = 0xC0 | (package_num << 3);
    Dxx_res[5] = AMS_num;
    Dxx_res[9] = filament_flag_on;
    Dxx_res[10] = filament_flag_on - filament_flag_NFC;
    Dxx_res[11] = filament_flag_on - filament_flag_NFC;
    Dxx_res[12] = 0xFF;
    Dxx_res[17] = AMS_num;
    Dxx_res[19] = use_flag;
    Dxx_res[20] = (read_num == 0xFF) ? 0xFF : read_num;
    Dxx_res[13] = filament_flag_NFC;
    Dxx_res[41] = get_filament_left_char(AMS_num);

    memcpy(Dxx_res + 21, &meters, sizeof(meters));
    memcpy(Dxx_res + 25, &pressure, sizeof(pressure));

    if (last_detect != 0)
    {
        if (last_detect > 10)
        {
            Dxx_res[19] = 0x01;
        }
        else
        {
            Dxx_res[12] = filament_flag_detected;
            Dxx_res[19] = 0x01;
            Dxx_res[20] = filament_flag_detected;
        }
        last_detect--;
    }

    package_send_with_crc(Dxx_res, sizeof(Dxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}

// ==================== 在线检测（新协议完整注册机制）====================
unsigned char F00_res[4][29] = {
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
     0x16,
     0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
     0x00, 0x00, 0x00, 0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x01,
     0x16,
     0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
     0x00, 0x00, 0x00, 0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x02,
     0x16,
     0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
     0x00, 0x00, 0x00, 0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x03,
     0x16,
     0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
     0x00, 0x00, 0x00, 0x33, 0xF0}
};

static inline void bus_wait_idle_5ms(void)
{
    delayMicroseconds(5000);
}

void send_for_Fxx(unsigned char *buf, int length)
{
    if (buf[5] == 0x00)
    {
        for (int i = 0; i < AMS_COUNT; i++)
        {
            if (have_registered[i]) continue;
            if (data_save.filament[i][0].statu != online) continue;

            F00_res[i][0] = 0x3D;
            F00_res[i][1] = 0xC0;
            F00_res[i][2] = 29;
            F00_res[i][3] = 0xB4;
            F00_res[i][4] = 0x05;
            F00_res[i][5] = 0x00;
            F00_res[i][6] = i;

            package_send_with_crc(F00_res[i], 29);
            bus_wait_idle_5ms();
        }
    }
    else if (buf[5] == 0x01 && buf[6] < AMS_COUNT)
    {
        uint8_t ams_num = buf[6];
        if (data_save.filament[ams_num][0].statu != online)
        {
            have_registered[ams_num] = false;
            return;
        }

        F00_res[ams_num][0] = 0x3D;
        F00_res[ams_num][1] = 0xC0;
        F00_res[ams_num][2] = 29;
        F00_res[ams_num][3] = 0xB4;
        F00_res[ams_num][4] = 0x05;
        F00_res[ams_num][5] = 0x01;
        F00_res[ams_num][6] = ams_num;

        package_send_with_crc(F00_res[ams_num], 29);

        if (memcmp(F00_res[ams_num] + 7, buf + 7, 20) == 0)
            have_registered[ams_num] = true;
        else
            have_registered[ams_num] = false;
    }
    else if (buf[4] == 0x06)
    {
        uint8_t ams_num = buf[5];
        if (res_for_06_num == 0xFF)
        {
            res_for_06_num = ams_num;
        }
        else
        {
            ams_num = res_for_06_num;
        }

        unsigned char F06_res[] = {
            0x3D, 0xC0, 0x0B, 0xA0, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        F06_res[1] = 0xC0 | (package_num << 3);
        F06_res[5] = ams_num;

        package_send_with_crc(F06_res, sizeof(F06_res));

        if (package_num < 7)
            package_num++;
        else
            package_num = 0;
    }
}

// ==================== MC在线 ====================
unsigned char long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_MC_online(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    if (AMS_num >= AMS_COUNT || data_save.filament[AMS_num][0].statu != online)
        return;

    data.datas = long_packge_MC_online;
    data.datas[0] = AMS_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}

// ==================== 读取耗材信息 ====================
unsigned char long_packge_filament[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00,
    0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xDD, 0xB1, 0xD4, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x18, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_filament(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];

    if (AMS_num >= AMS_COUNT || filament_num >= 4 ||
        data_save.filament[AMS_num][filament_num].statu != online)
        return;

    long_packge_filament[0] = AMS_num;
    long_packge_filament[1] = filament_num;
    memcpy(long_packge_filament + 19, data_save.filament[AMS_num][filament_num].ID, 8);
    memcpy(long_packge_filament + 27, data_save.filament[AMS_num][filament_num].name, 20);
    long_packge_filament[59] = data_save.filament[AMS_num][filament_num].color_R;
    long_packge_filament[60] = data_save.filament[AMS_num][filament_num].color_G;
    long_packge_filament[61] = data_save.filament[AMS_num][filament_num].color_B;
    long_packge_filament[62] = data_save.filament[AMS_num][filament_num].color_A;
    memcpy(long_packge_filament + 79, &data_save.filament[AMS_num][filament_num].temperature_max, 2);
    memcpy(long_packge_filament + 81, &data_save.filament[AMS_num][filament_num].temperature_min, 2);

    data.datas = long_packge_filament;
    data.data_length = sizeof(long_packge_filament);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}

// ==================== 版本和序列号数组定义 ====================

unsigned char long_packge_version_serial_number_AMS_lite[] = {
    0x0F,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0E, 0x7D,
    0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00
};

unsigned char long_packge_version_serial_number_AMS08[] = {
    0x0F,
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0E, 0x7D,
    0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00
};

unsigned char long_packge_version_version_and_name_AMS_lite[] = {
    0x00, 0x00, 0x00, 0x3C,
    0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

unsigned char long_packge_version_version_and_name_AMS08[] = {
    0x00, 0x00, 0x00, 0x5A,
    0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};
void send_for_long_packge_version(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    unsigned char *payload = nullptr;
    uint16_t payload_len = 0;

    if (bambubus_ams_address != BambuBus_address_AMS08)
    {
        return;
    }

    if (printer_data_long.type == 0x402)
    {
        AMS_num = printer_data_long.datas[33];
        if (AMS_num >= AMS_COUNT) AMS_num = 0;

        payload = long_packge_version_serial_number_AMS08;
        payload_len = sizeof(long_packge_version_serial_number_AMS08);

        payload[4] = 0x30 + AMS_num;
        payload[34] = 0xA0 + AMS_num;
        payload[65] = AMS_num;
    }
    else if (printer_data_long.type == 0x103)
    {
        payload = long_packge_version_version_and_name_AMS08;
        payload_len = sizeof(long_packge_version_version_and_name_AMS08);
        payload[payload_len - 1] = AMS_num;
    }

    if (payload == nullptr) return;

    data.datas = payload;
    data.data_length = payload_len;
    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}

// ==================== 设置耗材信息（短包 0xC5 0x08）====================
unsigned char Set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};

void send_for_Set_filament(unsigned char *buf, int length)
{
    uint8_t read_num = buf[5];
    uint8_t AMS_num = (read_num >> 4) & 0x0F;
    read_num = read_num & 0x0F;
    now_op_num = read_num;

    if (AMS_num >= 4 || read_num >= 4) return;

    memcpy(data_save.filament[AMS_num][read_num].ID, buf + 7, 8);
    data_save.filament[AMS_num][read_num].color_R = buf[15];
    data_save.filament[AMS_num][read_num].color_G = buf[16];
    data_save.filament[AMS_num][read_num].color_B = buf[17];
    data_save.filament[AMS_num][read_num].color_A = buf[18];
    memcpy(&data_save.filament[AMS_num][read_num].temperature_min, buf + 19, 2);
    memcpy(&data_save.filament[AMS_num][read_num].temperature_max, buf + 21, 2);
    memcpy(data_save.filament[AMS_num][read_num].name, buf + 23, 20);
    data_save.filament[AMS_num][read_num].name[19] = '\0';

    package_send_with_crc(Set_filament_res, sizeof(Set_filament_res));
    Bambubus_set_need_to_save();
}

// ==================== 设置耗材信息类型2（长包 0x05 0x218）====================
unsigned char set_filament_res_type2[] = {0x00, 0x00, 0x00};

void send_for_Set_filament_type2(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t read_num = printer_data_long.datas[1];

    if (AMS_num >= 4 || read_num >= 4) return;

    now_op_num = read_num;

    memcpy(data_save.filament[AMS_num][read_num].ID,
           printer_data_long.datas + 2, 8);

    data_save.filament[AMS_num][read_num].color_R = printer_data_long.datas[10];
    data_save.filament[AMS_num][read_num].color_G = printer_data_long.datas[11];
    data_save.filament[AMS_num][read_num].color_B = printer_data_long.datas[12];
    data_save.filament[AMS_num][read_num].color_A = printer_data_long.datas[13];

    memcpy(&data_save.filament[AMS_num][read_num].temperature_min,
           printer_data_long.datas + 14, 2);
    memcpy(&data_save.filament[AMS_num][read_num].temperature_max,
           printer_data_long.datas + 16, 2);

    memset(data_save.filament[AMS_num][read_num].name, 0, 20);
    memcpy(data_save.filament[AMS_num][read_num].name,
           printer_data_long.datas + 18, 16);
    data_save.filament[AMS_num][read_num].name[19] = '\0';

    set_filament_res_type2[0] = AMS_num;
    set_filament_res_type2[1] = read_num;
    set_filament_res_type2[2] = 0x00;

    data.datas = set_filament_res_type2;
    data.data_length = sizeof(set_filament_res_type2);
    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);

    Bambubus_set_need_to_save();
}

int get_cmd_type()
{
    return cmd;
}

/* 判断指定料位是否正在打印（内部状态） */
bool is_channel_printing(int num)
{
    if (num < 0 || num >= 16) return false;
    uint8_t ams = num / 4;
    uint8_t ch = num % 4;
    _filament_motion_internal m = filament_ext[ams][ch].motion;
    return (m == _filament_motion_internal::on_use ||
            m == _filament_motion_internal::before_on_use ||
            m == _filament_motion_internal::stop_on_use);
}

unsigned char get_now_op_num()
{
    return now_op_num;
}

// ==================== 主循环 ====================
int BambuBus_run()
{
    int stu = 0;
    static uint64_t time_set = 1000;
    uint64_t timex = get_time64();

    /* 先处理 UART 接收 */
    BambuBus_UART_Poll();

    if (timex > time_set)
    {
        stu = -1;
    }

    if (BambuBus_have_data)
    {
        int data_length = BambuBus_have_data;
        BambuBus_have_data = 0;
        stu = 1;
        time_set = timex + 1000;

        cmd = get_packge_type(buf_X, data_length);
        switch (cmd)
        {
        case BambuBus_package_heartbeat:
            break;
        case BambuBus_package_filament_motion_short:
            send_for_Cxx(buf_X, data_length);
            break;
        case BambuBus_package_filament_motion_long:
            send_for_Dxx(buf_X, data_length);
            break;
        case BambuBus_package_online_detect:
            send_for_Fxx(buf_X, data_length);
            break;
        case BambuBus_package_REQx6:
            break;
        case BambuBus_long_package_MC_online:
            send_for_long_packge_MC_online(buf_X, data_length);
            break;
        case BambuBus_longe_package_filament:
            send_for_long_packge_filament(buf_X, data_length);
            break;
        case BambuBus_long_package_version:
            send_for_long_packge_version(buf_X, data_length);
            break;
        case BambuBus_package_NFC_detect:
            break;
        case BambuBus_package_set_filament:
            send_for_Set_filament(buf_X, data_length);
            break;
        case BambuBus_package_set_filament_type2:
            send_for_Set_filament_type2(buf_X, data_length);
            break;
        default:
            break;
        }
    }

    if (Bambubus_need_to_save)
    {
        Bambubus_save();
        Bambubus_need_to_save = false;
    }

    return stu;
}
