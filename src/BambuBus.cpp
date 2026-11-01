// BambuBus.cpp - ESP32-S3 移植版本（保留 Tianshu 协议逻辑）
// 修改内容：
//   - 移除 RP2040 flash / uart / irq 依赖
//   - 使用 ESP32 HardwareSerial (UART1) 通信
//   - 使用 Preferences 存储设置
//   - 移除 SSD1306 OLED 依赖
//   - 移除 PCF8575 依赖

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
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

/* ==================== 485 兼容开关（位定义） ====================
 * 必须放在文件最前面 —— BambuBUS_UART_Init() 在 600 多行就用到 bit5，
 * 宏写在使用点之后会编译不过。
 * 各位的含义与现场用法见下方 compat_load() 前的长注释。 */
#define COMPAT_REG_HI    0x01   /* 应答 0x05 里 0x80|台号 那组注册（AMS-HT） */
#define COMPAT_411       0x02   /* 应答 0x411 烘干加热器查询 */
#define COMPAT_23C       0x04   /* 应答 0x23C */
#define COMPAT_237       0x08   /* 应答 0x237 */
#define COMPAT_RX_BATCH  0x10   /* 每轮排空 RX 队列（v1.1.31 起）；关=每轮一帧（v1.1.29） */
#define COMPAT_RX_BIGBUF 0x20   /* UART RX 缓冲 2048（v1.1.31 起）；关=默认 256（v1.1.29） */
#define COMPAT_NVS_ASYNC 0x40   /* 耗材信息后台写 flash（v1.1.31 起）；关=同步写（v1.1.29） */
#define COMPAT_ALL       0x7F
#define COMPAT_DEFAULT   0x00   /* 默认全关 = 完全退回 v1.1.29 的收发行为 */
#define COMPAT_NVS_NS    "s3ams-compat"

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

/* ==========================================================================
 * AMS 固件版本号（v1.1.13）
 * ==========================================================================
 *
 * 打印机会拿这个 4 字节版本号去跟自己的**白名单**比对，对不上就当成
 * "不认识的 AMS 设备"，直接弹窗：
 *   当前固件仅支持 AMS / AMS 2 Pro / AMS HT，最多可连接 4 台，
 *   请务必先移除所有 AMS，再重新插入上述支持的设备。[0500-40A5]
 *
 * ★ Tianshu 原版写的是 `00 00 00 5A`（90.00.00.00）—— 这是个**编造的版本号**，
 *   旧打印机固件不校验，新版一校验就废。现场表现就是"经常弹窗"。
 *
 * 实测可用的真实版本号（都从真机抓包得到，二选一）：
 *   01.00.06.87  → {0x57,0x06,0x00,0x01}  BMCU-X/BMCU-U 在用，A1 FW 01.08.01.00 验证过
 *   00.00.06.49  → {0x31,0x06,0x00,0x00}  BMCU370 实测抓包
 *   00.00.08.15  → {0x0F,0x08,0x00,0x00}  AMS Lite 线
 *
 * 默认用最新的 01.00.06.87。如果换了打印机还弹窗，去网页
 * 「和打印机的通信方式」→「AMS 版本号」换成别的，**不用重编译**。
 *
 * 小端存放：uint32 0x01000657 → 字节 57 06 00 01 → 显示 01.00.06.87
 */
#define AMS_FW_VER_DEFAULT  0x01000657u   /* 01.00.06.87 */

static uint32_t s_ams_fw_ver = AMS_FW_VER_DEFAULT;

uint32_t Bambubus_get_ams_fw_version(void)
{
    return s_ams_fw_ver;
}

void Bambubus_set_ams_fw_version(uint32_t v)
{
    /* 全 0 / 全 0xFF 明显是脏数据，拒绝 */
    if (v == 0 || v == 0xFFFFFFFFu) {
        log_w("AMS 版本号 0x%08X 非法，忽略", (unsigned)v);
        return;
    }
    s_ams_fw_ver = v;

    Preferences p;
    if (p.begin("s3ams-bus", false)) {
        p.putUInt("ver", v);
        p.end();
    }
    log_i("AMS 版本号已改为 %02u.%02u.%02u.%02u（重启后仍然有效）",
          (unsigned)((v >> 24) & 0xFF), (unsigned)((v >> 16) & 0xFF),
          (unsigned)((v >> 8) & 0xFF), (unsigned)(v & 0xFF));
}

static void ams_fw_ver_load(void)
{
    Preferences p;
    if (!p.begin("s3ams-bus", true)) return;
    uint32_t v = p.getUInt("ver", AMS_FW_VER_DEFAULT);
    p.end();
    if (v != 0 && v != 0xFFFFFFFFu) s_ams_fw_ver = v;
    log_i("AMS 上报版本号: %02u.%02u.%02u.%02u",
          (unsigned)((s_ams_fw_ver >> 24) & 0xFF), (unsigned)((s_ams_fw_ver >> 16) & 0xFF),
          (unsigned)((s_ams_fw_ver >> 8) & 0xFF), (unsigned)(s_ams_fw_ver & 0xFF));
}

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

    /* ★ v1.1.23：最近一次收到的 statu 位掩码 + 时间戳（ms）。
     *
     * 为什么要单独存：流量校准阶段打印机会持续从 AMS 抽料，但状态机可能
     * 一直停在 idle（帧没匹配上 on_use/before_on_use），于是
     * is_channel_printing() 为 false → 缓冲微动被整段挡掉、微动压下不送料。
     * 存下 statu 就能判断"打印机最近是否把这一路当成在用"，
     * 同时 /status 里可以直接看到它，排查"微动没反应"不用再靠抓包。 */
    uint8_t  last_statu    = 0;
    uint32_t last_statu_ms = 0;
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

    /* v1.1.23：记下这一路最近一次收到的 statu，供缓冲微动的准入判定和 /status 诊断用 */
    ext->last_statu    = statu_flags;
    ext->last_statu_ms = (uint32_t)get_time64();

    uint8_t loaded = get_loaded_filament(ams_num);
    bool allow_any = (loaded == 0xFF) || (loaded == filament_channel);
    bool allow_stop = (loaded == filament_channel);

    bool state_changed = false;

    {
        bool is_send_out = (statu_flags == 0x03) && (motion_flag == 0x00);
        bool is_before_on_use = (statu_flags == 0x09) && (motion_flag == 0xA5);
        /* ★★ v1.1.12：0x0F 是"使用中"（0x01|0x02|0x04|0x08），必须一起认 ★★
         *
         * 现场抓包（2026-10-25 第四次）：整份日志里 `0x07/0x7F`（我们原以为的
         * on_use）**一帧都没有**，而 `0x0F/0xFF` 出现 123 帧、`0x0F/0x00` 33 帧，
         * 且都出现在送料完成之后、下一次送料之前。
         * 说明这台打印机用的是 0x0F 编码：bit3(0x08) = "正在使用"。
         * 只认 0x07 的后果：on_use / stop_on_use 永远匹配不上，
         * 状态卡在 before_on_use → 打印机认为 AMS 没进入使用态。
         *
         * 兼容写法：0x07（旧/理论值）与 0x0F（实测值）都接受。 */
        bool is_stop_on_use = ((statu_flags == 0x07) || (statu_flags == 0x0F)) && (motion_flag == 0x00);
        bool is_on_use      = ((statu_flags == 0x07) || (statu_flags == 0x0F)) &&
                              ((motion_flag == 0x7F) || (motion_flag == 0xFF));
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

    /* ★ v1.1.44：DE 时序对齐参考工程 esp32s3-485/main/comm_485.c
     *
     * 参考工程那段注释原话：「DE 的时序是这条总线最容易翻车的地方」，
     * 它的顺序是：DE 拉高 → **等 3us** → 写数据 → 等发完 → **等 2us** → DE 拉低。
     *
     * 本工程原来少了这两个微秒级等待：
     *   · DE 一拉高就立刻写 FIFO。收发器从"接收"切到"发送"需要几百 ns~1us，
     *     这段时间总线上电平不确定，首字节（0x3D 帧头）的起始位可能被削掉，
     *     打印机那边就是 CRC 错 → 整帧丢弃 → 该查询无应答 → `[0500-40A3]`。
     *     这类错误在我们自己的统计里完全看不见（我们发出去的字节是对的），
     *     所以 `crc_err / overflow / qdrop` 全 0 却仍然报通信异常。
     *   · flush() 之后立刻拉低 DE，最后一字节的停止位可能还没走完就被掐断。
     *
     * 代价：每帧多 5us。一帧本身 ~0.3ms，占比 1.6%，可以忽略。 */
    digitalWrite(BambuBus_pin_de, 1);     // DE 高 = 发送
    delayMicroseconds(3);                 // 等收发器换向完成
    RS485.write(data, length);
    RS485.flush();                        // 等待发送完成（含移位寄存器）
    delayMicroseconds(2);                 // 释放总线
    digitalWrite(BambuBus_pin_de, 0);     // DE 低 = 接收
    TX_IDLE = true;
    bus485raw_record(1, (const uint8_t *)data, length);  /* 原始报文记录：本机发出 */
}

uint8_t buf_X[1000];
CRC8 _RX_IRQ_crcx(0x39, 0x66, 0x00, false, false);

/* ★ v1.1.31：RX 帧队列 —— 修"连续帧互相覆盖导致查询无应答"
 *
 * 现场（2026-11-01）：删掉 2 台 AMS 只留 A/B 之后，连续发 3 次打印任务，
 * 前两次都报 `[0500-40A3] AMS A 通信异常`，第三次才正常开始打印。
 *
 * 根因（本文件）：
 *   原来 RX_IRQ 每收完一帧就 `memcpy(buf_X, ...)` 并置 `BambuBus_have_data`，
 *   而 `BambuBus_run()` **一次只处理一帧**。打印机在**打印任务开始时是连发的**
 *   （给每台 AMS 的每个槽位下发耗材信息、再逐个查询），
 *   一个 `BambuBus_UART_Poll()` 里收完两帧时，第二帧直接**覆盖**第一帧 →
 *   第一帧的查询永远没有应答 → 打印机判 AMS A 通信异常。
 *   这也解释了"第一次第二次失败、第三次成功"：第三次下发的内容和上次一样，
 *   打印机不再连发 set_filament，突发消失。
 *
 * 修法：收帧只入队（环形 8 帧），`BambuBus_run()` 里把队列排空再返回。
 * 帧长由 `length` 字段决定，是 uint8_t，最大 255，所以每槽 256 字节够用。
 */
/* ★ v1.1.33：8 → 32。
 * 打印任务开始时打印机是**成批连发**的（2 台 × 4 槽的下发 + 逐个查询），
 * 一次突发十几到二十帧。8 帧队列在"每处理一帧要阻塞发一包（~0.5ms）"
 * 的情况下很容易填满 → 丢帧 → 那几帧的查询没应答。
 * 32 帧 = 8KB RAM，对 320KB 的 S3 完全够用。 */
#define RX_QUEUE_DEPTH 32
#define RX_FRAME_MAX   256
static uint8_t  s_rx_q[RX_QUEUE_DEPTH][RX_FRAME_MAX];
static uint16_t s_rx_q_len[RX_QUEUE_DEPTH];
/* ★ v1.1.44：每帧的收齐时刻。用来量「从收到问 → 开始回」隔了多久。 */
static uint32_t s_rx_q_ts[RX_QUEUE_DEPTH];
static uint8_t  s_rx_q_head  = 0;
static uint8_t  s_rx_q_tail  = 0;
static uint8_t  s_rx_q_count = 0;
static uint32_t s_485_rx_dropped = 0;   /* 队列满 / 帧过长被丢弃的帧数 */

/* ★ v1.1.44：应答延迟统计（这是 40A3 最缺的那块数据）
 *
 * 为什么要加：`qdrop` 只统计 Cxx/Dxx 两类，长包查询（0x402 序列号 / 0x103 版本 /
 * 0x06 握手 / 0x21A / 0x211）**永远不会被计入 qdrop** —— 它们走到越界分支时
 * 是把台号 clamp 到 0 继续应答，不算"丢"。所以现场看到的 `qdrop` 全 0
 * **完全不能证明"每问都答了"**，这误导了 v1.1.35~v1.1.43 好几版。
 *
 * 真正会要命的是"答得太晚"：主循环里有同步写 flash（Bambubus_save，
 * 一次 10~50ms）、ADC2 在 WiFi 开着时读 GPIO11/12/13、WiFi 任务抢占……
 * 任何一次卡顿超过打印机那边的应答超时，它就记一次通信异常 → `[0500-40A3]`。
 * 而这种迟到在 rx/tx 计数上完全看不出来（我们确实回了，只是晚了几十毫秒）。
 *
 * 判读：
 *   lat_max < 10ms            → 应答及时，40A3 不是迟到引起的
 *   lat_max 几十~几百 ms      → 主循环卡顿，看 /status 的 loop_max_gap_ms
 *   lat_over100 在涨          → 直接就是它，必须去掉同步写 flash */
static uint32_t s_485_lat_max_ms = 0;
static uint32_t s_485_lat_over5   = 0;   /* 延迟 > 5ms 的应答次数 */
static uint32_t s_485_lat_over20  = 0;   /* 延迟 > 20ms */
static uint32_t s_485_lat_over100 = 0;   /* 延迟 > 100ms */

/* ★ v1.1.45：帧解析异常计数 + UART 积压峰值（见 RX_IRQ 注释） */
static uint32_t s_485_bad_len    = 0;   /* 长度字段非法 → 复位解析器的次数 */
static uint32_t s_485_rx_peak    = 0;   /* 单次 poll 时 UART 环形缓冲里的最大积压字节数 */

static bool rx_queue_push(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > RX_FRAME_MAX) {
        s_485_rx_dropped++;
        return false;
    }

    /* ★ v1.1.40：默认（bit4 关）= **完全照 v1.1.29 的语义 —— 只保留最后一帧**。
     *
     * 这才是真凶。打印机在打印任务开始时是**连发**的（2 台 × 4 槽下发 + 逐个查询）。
     *
     * v1.1.29：`memcpy(buf_X, ...)` 直接覆盖，一次 poll 里收到的多帧只剩最后一帧，
     *          所以回复永远发生在**突发结束、总线已经安静**之后 → 不撞车 → 能打印。
     * v1.1.31~v1.1.39：入队后逐帧回包。回第 N 帧的时候，打印机的第 N+1 帧
     *          **正在总线上传**，RS485 是半双工 —— 两边同时驱动 → 双方数据都成垃圾：
     *          · 我们收到的后续帧 CRC 错 → 不回（但我们以为回了）；
     *          · 打印机收到的我们的应答是垃圾 → 判 `AMS A 通信异常`。
     *
     * 这也解释了为什么"空闲时抓包两版回包逐字节一致、延迟都是 1ms"：
     * 空闲时打印机一帧一问、问完等答，**根本没有撞车的机会**。
     * 也解释了 `/status` 里 `qdrop` 全是 0（我们确实"答"了）却仍然 40A3
     * —— 答了，但被撞坏了，打印机没收到。
     *
     * 打开 bit4 = 入队 + 静默门控（见 BambuBus_run）：不丢帧也不撞车。 */
    if (!(bambubus_compat_get() & COMPAT_RX_BATCH)) {
        if (s_rx_q_count > 0) {
            s_485_rx_dropped++;      /* 被新帧覆盖掉的旧帧（= v1.1.29 的行为） */
        }
        memcpy(s_rx_q[0], data, len);
        s_rx_q_len[0] = len;
        s_rx_q_ts[0]  = millis();
        s_rx_q_head  = (uint8_t)(1 % RX_QUEUE_DEPTH);
        s_rx_q_tail  = 0;
        s_rx_q_count = 1;
        return true;
    }

    if (s_rx_q_count >= RX_QUEUE_DEPTH) {
        s_485_rx_dropped++;
        return false;
    }
    memcpy(s_rx_q[s_rx_q_head], data, len);
    s_rx_q_len[s_rx_q_head] = len;
    s_rx_q_ts[s_rx_q_head]  = millis();
    s_rx_q_head = (uint8_t)((s_rx_q_head + 1) % RX_QUEUE_DEPTH);
    s_rx_q_count++;
    return true;
}

static int rx_queue_pop(uint8_t *dst, uint16_t dst_max, uint32_t *out_ts)
{
    if (s_rx_q_count == 0) return 0;
    const uint16_t len = s_rx_q_len[s_rx_q_tail];
    if (len <= dst_max) memcpy(dst, s_rx_q[s_rx_q_tail], len);
    if (out_ts) *out_ts = s_rx_q_ts[s_rx_q_tail];
    s_rx_q_tail = (uint8_t)((s_rx_q_tail + 1) % RX_QUEUE_DEPTH);
    s_rx_q_count--;
    return (int)len;
}

/* ★ v1.1.25：485 链路计数（只读诊断，不参与任何逻辑）
 *
 * 为什么要加：现场"打印机报 [0500-40A3] AMS A 通信异常"时，光看现象
 * 分不清三种完全不同的原因：
 *   ① 物理层断了（线松/焊点虚/收发器坏/极性接反）
 *      → rx_bytes 完全不涨
 *   ② 收到帧但 CRC 大量错（波特率/干扰/共地不良/电源纹波）
 *      → rx_bytes 涨、rx_frames 不涨、crc_err 猛涨
 *   ③ 帧收得好好的，但我们对某类查询没应答（协议 bug）
 *      → rx_frames 正常涨、对应 tx_* 不涨
 * 刷一次机看 /status 的 ams.diag 就能定性，不用再靠抓包。 */
static uint32_t s_485_rx_bytes    = 0;  /* 收到的原始字节数 */
static uint32_t s_485_rx_frames   = 0;  /* CRC 校验通过、完整收下的帧数 */
static uint32_t s_485_rx_crc_err  = 0;  /* CRC8 校验失败的帧数 */
static uint32_t s_485_rx_overflow = 0;  /* 超长/半截帧被丢弃的次数 */
static uint32_t s_485_tx_402      = 0;  /* 0x402 序列号查询的应答次数 */
static uint32_t s_485_tx_103      = 0;  /* 0x103 版本查询的应答次数 */
static uint32_t s_485_tx_06       = 0;  /* 0x06 握手查询的应答次数 */
static uint32_t s_485_last_rx_ms  = 0;  /* 最近一次成功收帧的时刻 */
/* ★ v1.1.44：最近一次**收到任意字节**的时刻。
 *
 * 为什么必须单独记：v1.1.40 的"总线静默门控"用的是 `s_485_last_rx_ms`，
 * 它只在**整帧收完**时更新。一帧在传的过程中（0.3~2ms）门控是开着的，
 * 于是我们会在打印机**正在发**的时候插话 → 半双工撞车 → 双方数据都成垃圾。
 * 这就是"bit4（兼容位 16）试了也失败"最可能的原因：门控本身写错了，
 * 不是这个思路错。现在改成看"最后一个字节"的时刻。 */
static uint32_t s_485_last_byte_ms = 0;

/* ==========================================================================
 * 帧解析器（单字节状态机）
 *
 * ★ v1.1.45：按 BMCU-U 的 `src/printer_rx_framer.cpp` 加固。
 *
 * 原来的三个缺陷，每一个都会把"偶发丢一个字节"放大成"连续十几帧全丢"：
 *
 *   ① **长包长度只取 `buf[4]`，丢掉了 `buf[5]` 高字节。**
 *      协议里 `package_length` 是 uint16 小端（见 `long_packge_data`：
 *      `uint16_t package_number; uint16_t package_length; uint8_t crc8; ...`
 *      从 buf[2] 开始 memcpy 11 字节，所以长度跨 buf[4..5]）。
 *      BMCU-U 写的是 `header_[4] | (header_[5] << 8)`，我们原来只有低字节。
 *
 *   ② **长度字段没有合法性检查。** 丢一个字节就会把 length 读成 0~6 或 200+：
 *        · length 很小 → 立刻判"收帧完成"，交出一个垃圾帧，
 *          还可能触发一条垃圾应答发到总线上（进一步污染总线）；
 *        · length 很大 → 一直吞字节，最多吞到 999 才复位
 *          —— **一次丢字节能连带毁掉后面十几帧**。
 *      BMCU-U 的做法：`expected_length_ <= crc8_index_ || > kMaxFrameLength`
 *      就立刻 `reset()`，不等收完。
 *
 *   ③ **没有半帧超时。** 错位之后如果等不到 0x3D，就一直挂在半帧状态。
 *
 * 三条叠加就是"偶发 40A3"的放大器：
 *   主循环卡一下 → UART 环形缓冲溢出丢几个字节 → 解析器错位
 *   → 后续十几帧全部报废 → 打印机那几帧查询没人应答 → `[0500-40A3]`。
 *   而本地统计里 `crc_err` / `qdrop` 都是 0（我们压根没收到那些帧）。
 *
 * 顺带：原来 `_index >= 999` 那个保护跟"UART 溢出"毫无关系，
 * 所以现场一直看到 `overflow=0` —— 又一个诊断盲区。
 *
 * ★★★★ v1.1.45 初版踩的坑（"AMS 不显示"的真凶，必须记住）：
 *
 *   初版在帧头把 `s_rx_expect` 初始化成了 **0**，于是收帧判定
 *   `s_rx_index >= s_rx_expect` 在**第 1 个字节**就成立 →
 *   帧以 `len = 0` 判定完成 → `rx_queue_push(buf, 0)` 计入 rx_dropped
 *   → 一帧都没进队列 → 一条查询都没应答 → 打印机里 AMS 根本不显示。
 *
 *   **"期望长度"在读到长度字段之前必须是"大值"**：
 *     旧代码 = 6（`length = data_CRC8_index = 6`）
 *     BMCU-U = 999（`expected_length_ = 999u`）
 *   现在照 BMCU-U 用 999。
 *
 *   同理，长包的长度下标必须用 **5**（不是 4）：uint16 长度跨 buf[4..5]，
 *   在 index 4 就写进"期望长度"会让半截长度触发收帧判定。
 *   BMCU-U 的 `data_length_index_ = 5u` 就是这个道理。
 *
 *   教训：**"新旧算法结果等价"这种判断，必须把"期望长度初值"和
 *   "长度字段下标"一起算进去** —— 只比对最终算出的长度是不够的。
 * ========================================================================== */
#define RX_FRAME_TIMEOUT_MS 20    /* 一帧最多允许收 20ms（最长帧 ~2ms） */
#define RX_EXPECT_PENDING   999  /* 长度未读到时的"期望长度"（BMCU-U 同款） */

static int      s_rx_index   = 0;    /* 当前帧已收字节数，0 = 等帧头 */
static uint16_t s_rx_expect  = RX_EXPECT_PENDING; /* 本帧总长度（未读到时=999） */
static uint8_t  s_rx_len_idx = 2;    /* 长度字段所在下标（短包 2 / 长包 5） */
static uint8_t  s_rx_crc_idx = 3;    /* 头 CRC8 所在下标（短包 3 / 长包 6） */
static uint32_t s_rx_t0      = 0;    /* 帧头到达时刻（半帧超时用） */

static void RX_IRQ(unsigned char data)
{
    const uint32_t now = millis();

    /* ③ 半帧超时：错位后不会一直挂着 */
    if (s_rx_index > 0 && (uint32_t)(now - s_rx_t0) > RX_FRAME_TIMEOUT_MS)
    {
        s_485_rx_overflow++;
        s_rx_index = 0;
    }

    if (s_rx_index == 0)
    {
        if (data != 0x3D) return;
        BambuBus_data_buf[0] = 0x3D;
        _RX_IRQ_crcx.restart();
        _RX_IRQ_crcx.add(0x3D);
        s_rx_len_idx = 2;
        s_rx_crc_idx = 3;
        /* ★★★ 关键：长度没读到之前，期望长度必须是"大值"，不能是 0 !!!
         *
         * v1.1.45 初版在这里写的是 `s_rx_expect = 0`，于是收帧判定
         *   `s_rx_index >= s_rx_expect`
         * 在**第 1 个字节**就成立（1 >= 0）→ 帧以 `len = 0` 判定完成 →
         *   rx_queue_push(buf, 0) → len==0 → 计入 rx_dropped，帧被丢掉；
         *   bus485raw_record(buf, 0) → len==0 直接 return，什么都录不到；
         *   get_packge_type() 永远收不到完整帧 → 一条查询都不应答
         *   → 打印机里 AMS 根本不显示。
         *
         * 现象全部对得上：rx_dropped 与 rx_frames **严格 1:1**、
         * rx_bytes 在涨、crc_err/bad_len 全 0（长度校验那几行根本没执行到）、
         * /bus485raw 永远 RX_TOTAL=0。
         *
         * 旧代码在这里是 `length = data_CRC8_index = 6`（最小 6 字节），
         * BMCU-U 是 `expected_length_ = 999` —— 两者都是"大值"，
         * 保证帧头阶段不会被误判成收帧完成。这里照 BMCU-U 用 999。 */
        s_rx_expect  = RX_EXPECT_PENDING;
        s_rx_t0      = now;
        s_rx_index   = 1;
        return;
    }

    /* 保险：绝不允许写出缓冲区 */
    if (s_rx_index >= (int)sizeof(BambuBus_data_buf))
    {
        s_485_rx_overflow++;
        s_rx_index = 0;
        return;
    }

    BambuBus_data_buf[s_rx_index] = data;

    /* 第 2 个字节决定是短包（buf[1] 带 0x80）还是长包
     *
     * ★ v1.1.45 修正：长包的**长度下标改成 5**（不是 4）。
     *   长包长度是 uint16 小端，跨 buf[4..5]：
     *     - 若在 index 4 就把低字节写进 s_rx_expect，那么 index 4 之后
     *       `s_rx_index >= s_rx_expect` 可能被"半截长度"误触发；
     *     - BMCU-U 的做法就是 `data_length_index_ = 5`，到 index 5 才
     *       一次性算出 `header_[4] | (value << 8)`（短包才是 index 2）。
     *   这样"期望长度"要么还是 999（没读到），要么已经是完整长度。 */
    if (s_rx_index == 1)
    {
        if (data & 0x80) { s_rx_len_idx = 2; s_rx_crc_idx = 3; }   /* 短包 */
        else             { s_rx_len_idx = 5; s_rx_crc_idx = 6; }   /* 长包 */
    }

    if (s_rx_index < s_rx_crc_idx) _RX_IRQ_crcx.add(data);

    /* ① 长度字段：短包单字节(buf[2])，长包 16 位小端(buf[4..5]，index 5 才齐) */
    if (s_rx_index == s_rx_len_idx)
    {
        if (s_rx_len_idx == 2) {
            s_rx_expect = data;
        } else {
            s_rx_expect = (uint16_t)(BambuBus_data_buf[4] | ((uint16_t)data << 8));
        }

        /* ② 长度一到齐就检查合法性，不等收完 */
        if (s_rx_expect <= s_rx_crc_idx || s_rx_expect > RX_FRAME_MAX)
        {
            s_485_bad_len++;
            s_rx_index = 0;   /* 立刻复位，重新找 0x3D */
            return;
        }
    }

    if (s_rx_index == s_rx_crc_idx)
    {
        if (data != _RX_IRQ_crcx.calc())
        {
            s_485_rx_crc_err++;   /* v1.1.25 诊断 */
            s_rx_index = 0;
            return;
        }
    }

    s_rx_index++;

    if (s_rx_index >= (int)s_rx_expect)
    {
        const uint16_t len = s_rx_expect;
        s_rx_index = 0;
        /* ★ v1.1.31：入队，不再直接覆盖 buf_X（见 rx_queue_push 注释） */
        rx_queue_push(BambuBus_data_buf, len);
        BambuBus_have_data = s_rx_q_count;
        s_485_rx_frames++;                 /* v1.1.25 诊断 */
        s_485_last_rx_ms = now;
        bus485raw_record(0, BambuBus_data_buf, len);  /* 原始报文记录：打印机发来 */
    }
}

/* 从 UART 缓冲区读取并喂给协议解析器 */
static void BambuBus_UART_Poll(void)
{
    /* ★ v1.1.45：记录积压峰值。这是"主循环是不是被堵住了"最直接的证据 ——
     * 一旦接近 RX 缓冲大小，就说明有字节被 UART 驱动丢掉了。 */
    const int avail = RS485.available();
    if (avail > 0 && (uint32_t)avail > s_485_rx_peak) s_485_rx_peak = (uint32_t)avail;

    while (RS485.available() > 0)
    {
        unsigned char x = RS485.read();
        s_485_rx_bytes++;      /* v1.1.25 诊断：物理层是否有数据进来 */
        s_485_last_byte_ms = millis();   /* v1.1.44：静默门控看这个，不看整帧时刻 */
        RX_IRQ(x);
    }
}

void BambuBUS_UART_Init(uint32_t baudrate)
{
    /* 历史（v1.1.31 → v1.1.39）：
     *   v1.1.31 曾把 256 提到 2048；v1.1.39 又把它做成兼容位 bit5（默认关 = 256），
     *   理由是"现场确认 v1.1.29（256）能打印、v1.1.38（2048）不能"。
     *   **那个结论是错的**：v1.1.41（256）后来也照样报 40A3，
     *   说明能不能打印跟这一位无关，是概率问题。真正的原因见下面 v1.1.45。 */
    /* ★ v1.1.45（修正）：默认退回 256（= v1.1.29 / v1.1.41 证明能发现 AMS 的行为）。
     *
     * v1.1.45 初版把这里默认改成 8192，结果刷入后打印机里 AMS 不显示
     * （设备发现失败）。复盘：大缓冲本身不会丢帧，但为了避免任何变量、
     * 优先恢复「最后确认能发现并正常打印」的 256 行为。
     *
     * 8192 仍保留为 A/B 对照 —— 打开兼容位 bit5 才用（用来验证
     * "RX 溢出丢字节→40A3" 这个假设到底成不成立）。 */
    if (bambubus_compat_get() & COMPAT_RX_BIGBUF) {
        RS485.setRxBufferSize(8192);
        log_i("485 RX 缓冲 = 8192（兼容位 bit5 打开，BMCU-U 假设对照）");
    } else {
        RS485.setRxBufferSize(256);
        log_i("485 RX 缓冲 = 256（v1.1.46 默认 = v1.1.29 证明可用）");
    }

    /* ★ v1.1.46：兼容开关非 0 时**大声警告**。
     *
     * 这个坑踩过：调试时把兼容位设成 0x70（bit4+bit5+bit6）做 A/B，
     * 之后**它一直留在 NVS 里**（`s3ams-compat` 命名空间的 flags3 键），
     * 跨固件升级都带着 —— 于是
     *   bit4 → 静默门控生效（等总线安静 4ms 才回包）
     *   bit5 → RX 缓冲被改成 8192（覆盖掉固件里的默认值）
     *   bit6 → NVS 后台写
     * 全都还在跑，而固件里的"默认值"完全不起作用，
     * 现场表现就是"改了代码没效果"+"AMS 不显示"，极难查。
     *
     * 现在每次开机只要发现 compat != 0 就打一条醒目的警告，
     * 提醒先去网页「485 兼容位」恢复默认（0x00）。 */
    if (bambubus_compat_get() != COMPAT_DEFAULT)
    {
        log_w("★ 485 兼容开关非默认 (0x%02X)：批量收帧=%d 大RX缓冲=%d 异步存NVS=%d "
              "★ 固件默认值已被覆盖！如行为异常请到网页恢复默认(0x00)",
              (unsigned)bambubus_compat_get(),
              (bambubus_compat_get() & COMPAT_RX_BATCH) ? 1 : 0,
              (bambubus_compat_get() & COMPAT_RX_BIGBUF) ? 1 : 0,
              (bambubus_compat_get() & COMPAT_NVS_ASYNC) ? 1 : 0);
    }

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

    ams_fw_ver_load();   /* v1.1.13：上报给打印机的 AMS 版本号 */

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

    for (int a = 0; a < 4; a++)
    {
        for (int j = 0; j < 4; j++)
        {
            /* ★ v1.1.21：只有前 AMS_COUNT 台（=2）报 online，其余报 offline。
             *
             * v1.1.16 曾把全部 4×4 料位一律设 online（照 Tianshu），本意是修
             * "进料被 is_filament_online() 判离线挡住"。但本板 8 通道 ÷ 4 料位
             * 只有 2 台 AMS，AMS-C/AMS-D 是没有任何物理通道的幽灵设备 ——
             * 报 online 后 send_for_Fxx() 的 0x05 注册循环会把它们也注册上去，
             * 打印机的耗材页就多出 C/D 两组卡片。
             *
             * 保留 online 的初衷不变（有料位就要能进料），只是限定在真实存在的
             * 2 台之内。幽灵台报 offline 后，0x05/0x402/0x103/filament 长包
             * 都会因为 `>= AMS_COUNT` 直接 return，不再应答。 */
            data_save.filament[a][j].statu =
                (a < AMS_COUNT) ? online : offline;
            data_save.filament[a][j].motion_set = waiting;
        }
    }

    BambuBUS_UART_Init(1228800);

    /* ★ v1.1.31：启动"耗材信息落盘"后台任务（见 nvs_save_task 注释） */
    bambubus_nvs_task_start();
}

/* ★ v1.1.34：按当前 AMS_COUNT 重写 16 个料位的 statu。
 * WEB 改「AMS 台数」后调用，立即生效，不用重启。 */
void bambubus_apply_ams_count(void)
{
    for (int a = 0; a < 4; a++) {
        for (int j = 0; j < 4; j++) {
            data_save.filament[a][j].statu =
                (a < AMS_COUNT) ? online : offline;
        }
    }
    for (int i = 0; i < 4; i++) have_registered[i] = false;
    log_i("AMS 台数已改为 %d（%s），已重写 16 个料位状态",
          AMS_COUNT, (AMS_COUNT <= 2) ? "耗材页只有 A/B" : "耗材页会出现空的 C/D");
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
        /* ★ v1.1.35：以前落到 default 只记日志、不应答的三类，现在真回包 */
        case 0x411:
            return BambuBus_long_package_ext_411;
        case 0x23C:
            return BambuBus_long_package_ext_23C;
        case 0x237:
            return BambuBus_long_package_ext_237;
        default:
            /* 未实现的长包类型，记下来（3 秒限流）。
             * 2026-11-01 实测打印机在空闲时反复发 0x411 / 0x23C / 0x237，
             * 现在这三种已单独处理，能落到这里的都是**新**类型。 */
            {
                static uint32_t s_unk_last_ms = 0;
                if (millis() - s_unk_last_ms > 3000) {
                    s_unk_last_ms = millis();
                    log_w("485 收到未实现的长包类型 0x%03X（payload %d 字节），未应答",
                          (unsigned)printer_data_long.type,
                          (int)(printer_data_long.data_length));
                }
            }
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

/* ★ v1.1.33：按「台号 × 包类型」统计打印机问了我们多少次
 *
 * 为什么加：删掉 AMS-C/D 只留 A/B 之后，发打印任务**经常**报
 * `[0500-40A3] AMS A 通信异常`。已经猜错过三次（v1.1.21 / 22 / 31），
 * 这次先让数据说话 —— 关键就一个问题：
 *
 *   **打印机到底会不会来问台 2 / 台 3？**
 *
 *   · q[2] / q[3] 恒为 0 → 它只认 A/B，40A3 另有原因（继续查别处）
 *   · q[2] / q[3] 不为 0 → 打印机侧还残留"4 台 AMS"的配置，
 *     我们按 AMS_COUNT=2 静默不回，它就一直记成通信异常
 *
 * qdrop 记「问了但没应答」的次数 —— 这个数**只要涨就一定会报 40A3**。
 * 类型下标见 QK_*，`/status` 的 `ams.q` / `ams.qdrop` 直接可读。 */
#define QK_402 0   /* 0x402 序列号 */
#define QK_103 1   /* 0x103 版本   */
#define QK_06  2   /* 0x06  握手   */
#define QK_21A 3   /* 0x21A 耗材信息 */
#define QK_211 4   /* 0x211 MC 在线  */
#define QK_CXX 5   /* Cxx  送料状态查询 */
#define QK_DXX 6   /* Dxx  送料动作命令 */
#define QK_05  7   /* 0x05  注册（打印机问"你是不是第 N 台"） */
#define QK_05HI 8  /* 0x05  注册，但台号带 0x80 前缀（0x80|N） */
#define QK_411 9   /* 0x411 长包 */
#define QK_23C 10  /* 0x23C 长包 */
#define QK_237 11  /* 0x237 长包 */
#define QK_NUM 12
#define QK_UNITS 4

/* ★ v1.1.35：兼容开关。见 BambuBus.h 注释。
 *
 * ⚠️ v1.1.36 重要更正：**默认值必须是 0（全关）**。
 *
 * v1.1.35 曾把这四条全开，结果打印机报出一整屏：
 *   「AMS-HT A/B/C/D 加热器短路 / 异常发热 / 温度传感器离线」
 *   「AMS A/B/C/D 加热器1短路 / 加热器2故障」
 * 真相：
 *   · `0x05` 里 `0x80|n` 那一组 = **AMS-HT**（带烘干加热器的型号）的注册探测
 *   · `0x411` / `0x23C` / `0x237` = **烘干加热器状态查询**
 *   本板没有加热器，回一包全 0 的镜像包，打印机就理解成"加热器故障"，
 *   而应答 0x80|n 注册还会凭空造出 4 台 AMS-HT。
 *
 * 所以对这四类**保持静默才是正确语义**（"我没有这个设备/这个功能"），
 * 和 v1.1.20 能正常打印时的行为一致。开关保留只为以后做实验。
 *
 * ★ v1.1.39：把「v1.1.30 之后唯一还和 v1.1.29 不同的三处 485 收发行为」
 * 也做成开关（bit4~6），默认全关 = **完全退回 v1.1.29 的行为**。
 *
 * 背景：现场实测 v1.1.29（存档原厂固件）同设备同线**直接能打印**，
 * v1.1.38 一发打印任务就 `[0500-40A3] AMS A 通信异常`。
 * 而逐函数比对已证明两版的**应答内容与应答延迟完全一致**（只有滚动包号不同），
 * 所以差异只可能出在"什么时候发/什么时候被别的活堵住"这三点上：
 *
 *   bit4 COMPAT_RX_BATCH  每轮把 RX 队列排空（v1.1.31 起）vs 每轮只处理一帧（v1.1.29）
 *                         打开后一帧查询进来会连续回好几包，包间隔≈0；
 *                         v1.1.29 每轮只回一包，中间隔着 delay(2) 和 AMCU_run。
 *   bit5 COMPAT_RX_BIGBUF UART RX 环形缓冲 2048（v1.1.31 起）vs Arduino 默认 256（v1.1.29）
 *   bit6 COMPAT_NVS_ASYNC 耗材信息由 core0 后台任务写 flash（v1.1.31 起）
 *                         vs 就地同步写（v1.1.29）。
 *                         后台任务写 flash 时会关掉双核 cache，可能正好卡在一次查询中间。
 *
 * 现场用法：默认 0 先确认能打印；能打印后再逐位打开，看是哪一位把打印机惹毛的。
 * 位定义在文件最前面（BambuBUS_UART_Init 要用到 bit5，必须早定义）。 */

static uint8_t s_compat = COMPAT_DEFAULT;
static bool    s_compat_loaded = false;

static void compat_load(void)
{
    if (s_compat_loaded) return;
    s_compat_loaded = true;
    Preferences p;
    if (p.begin(COMPAT_NVS_NS, true)) {
        /* 键名带版本：v1.1.35 在 "flags" 里存过 15，v1.1.35~38 在 "flags2" 里
         * 也可能留着 15（调试时打开过加热器应答）。v1.1.39 换 "flags3"，
         * 保证这一版**从 0 开始**——否则一上电就带着"应答加热器"的旧设置，
         * 又会弹一屏加热器故障，把问题搅浑。 */
        s_compat = p.getUChar("flags3", COMPAT_DEFAULT);
        p.end();
    }
    s_compat &= COMPAT_ALL;
}

uint8_t bambubus_compat_get(void) { compat_load(); return s_compat; }

void bambubus_compat_set(uint8_t v)
{
    compat_load();
    s_compat = v & COMPAT_ALL;
    Preferences p;
    if (p.begin(COMPAT_NVS_NS, false)) {
        p.putUChar("flags3", s_compat);
        p.end();
    }
    log_i("485 兼容开关 = 0x%02X（reg_hi=%d 411=%d 23C=%d 237=%d | 批量收帧=%d 大RX缓冲=%d 异步存NVS=%d）",
          (unsigned)s_compat,
          (s_compat & COMPAT_REG_HI) ? 1 : 0,
          (s_compat & COMPAT_411) ? 1 : 0,
          (s_compat & COMPAT_23C) ? 1 : 0,
          (s_compat & COMPAT_237) ? 1 : 0,
          (s_compat & COMPAT_RX_BATCH) ? 1 : 0,          (s_compat & COMPAT_RX_BIGBUF) ? 1 : 0,
          (s_compat & COMPAT_NVS_ASYNC) ? 1 : 0);
}

static uint16_t s_q[QK_UNITS][QK_NUM];
static uint16_t s_q_drop[QK_NUM];

static inline void q_count(unsigned unit, int kind)
{
    if (unit < QK_UNITS) s_q[unit][kind]++;
}
static inline void q_drop(int kind) { s_q_drop[kind]++; }

/* ★ v1.1.35：台号字节可能带 0x80 标志位。
 *
 * 现场实测（v1.1.35 刷入后 15 秒窗口）：
 *   Dxx 送料命令的台号分布 = {0:92, 1:97, 2:105, 3:113,
 *                            0x80:119, 0x81:117, 0x82:106, 0x83:100}
 *   —— 打印机对**每个**台号同时用 0x00|n 和 0x80|n 两套地址在发，
 *   注册查询 0x05 也一样（0x80 那组 payload 长度为 0，没带指纹）。
 *
 * v1.1.34 及以前 0x80|n 全部落进 `>= AMS_COUNT` 的越界分支被静默丢弃，
 * 折算每秒 3.7 次"问了不回" —— 这就是 40A3 的持续来源。
 *
 * 处理原则：**剥掉 0x80 取真实台号去干活，回包时把原始地址字节镜像回去**。
 * 不新增逻辑台，不会造出幽灵 AMS；打印机问哪个地址，我们就从哪个地址答。 */
static inline uint8_t ams_unit_of(uint8_t addr) { return addr & 0x7F; }

void bambubus_query_stats(uint16_t out_unit[QK_UNITS][QK_NUM], uint16_t out_drop[QK_NUM])
{
    memcpy(out_unit, s_q, sizeof(s_q));
    memcpy(out_drop, s_q_drop, sizeof(s_q_drop));
}

void bambubus_query_stats_reset(void)
{
    memset(s_q, 0, sizeof(s_q));
    memset(s_q_drop, 0, sizeof(s_q_drop));
}

bool set_cxx_buf(unsigned char *set_buf, unsigned char *buf, int length)
{
    unsigned char AMS_num = buf[5];   /* v1.1.37：不剥 0x80，见 send_for_Dxx 注释 */
    unsigned char read_num = buf[7];
    now_op_num = read_num;
    float meters = -1;

    if (AMS_num >= AMS_COUNT) return false;

    meters = data_save.filament[AMS_num][read_num].meters;

    uint8_t flagx = get_filament_use_flag(AMS_num);
    if (flagx == 0x00) flagx = 0x02;

    set_buf[2] = flagx;
    set_buf[3] = read_num;

    memcpy(set_buf + 4, &meters, sizeof(meters));

    uint16_t pressure = get_pressure_value(AMS_num, read_num);
    memcpy(set_buf + 8, &pressure, sizeof(pressure));

    set_buf[24] = get_filament_left_char(AMS_num);
    return true;
}

void send_for_Cxx(unsigned char *buf, int length)
{
    q_count(buf[5], QK_CXX);

    Cxx_res[1] = 0xC0 | (package_num << 3);
    /* ★ v1.1.37：**不要**往 Cxx_res[5] 写台号。
     * v1.1.29（能打印的版本）里 Cxx_res[5] 一直是模板里的 0x00，
     * v1.1.35 擅自改成镜像 buf[5]，等于改了回包内容 —— 已回退。 */

    /* ★ v1.1.33：越界台号**不能回包**。
     *
     * 原来 set_cxx_buf() 里遇到 AMS_num >= AMS_COUNT 直接 return（什么都没改），
     * 但这里**照样**把 Cxx_res 发了出去 —— 发的是**上一台**的
     * flagx / read_num / meters / pressure，等于用 AMS-A 的数据冒充 AMS-C。
     * v1.1.20 时 AMS_COUNT=4 永远走不到这条分支，改成 2 台后才暴露。
     * 对打印机来说"问了 C，回来的数据却是 A 的"，比不回更糟。 */
    if (!set_cxx_buf(Cxx_res + 5, buf, length)) {
        q_drop(QK_CXX);
        return;
    }

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
    /* ★ v1.1.37：这里**必须用原始 buf[5]**，不能剥 0x80。
     *
     * v1.1.35 曾把 `0x80|n` 剥成 n 后照常应答并跑送料状态机 ——
     * 等于把 AMS-HT（另一种设备）的送料命令当成 AMS A/B 执行，
     * 打印机那边看到"命令被别的设备执行了"，表现就是
     * `[0500-40A3] AMS A 通信异常` + 自动取消打印任务。
     * v1.1.29 的原始写法是 `buf[5] >= AMS_COUNT` 直接 return（静默），
     * 那才是正确语义：我们没有 AMS-HT，就不该替它干活。 */
    unsigned char AMS_id  = buf[5];
    unsigned char AMS_num = buf[5];
    unsigned char statu_flags = buf[6];
    unsigned char fliment_motion_flag = buf[7];
    unsigned char read_num = buf[9];
    now_op_num = read_num;
    float meters = -1;
    uint8_t flagx = 0x02;

    q_count(AMS_num, QK_DXX);

    /* ★ v1.1.33：这里必须保持"越界不回"—— Dxx 会跑送料状态机
     * （process_motion_state_machine），台 2/3 会把 filament_ext[2][x]
     * 当成通道 8~11 去驱动，本板只有 8 路，绝不能碰。
     * v1.1.35：`0x80|n` 已剥掉高位，这里只剩真正的越界（台号 >= AMS_COUNT）。 */
    if (AMS_num >= AMS_COUNT) { q_drop(QK_DXX); return; }

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
        /* ★ 必须先判 < 16：BambuBus_now_filament_num 初值/全复位后是 0xFF，
           0xFF % 4 == 3 → 会去改"通道3"，把真正的送料通道漏掉 */
        if (data_save.BambuBus_now_filament_num < 16)
        {
            uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
            if (filament_ext[AMS_num][now_ch].motion == _filament_motion_internal::before_pull_back)
            {
                set_motion_internal(AMS_num, now_ch, _filament_motion_internal::pull_back);
            }
        }
    }
    else if (read_num == 0xFF && statu_flags == 0x01)
    {
        /* ★ 同上：0xFF % 4 == 3 的坑。未确定当前料位时直接忽略，
           不能瞎清通道 3（否则真实送料通道永远停不下来 / 被误清） */
        if (data_save.BambuBus_now_filament_num < 16)
        {
            uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
            /* ★ v1.1.12：打印中不要被"停止广播"清掉。
               打印机会周期性发 statu=0x01（本日志 1259 帧），
               一旦把正在打印的通道清成 idle，跟随送料/微动补料立刻断掉，
               现场表现就是"打一会儿停一会儿"。 */
            _filament_motion_internal m = filament_ext[AMS_num][now_ch].motion;
            if (m != _filament_motion_internal::on_use &&
                m != _filament_motion_internal::before_on_use &&
                m != _filament_motion_internal::stop_on_use)
            {
                set_motion_internal(AMS_num, now_ch, _filament_motion_internal::idle);
                time_sendout_onuse_ticks[now_ch] = 0;
            }
        }
    }
    else if (read_num == 0xFF)
    {
        /* ★★★ v1.1.12 最关键修复：不要把 0x0F 当成"清空" ★★★
         *
         * 现场抓包（2026-10-25 第四次）：
         *   15:06:10 打印机发 `type=04 statu=0F motion=00 read=FF`
         *   同一时刻我方应答 `use=0x00 pressure=0xFFFF 料位=FF`
         *   → 说明内部状态被整段清空了。
         *
         * 原来这里无条件执行：所有通道设 idle、now_filament_num = 0xFF。
         * 而打印机在"使用中"会周期性发 statu=0x0F + read=0xFF（本日志 33 帧），
         * 于是状态在"送料"和"全清"之间反复横跳：
         *   ① 电机反复启停 → 耗材断续 → 挤出机拉一点就拉不动
         *   ② now_filament_num 变成 0xFF 后，get_loaded_filament() 返回 0xFF
         *      → allow_stop 恒为 false → 打印机发 09/3F (before_pull_back)
         *      和 03/00/FF (pull_back) 全部被拒 → **退料电机不动**
         *      （这就是"有的时候退料没动作"的真正原因）
         *
         * 修法：只有明确的"复位/取消"（statu_flags == 0x00）才允许清空，
         * 其余未定义状态一律忽略并保留当前状态。 */
        if (statu_flags == 0x00)
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
    }

    uint16_t pressure = 0xFFFF;
    uint8_t use_flag = 0x00;
    if (read_num < 4)
    {
        pressure = get_pressure_value(AMS_num, read_num);
        use_flag = get_filament_use_flag(AMS_num);
    }
    else if (read_num == 0xFF && statu_flags != 0x01 &&
             data_save.BambuBus_now_filament_num < 16)
    {
        /* ★ v1.1.12：read=0xFF + statu=0x03/0x0F 时，打印机问的是"当前料位"的状态，
           不能回 0xFFFF（无耗材）。
           原来这里固定回 0xFFFF，打印机在 0x0F（使用中）阶段周期性查询时，
           看到 AMS 报"无耗材"，就会认为 AMS 掉线/空料 → 判挤出失败。
           statu=0x01 是"全部停止"广播，仍回 0xFFFF。 */
        uint8_t now_ch = data_save.BambuBus_now_filament_num % 4;
        pressure = get_pressure_value(AMS_num, now_ch);
        use_flag = get_filament_use_flag(AMS_num);
    }

    Dxx_res[1] = 0xC0 | (package_num << 3);
    Dxx_res[5] = AMS_id;      /* 镜像原始地址（含 0x80 标志位） */
    Dxx_res[9] = filament_flag_on;
    Dxx_res[10] = filament_flag_on - filament_flag_NFC;
    Dxx_res[11] = filament_flag_on - filament_flag_NFC;
    Dxx_res[12] = 0xFF;
    Dxx_res[17] = AMS_id;     /* 同上 */
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
    /* ★ v1.1.33：这三条分支必须按 buf[4]（子命令）分开判。
     *
     * 原来第一条是 `if (buf[5] == 0x00)`，**没看子命令** ——
     * 打印机发 0x06 握手、台号正好是 0 时（buf[4]=0x06, buf[5]=0x00），
     * 会误进"广播注册"分支：回两包注册包，却**完全不应答这次握手**，
     * 打印机超时 → `[0500-40A3]`。这是从 v1.1.16 就带进来的老问题，
     * 只是因为 AMS_COUNT=4 时分支组合更多、被掩盖了。 */
    const uint8_t sub = buf[4];   /* 0x05 = 注册查询，0x06 = 握手 */

    if (sub == 0x05 && buf[5] == 0x00)
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
    else if (sub == 0x05 && buf[5] == 0x01)
    {
        /* ★ v1.1.37：恢复 v1.1.29 的写法 —— **不做 0x80 剥离**。
         *
         * 打印机对每个台号会问两遍：`0x00|n`（带 20 字节指纹）和
         * `0x80|n`（payload 长度 0）。后者是 **AMS-HT** 的注册探测。
         * v1.1.35 把它剥成 n 后照常应答，结果打印机凭空注册出
         * AMS-HT A/B/C/D 四台，还连带报一屏加热器故障。
         * 正确语义：`buf[6] >= AMS_COUNT` 就静默（"我们没有这台"）。 */
        const uint8_t req_id  = buf[6];
        const bool    hi_flag = (req_id & 0x80) != 0;

        /* ★ v1.1.37：恢复 v1.1.29 的 `buf[6] < AMS_COUNT` 语义。
         * `0x80|n` = AMS-HT 的注册探测，>= AMS_COUNT 就是"我们没有这台"，
         * **静默是正确行为**，不计入 qdrop。 */
        q_count(req_id & 0x7F, hi_flag ? QK_05HI : QK_05);
        if (req_id >= AMS_COUNT) return;

        if (data_save.filament[req_id][0].statu != online)
        {
            have_registered[req_id] = false;
            return;
        }

        F00_res[req_id][0] = 0x3D;
        F00_res[req_id][1] = 0xC0;
        F00_res[req_id][2] = 29;
        F00_res[req_id][3] = 0xB4;
        F00_res[req_id][4] = 0x05;
        F00_res[req_id][5] = 0x01;
        F00_res[req_id][6] = req_id;

        package_send_with_crc(F00_res[req_id], 29);
        s_485_tx_06++;   /* v1.1.25 诊断：0x06 握手已应答 */

        if (memcmp(F00_res[req_id] + 7, buf + 7, 20) == 0)
            have_registered[req_id] = true;
        else
            have_registered[req_id] = false;
    }
    else if (sub == 0x06)
    {
        uint8_t req_num = buf[5];

        /* ★ v1.1.22：撤销 v1.1.21 在这里加的 `if (req_num >= AMS_COUNT) return;`。
         *
         * 0x06 是打印机**逐台握手确认**的帧，参考实现 esp32s3-485 对它
         * 是无条件回包的。少回一次就是"链路上有一次无应答"，打印机侧
         * 很容易记成通信异常 —— 和 0x402 是同一类错误。
         * 重复 AMS 卡片的问题不在"回不回包"，而在这里无条件回、且
         * res_for_06_num 会把所有台号都改成同一个（见下方逻辑，暂未动）。 */
        uint8_t ams_num = req_num;
        q_count(req_num, QK_06);
        if (res_for_06_num == 0xFF)
        {
            res_for_06_num = req_num;
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

    /* ★ v1.1.37：恢复 v1.1.29 语义 —— 台号越界或该台不在线就**静默**。
     * v1.1.33 曾改成"只挡数组越界、一律回空包"，但 v1.1.29 那样才是能打印的。 */
    const uint8_t AMS_num = printer_data_long.datas[0];
    q_count(AMS_num, QK_21A);

    if (AMS_num >= AMS_COUNT ||
        data_save.filament[AMS_num][0].statu != online) return;

    data.datas = long_packge_MC_online;
    data.datas[0] = AMS_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}

/* ★ v1.1.35：0x411 / 0x23C / 0x237 三种"从未应答过"的长包。
 *
 * 现场实测（打印机空闲，25 秒窗口）：0x411 271 帧、0x23C 70 帧、0x237 22 帧，
 * 我们**一帧都没回**。三者结构和 0x21A（我们一直在回的"MC 在线"查询）
 * 完全同族 —— 同一个包头 `3D 05 00 00 <len> <crc8> 00 07 00 03`，
 * 也就是 printer(0x0300) → AMS(0x0700)，标准"查询-应答"格式。
 * 0x411 的 payload 是 4 字节且 [0] 在 0~3 之间轮转（就是台号），
 * 和 0x21A 的请求格式一模一样。
 *
 * 这里用**镜像回包**：类型、包号原样带回，payload 长度与请求一致、
 * 内容照抄请求（这几类请求的 payload 除台号外全是 0），源地址改成 AMS。
 * 不做语义猜测 —— 打印机问什么地址，我们就从什么地址答什么。
 *
 * 风险控制：三条都挂在 bambubus_compat_set() 的开关上，出问题可以
 * 单独关掉任何一条退回 v1.1.34 的静默行为。 */
static void send_for_ext_long(uint16_t type, uint8_t flag, int kind)
{
    uint8_t AMS_id = (printer_data_long.data_length > 0) ? printer_data_long.datas[0] : 0;
    q_count(ams_unit_of(AMS_id), kind);

    if (!(bambubus_compat_get() & flag)) {
        /* v1.1.36：这三类是**烘干加热器状态查询**。本板没有加热器，
         * 回包反而会让打印机报"加热器短路/过热"，所以默认静默。
         * 静默是正确语义，不计入 qdrop（qdrop 只统计该回没回的）。 */
        static uint32_t s_heat_last_ms = 0;
        if (millis() - s_heat_last_ms > 10000) {
            s_heat_last_ms = millis();
            log_i("485 收到加热器查询 0x%03X（本板无加热器，按协议静默）",
                  (unsigned)type);
        }
        return;
    }

    static uint8_t echo[64];
    uint16_t n = printer_data_long.data_length;
    if (n > sizeof(echo)) n = sizeof(echo);
    memset(echo, 0, sizeof(echo));
    if (n > 0) memcpy(echo, printer_data_long.datas, n);

    long_packge_data data;
    data.datas = echo;
    data.data_length = n;
    data.package_number = printer_data_long.package_number;
    data.type = type;
    data.source_address = BambuBus_address_AMS08;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}

void send_for_long_packge_ext_411(unsigned char *buf, int length)
{
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    send_for_ext_long(0x411, COMPAT_411, QK_411);
}

void send_for_long_packge_ext_23C(unsigned char *buf, int length)
{
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    send_for_ext_long(0x23C, COMPAT_23C, QK_23C);
}

void send_for_long_packge_ext_237(unsigned char *buf, int length)
{
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    send_for_ext_long(0x237, COMPAT_237, QK_237);
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

    /* ★ v1.1.37：恢复 v1.1.29 语义（台号越界 / 料位越界 / 该台不在线 → 静默）。 */
    const uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];

    q_count(AMS_num, QK_211);

    if (AMS_num >= AMS_COUNT || filament_num >= 4 ||
        data_save.filament[AMS_num][filament_num].statu != online) return;

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
    s_485_tx_402++;   /* v1.1.25 诊断：0x402 序列号查询已应答 */
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
        /* 台号在 payload[33]；包不够长时退回 payload[0]（esp32s3-485 同做法） */
        if (printer_data_long.data_length > 33)
            AMS_num = printer_data_long.datas[33];
        else if (printer_data_long.data_length > 0)
            AMS_num = printer_data_long.datas[0];

        q_count(AMS_num, QK_402);

        /* ★ v1.1.22：越界台号必须 clamp 到 0 后**继续应答**，绝不能 return。
         *
         * v1.1.21 曾把这里改成 `if (AMS_num >= AMS_COUNT) return;`（照其他长包
         * "越界就不应答"的写法统一），结果打印机在枚举 AMS-A 时报
         * `[0500-40A3] AMS(或AMS lite) A 通信异常`。
         *
         * 原因：0x402 的台号并不总在 payload[33]。包长 ≤ 33 时会退到 payload[0]，
         * 而 payload[0] 不一定是台号（可能是别的字段），值一旦 >= AMS_COUNT
         * 就 return → 打印机**一个序列号包都收不到** → 判定 A 通信异常。
         * 参考实现 esp32s3-485 的 on_version() 同样先 `unit = pl[0]` 兜底，
         * 但紧接着是 `if (unit >= AMS_UNITS) unit = 0;` —— **clamp 不是 return**。
         *
         * 宁可回一个"台 0 的序列号"（最多让耗材页多张卡片，不影响打印），
         * 也绝不能让链路上出现"查询无应答"（直接报通信异常、无法打印）。 */
        if (AMS_num >= AMS_COUNT) AMS_num = 0;

        payload = long_packge_version_serial_number_AMS08;
        payload_len = sizeof(long_packge_version_serial_number_AMS08);

        payload[4] = 0x30 + AMS_num;
        payload[34] = 0xA0 + AMS_num;
        payload[65] = AMS_num;
    }
    else if (printer_data_long.type == 0x103)
    {
        /* ★ v1.1.22：长度回退 21 字节（v1.1.21 曾改成 22）。
         *
         * 22 字节是参考实现 esp32s3-485 的做法（b[20]/b[21] 都写台号），
         * 但本工程 1.1.20 及以前一直是 21 字节、台号只写 [20]，且**实测能
         * 正常识别 AMS-A/B**。既然 21 是已验证行为，就不要为"多写一个
         * 台号副本"去改长包的 data_length —— 打印机会校验长度，
         * 长度变了就可能整包丢弃，表现同样是通信异常。
         *
         * 包结构（21 字节）：
         *   [0..3]  版本号 uint32 小端，例如 01.00.06.87 → 57 06 00 01
         *   [4..8]  型号代号 "AMS08"（AMS / AMS 2 Pro / AMS HT 这条线）
         *   [9..19] 补 0
         *   [20]    台号
         *
         * 打印机会同时校验「版本号是否在白名单」和「型号代号」，两者任一
         * 对不上都可能弹「当前固件仅支持 AMS / AMS 2 Pro / AMS HT」。 */
        static unsigned char ver_pkt[21];
        q_count(AMS_num, QK_103);
        uint32_t v = s_ams_fw_ver;
        ver_pkt[0] = (unsigned char)(v & 0xFF);
        ver_pkt[1] = (unsigned char)((v >> 8) & 0xFF);
        ver_pkt[2] = (unsigned char)((v >> 16) & 0xFF);
        ver_pkt[3] = (unsigned char)((v >> 24) & 0xFF);
        memcpy(ver_pkt + 4, "AMS08", 5);
        memset(ver_pkt + 9, 0x00, 11);   /* [9..19] 清零 */
        ver_pkt[20] = AMS_num;

        payload = ver_pkt;
        payload_len = sizeof(ver_pkt);
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

/* ★ v1.1.23：缓冲微动用的"该通道正在被打印机使用"判定。
 *
 * 比 is_channel_printing() 多认一个 send_out（送料中）。
 *
 * 现场（2026-10-31）：流量校准阶段微动压下不送料，而 v1.1.14（微动直连、
 * 不看状态）是正常的。查下来是 check_buffer_status() 用 is_channel_printing()
 * 做准入，而它只认 on_use / before_on_use / stop_on_use —— **不含 send_out**。
 * 流量校准时打印机一直在从 AMS 抽料，但状态机停在 send_out（或尚未转到
 * on_use），于是整段 return，微动触发也白搭。
 *
 * 缓冲送料的本意就是"缓冲器空了就补"，只要该通道正在被打印机使用
 * （送料中 / 进入使用 / 使用中 / 停止使用）就该生效。 */
bool is_channel_in_use(int num)
{
    if (num < 0 || num >= 16) return false;
    switch (filament_ext[num / 4][num % 4].motion)
    {
    case _filament_motion_internal::send_out:
    case _filament_motion_internal::before_on_use:
    case _filament_motion_internal::on_use:
    case _filament_motion_internal::stop_on_use:
        return true;
    default:
        return false;
    }
}

/* ★ v1.1.23：该通道"最近是否被打印机当成在用"—— 看 statu 位掩码，不看 motion。
 *
 * 流量校准阶段打印机持续从 AMS 抽料，但状态机可能一直停在 idle
 * （帧没匹配上 on_use/before_on_use），这时 is_channel_in_use() 也是 false。
 * 只要最近 window_ms 内打印机对该通道报过 0x03/0x07/0x09/0x0F 之一，
 * 就认为它"在用"，缓冲微动应该生效。
 *
 * ★★★ v1.1.27：退料阶段必须排除 ★★★
 *
 * 现场（2026-10-31）：**打印结束、退料过程中，开始微动一触发就又把料
 * 送进挤出机了**。
 *
 * 根因就在这个函数：`statu == 0x09` 有两种完全相反的含义 ——
 *     0x09 + motion_flag 0xA5 = before_on_use    （准备使用，该送料）
 *     0x09 + motion_flag 0x3F = before_pull_back （准备退料，绝不能送料）
 * 这里只看 statu、不看 motion_flag，于是"准备退料"被当成"在用" →
 * 缓冲准入放行 → 微动一触发就 AMCU_drive_channel(FEED) →
 * **退料刚退出来的料又被塞回挤出机**。
 *
 * 所以只要 motion 已经是 before_pull_back / pull_back，一律不算"在用"。 */
bool is_channel_active_recently(int num, uint32_t window_ms)
{
    if (num < 0 || num >= 16) return false;
    const _filament_ext *ext = &filament_ext[num / 4][num % 4];
    if (ext->last_statu_ms == 0) return false;

    /* ★ v1.1.27：退料阶段一律不算"在用"（见上） */
    if (ext->motion == _filament_motion_internal::before_pull_back ||
        ext->motion == _filament_motion_internal::pull_back) {
        return false;
    }

    const uint8_t s = ext->last_statu;
    const bool in_use = (s == 0x03) || (s == 0x07) || (s == 0x09) || (s == 0x0F);
    if (!in_use) return false;

    return ((uint32_t)get_time64() - ext->last_statu_ms) < window_ms;
}

/* ★ v1.1.27：该通道是否处于"退料阶段"（准备退料 / 正在退料）
 *
 * 缓冲补料必须整段让出 —— 不只是 is_need_pull_back 那一瞬间。
 * 原因：退料 3 秒超时后 is_need_pull_back 就被清了，但打印机可能还在
 * 退料流程里（motion 仍是 pull_back），此时微动一触发就会把刚退出来的
 * 料又送回去。 */
bool is_channel_pulling_back(int num)
{
    if (num < 0 || num >= 16) return false;
    const _filament_motion_internal m = filament_ext[num / 4][num % 4].motion;
    return (m == _filament_motion_internal::before_pull_back ||
            m == _filament_motion_internal::pull_back);
}

/* 供 /status 诊断：该通道最近一次收到的 statu（0 表示还没收到过） */
uint8_t get_channel_last_statu(int num)
{
    if (num < 0 || num >= 16) return 0;
    return filament_ext[num / 4][num % 4].last_statu;
}

/* ★ v1.1.25：485 链路诊断计数快照（供 /status 的 ams.diag 输出）
 *
 * 判读表（刷完看一次 /status 即可定性 40A3）：
 *   rx_bytes 不涨             → 物理层没数据：线松/焊点虚/收发器坏/A-B 接反
 *   rx_bytes 涨、crc_err 猛涨 → 干扰或波特率/共地问题（电机电源纹波常见）
 *   rx_frames 涨、tx_402=0    → 我们没应答序列号查询（协议分支 bug）
 *   tx_402 涨、仍报 40A3      → 应答内容不对（台号/长度/型号指纹），查协议内容
 *   last_rx_age 很大          → 打印机根本没在跟我们说话（对端/接线） */
void bambubus_485_stats(uint32_t *rx_bytes, uint32_t *rx_frames, uint32_t *crc_err,
                        uint32_t *overflow, uint32_t *tx_402, uint32_t *tx_103,
                        uint32_t *tx_06, uint32_t *last_rx_ms)
{
    if (rx_bytes)  *rx_bytes  = s_485_rx_bytes;
    if (rx_frames) *rx_frames = s_485_rx_frames;
    if (crc_err)   *crc_err   = s_485_rx_crc_err;
    if (overflow)  *overflow  = s_485_rx_overflow;
    if (tx_402)    *tx_402    = s_485_tx_402;
    if (tx_103)    *tx_103    = s_485_tx_103;
    if (tx_06)     *tx_06     = s_485_tx_06;
    if (last_rx_ms) *last_rx_ms = s_485_last_rx_ms;
}

/* v1.1.25：清零 485 链路计数（改接线后重新开始计数，便于对照） */
void bambubus_485_stats_reset(void)
{
    s_485_rx_bytes = s_485_rx_frames = s_485_rx_crc_err = s_485_rx_overflow = 0;
    s_485_tx_402 = s_485_tx_103 = s_485_tx_06 = 0;
    s_485_last_rx_ms = 0;
    s_485_last_byte_ms = 0;    /* ★ v1.1.44 */
    s_485_rx_dropped = 0;      /* ★ v1.1.31 */
    s_485_lat_max_ms = 0;      /* ★ v1.1.44 */
    s_485_lat_over5 = s_485_lat_over20 = s_485_lat_over100 = 0;
    s_485_bad_len = 0;         /* ★ v1.1.45 */
    s_485_rx_peak = 0;
    s_rx_index = 0;            /* 解析器一起复位，避免残留半帧 */
    s_rx_expect = RX_EXPECT_PENDING;  /* ★ v1.1.45：必须是大值，不能是 0 */
    s_rx_len_idx = 2;
    s_rx_crc_idx = 3;
    s_rx_t0 = 0;
}

/* ★ v1.1.31：队列丢帧计数（正常恒 0） */
uint32_t bambubus_rx_dropped(void) { return s_485_rx_dropped; }

/* ★ v1.1.44：应答延迟统计快照（供 /status 的 ams.lat 输出）
 * 判读见 s_485_lat_max_ms 的注释。 */
void bambubus_485_lat_stats(uint32_t *max_ms, uint32_t *over5,
                            uint32_t *over20, uint32_t *over100)
{
    if (max_ms)  *max_ms  = s_485_lat_max_ms;
    if (over5)   *over5   = s_485_lat_over5;
    if (over20)  *over20  = s_485_lat_over20;
    if (over100) *over100 = s_485_lat_over100;
}

/* ★ v1.1.45：帧解析异常 + UART 积压峰值（供 /status 的 ams.frame 输出）
 *
 *   bad_len  长度字段非法 → 复位解析器的次数
 *            **只要涨，就说明总线上真的丢过字节**（丢字节把 length 读坏了）
 *   peak     单次 poll 时 UART 环形缓冲里的最大积压字节数
 *            接近缓冲大小（8192）= 主循环被堵住，字节正在被丢 */
void bambubus_485_frame_stats(uint32_t *bad_len, uint32_t *peak)
{
    if (bad_len) *bad_len = s_485_bad_len;
    if (peak)    *peak    = s_485_rx_peak;
}

/* 打印阶段细分查询（v1.1.12）—— 见 BambuBus.h 注释 */
bool is_channel_before_on_use(int num)
{
    if (num < 0 || num >= 16) return false;
    return filament_ext[num / 4][num % 4].motion == _filament_motion_internal::before_on_use;
}

bool is_channel_on_use(int num)
{
    if (num < 0 || num >= 16) return false;
    return filament_ext[num / 4][num % 4].motion == _filament_motion_internal::on_use;
}

bool is_channel_stop_on_use(int num)
{
    if (num < 0 || num >= 16) return false;
    return filament_ext[num / 4][num % 4].motion == _filament_motion_internal::stop_on_use;
}

unsigned char get_now_op_num()
{
    return now_op_num;
}

// ==================== 耗材信息落盘（★ v1.1.31 移出 485 热路径）====================
/*
 * 现场（2026-11-01）：删掉 2 台 AMS 只留 A/B 后，连续发 3 次打印任务，
 * 前两次报 `[0500-40A3] AMS A 通信异常`，第三次才正常。
 *
 * 根因之二（本段）：
 *   打印机在**打印任务开始时**会给每台 AMS 的每个槽位下发一次耗材信息
 *   （0xC5 0x08 / 0x05 0x218），而 `send_for_Set_filament()` 里会调
 *   `Bambubus_set_need_to_save()`；原来 `BambuBus_run()` 末尾**立刻**
 *   `Bambubus_save()` 同步写 NVS —— 一次 putBytes(≈800B) + end() 提交，
 *   实测 10~50ms，NVS 页满时还要擦除（擦除期间**两颗核的 flash cache 全停**）。
 *   8 个槽位连发就是 8 次写，累计上百毫秒 → 这期间 485 一个应答都发不出去
 *   → 打印机判 AMS A 通信异常。
 *
 * 修法：
 *   ① 485 侧只置一个 dirty 标志（微秒级），**绝不写 flash**
 *   ② 独立低优先级任务跑在 core 0（485 主循环在 core 1），
 *      防抖 1.2 秒把这一批合并成**一次**写
 *   ③ 内容没变就不写（memcmp 快照）—— 打印机重复下发同样内容时不碰 flash
 *   ④ 记录耗时，`/status` 的 `ams.nvs` 能直接看到最坏值
 */
static volatile bool s_save_dirty = false;
static uint32_t s_nvs_saves = 0;
static uint32_t s_nvs_save_max_ms = 0;
static uint32_t s_nvs_save_total_ms = 0;
static uint32_t s_nvs_skipped = 0;      /* 内容未变而跳过的次数 */
static uint8_t  s_nvs_snapshot[sizeof(data_save)];
static bool     s_nvs_snapshot_valid = false;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_nvs_task = NULL;

static void nvs_save_task(void *arg)
{
    (void)arg;
    static flash_save_struct snap;

    for (;;)
    {
        if (!s_save_dirty) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        /* 防抖：把这一批 set_filament 全部等完，合并成一次写 */
        vTaskDelay(pdMS_TO_TICKS(1200));
        if (!s_save_dirty) continue;
        s_save_dirty = false;

        /* 拷贝期间关中断，避免 485 侧改到一半（约 800B，几十微秒） */
        portENTER_CRITICAL(&s_data_mux);
        memcpy(&snap, &data_save, sizeof(snap));
        portEXIT_CRITICAL(&s_data_mux);

        if (s_nvs_snapshot_valid &&
            memcmp(s_nvs_snapshot, &snap, sizeof(snap)) == 0) {
            s_nvs_skipped++;
            continue;   /* 内容没变，不碰 flash */
        }

        const uint32_t t0 = millis();
        s_prefs.begin("s3ams", false);
        s_prefs.putBytes(FLASH_KEY, &snap, sizeof(snap));
        s_prefs.end();
        const uint32_t dt = millis() - t0;

        memcpy(s_nvs_snapshot, &snap, sizeof(snap));
        s_nvs_snapshot_valid = true;
        s_nvs_saves++;
        s_nvs_save_total_ms += dt;
        if (dt > s_nvs_save_max_ms) s_nvs_save_max_ms = dt;

        /* 只在慢的时候打日志，正常情况不刷屏 */
        if (dt >= 30) {
            log_w("NVS 写耗材信息 %uB 用时 %ums（后台任务，已不挡 485）",
                  (unsigned)sizeof(snap), (unsigned)dt);
        }
    }
}

void bambubus_nvs_task_start(void)
{
    if (s_nvs_task != NULL) return;
    /* core 0 / 优先级 1：远低于 loopTask(core1) 与离合任务，绝不抢 485 的 CPU */
    xTaskCreatePinnedToCore(nvs_save_task, "nvs_save", 4096, NULL, 1, &s_nvs_task, 0);
}

void bambubus_nvs_stats(uint32_t *saves, uint32_t *skipped,
                        uint32_t *max_ms, uint32_t *total_ms, bool *pending)
{
    if (saves)    *saves    = s_nvs_saves;
    if (skipped)  *skipped  = s_nvs_skipped;
    if (max_ms)   *max_ms   = s_nvs_save_max_ms;
    if (total_ms) *total_ms = s_nvs_save_total_ms;
    if (pending)  *pending  = s_save_dirty;
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

    /* ★ v1.1.40：收帧/回包节奏，受 COMPAT_RX_BATCH（bit4）控制。
     *
     * 关（默认）= v1.1.29 的语义：
     *   `rx_queue_push()` 只保留**最后一帧**（新帧覆盖旧帧），
     *   所以回复一定发生在**突发结束、总线安静**之后 —— 不撞车。
     *   这是现场唯一确认过"能打印"的行为。
     *
     * 开 = 入队 + **静默门控**（v1.1.40 新方案）：
     *   帧全部入队不丢，但要等总线**连续 3ms 没有新字节**才把队列排空回包。
     *   既不像 v1.1.29 那样丢查询，也不像 v1.1.31~39 那样边收边回撞车。
     *
     * 为什么门控是必须的：RS485 半双工，打印机连发时我们插话 →
     * 双方数据都成垃圾 → 打印机判 `AMS A 通信异常`。 */
    const bool batch = (bambubus_compat_get() & COMPAT_RX_BATCH) != 0;
    if (batch) {
        /* 总线还在动（4ms 内收到过**任意字节**）就先别开口。
         * ★ v1.1.44：判据从 `s_485_last_rx_ms`（整帧收完的时刻）改成
         *   `s_485_last_byte_ms`（最后一个字节的时刻）。
         *   原来一帧在传的过程中门控是开着的，等于没有门控 ——
         *   这是 v1.1.40 里 bit4 实测失败的直接原因。 */
        if ((uint32_t)(millis() - s_485_last_byte_ms) < 4) {
            return stu;
        }
    }
    const int max_frames = batch ? RX_QUEUE_DEPTH : 1;
    int processed = 0;
    while (s_rx_q_count > 0 && processed < max_frames)
    {
        uint32_t rx_ts = 0;
        const int data_length = rx_queue_pop(buf_X, sizeof(buf_X), &rx_ts);
        if (data_length <= 0) break;
        processed++;
        stu = 1;
        time_set = timex + 1000;

        /* ★ v1.1.44：量「收到问 → 开始回」的延迟（见 s_485_lat_max_ms 注释） */
        {
            const uint32_t lat = (uint32_t)(millis() - rx_ts);
            if (lat > s_485_lat_max_ms) s_485_lat_max_ms = lat;
            if (lat > 5)   s_485_lat_over5++;
            if (lat > 20)  s_485_lat_over20++;
            if (lat > 100) s_485_lat_over100++;
        }

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
        case BambuBus_long_package_ext_411:
            send_for_long_packge_ext_411(buf_X, data_length);
            break;
        case BambuBus_long_package_ext_23C:
            send_for_long_packge_ext_23C(buf_X, data_length);
            break;
        case BambuBus_long_package_ext_237:
            send_for_long_packge_ext_237(buf_X, data_length);
            break;
        default:
            break;
        }
    }
    BambuBus_have_data = s_rx_q_count;

    /* ★ v1.1.31：只置脏标志，**绝不在这里写 flash**
     * （原来这里同步 Bambubus_save()，一次 10~50ms，8 个槽位连发上百毫秒，
     *   期间 485 完全不应答 —— 见 nvs_save_task 的注释）
     *
     * ★ v1.1.39：改成受 COMPAT_NVS_ASYNC（bit6）控制。
     *
     * ★ v1.1.45（修正）：恢复 v1.1.41 / v1.1.44 的原逻辑 —— 就地写，
     * 不"等总线安静 200ms 再写"。
     *
     * v1.1.45 初版把 NVS 写延后到"总线安静 200ms 后"，实测结果是
     * 打印机里 AMS 直接不显示。复盘：注册/发现阶段本就不频繁写 NVS，
     * 延后写不会改善 AMS 发现，反而多引入一个未验证的状态机，
     * 增加了变量。先把这一路退回已证明能发现 AMS 的写法。 */
    if (Bambubus_need_to_save)
    {
        Bambubus_need_to_save = false;
        if (bambubus_compat_get() & COMPAT_NVS_ASYNC) {
            s_save_dirty = true;
        } else {
            Bambubus_save();   /* 同步写（= v1.1.29 行为） */
        }
    }

    return stu;
}
