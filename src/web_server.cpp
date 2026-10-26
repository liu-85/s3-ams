/**
 * web_server.cpp —— ESP32-S3 Web 服务器（Arduino 框架）
 *
 * 保留 esp32s3-485 的 Web 前端，提供兼容后端。
 * 使用 ESP32 Arduino 的 WiFi + WebServer + Update 库。
 */

/* ams_log.h 必须第一个 include：它先拉 Arduino.h 拿到原始 log_* 宏，
   再劫持成「串口照旧 + 抄送网页日志」。见文件头注释。 */
#include "ams_log.h"
#include "web_server.h"
#include "web_files.h"
#include "board_pins.h"
#include "clutch.h"
#include "motor.h"
#include "filament_sensor.h"
#include "current_sense.h"
#include "bus485_raw.h"
#include "BambuBus.h"
#include "AMCU.h"
#include "settings_storage.h"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_ota_ops.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <stdarg.h>

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define WEB_SERVER_PORT 80
#define AP_SSID_PREFIX "ESP-AMS-"
#define AP_PASSWORD "12345678"
#define WIFI_TIMEOUT_MS 25000

/* ==========================================================================
 * 全局
 * ========================================================================== */
static WebServer server(WEB_SERVER_PORT);
static Preferences s_wifi_prefs;

static bool s_wifi_connected = false;
static bool s_ap_mode = false;
static int s_net_mode = 2;  // 0=STA, 1=AP, 2=离线
static uint32_t s_boot_time = 0;

/* ==========================================================================
 * 日志环形缓冲（行式：48 行 × 每行 160 字符，带单调递增 seq）
 *
 * ★ 2026-10-25：旧实现是"单字符环"，且 web_log() 没有任何调用方 ——
 *   固件日志全走 log_i/log_e（只进串口），网页"设备日志"面板永远空的。
 *   现在：
 *     1. esp_log_set_vprintf() 挂 hook，把所有 log_i/log_w/log_e 输出
 *        截一份进本缓冲（串口输出不变）；
 *     2. 缓冲按"行"组织并带 seq，/log 返回 {"log":[...],"seq":[...]}，
 *        与前端 renderLog 期望的格式对齐（旧格式是单串，前端拿到会
 *        lines.join TypeError，面板静默死亡）。
 *   写入口可能来自多个任务（主循环 / 电流采样定时器 / 离合任务），
 *   用自旋锁保护；临界区只有一次 strncpy，不会死等。
 * ========================================================================== */
#define LOG_LINE_CAP   48
#define LOG_LINE_LEN   160

static char     s_log_lines[LOG_LINE_CAP][LOG_LINE_LEN];
static uint32_t s_log_seqs[LOG_LINE_CAP];
static uint16_t s_log_rep[LOG_LINE_CAP];   /* 该行被连续重复的次数（0=只出现过1次） */
static int      s_log_count = 0;    /* 已写入行数（未满时即行数） */
static int      s_log_next  = 0;    /* 下一个写入位置（环形） */
static uint32_t s_log_seq   = 0;    /* 行号计数器 */
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;

void web_log(const char *msg)
{
    if (!msg || !msg[0]) return;

    portENTER_CRITICAL(&s_log_mux);

    /* 与上一条完全相同：合并成「… ×N」，不占新行、不推进 seq。
       现场要求：同样的消息没变化时不更新 —— 前端按 seq 增量渲染，
       这里不推进 seq，面板就不会被重复消息刷屏；次数记在行尾，
       刷新页面时可见（handle_log 负责拼接 ×N）。 */
    if (s_log_count > 0) {
        int last = (s_log_next - 1 + LOG_LINE_CAP) % LOG_LINE_CAP;
        if (strncmp(msg, s_log_lines[last], LOG_LINE_LEN) == 0) {
            if (s_log_rep[last] < 999) s_log_rep[last]++;
            portEXIT_CRITICAL(&s_log_mux);
            return;
        }
    }

    /* 截断时回退到完整 UTF-8 字符边界（v1.1.8）：
       中文日志常在 LOG_LINE_LEN-1 字节处被切断，留下半个汉字
       （如 0xE6 后面没有 continuation 字节）→ /log 返回的 JSON 里
       出现非法 UTF-8 → 前端 JSON.parse 直接抛错，日志面板空白。
       回退规则：截断点若落在 continuation 字节(10xxxxxx)上，
       一路退到该字符的 lead 字节，整个字符丢弃。 */
    size_t n = strlen(msg);
    if (n > (size_t)(LOG_LINE_LEN - 1)) {
        n = LOG_LINE_LEN - 1;
        while (n > 0 && ((unsigned char)msg[n] & 0xC0) == 0x80) n--;
    }
    memcpy(s_log_lines[s_log_next], msg, n);
    s_log_lines[s_log_next][n] = '\0';

    s_log_rep[s_log_next] = 0;
    s_log_seqs[s_log_next] = ++s_log_seq;
    s_log_next = (s_log_next + 1) % LOG_LINE_CAP;
    if (s_log_count < LOG_LINE_CAP) s_log_count++;

    portEXIT_CRITICAL(&s_log_mux);
}

/* 供 ams_log.h 的 log_* 劫持宏调用：格式化后进网页日志环 */
void web_logf(const char *fmt, ...)
{
    if (!fmt) return;
    char line[LOG_LINE_LEN + 16];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    web_log(line);
}

/* esp_log hook：截获 log_i/log_w/log_e → 喂 web_log；串口输出保持默认 */
static int web_log_vprintf(const char *format, va_list args)
{
    char line[LOG_LINE_LEN + 16];
    va_list a1, a2;
    va_copy(a1, args);
    int n = vsnprintf(line, sizeof(line), format, a1);
    va_end(a1);

    if (n > 0) {
        /* log 宏自带结尾 '\n'；按行拆开分别入库 */
        char *p = line;
        for (;;) {
            char *nl = strchr(p, '\n');
            if (nl) *nl = '\0';
            if (*p) web_log(p);
            if (!nl) break;
            p = nl + 1;
        }
    }

    va_copy(a2, args);
    int m = vprintf(format, a2);   /* 串口照旧 */
    va_end(a2);
    return (m >= 0) ? m : n;
}

/* 尽早安装 hook（main.cpp setup 最开头调），别漏掉启动日志 */
void web_log_install_hook(void)
{
    static bool installed = false;
    if (installed) return;
    installed = true;
    esp_log_set_vprintf(web_log_vprintf);
}

/* App 版本 */
static const char *APP_VERSION = "1.1.18-s3ams";

/* 单 IO 双微动：每路一个 ADC 脚（开始位/停止位共用同一脚，电阻编码） */
static const int BOARD_SENSOR_PINS[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_ARRAY;

/* ==========================================================================
 * 简易 JSON 取值（前端 body 都是 {"k":v,...} 小对象，没必要上 ArduinoJson）
 *   json_get_int/double：找到 "key" 后的冒号，解析其后的数字
 *   多字段 body 也安全（不会像 indexOf("}") 那样把后一个字段吃进来）
 * ========================================================================== */
static bool json_get_int(const String &body, const char *key, long *out)
{
    String k = String("\"") + key + "\"";
    int idx = body.indexOf(k);
    if (idx < 0) return false;
    idx = body.indexOf(':', idx + k.length());
    if (idx < 0) return false;
    idx++;
    while (idx < (int)body.length() &&
           (body[idx] == ' ' || body[idx] == '\t' || body[idx] == '\n' || body[idx] == '\r')) {
        idx++;
    }
    bool neg = false;
    if (idx < (int)body.length() && body[idx] == '-') { neg = true; idx++; }
    long v = 0;
    bool any = false;
    while (idx < (int)body.length() && isDigit((unsigned char)body[idx])) {
        v = v * 10 + (body[idx] - '0');
        idx++;
        any = true;
    }
    if (!any) return false;
    *out = neg ? -v : v;
    return true;
}

static bool json_get_double(const String &body, const char *key, double *out)
{
    String k = String("\"") + key + "\"";
    int idx = body.indexOf(k);
    if (idx < 0) return false;
    idx = body.indexOf(':', idx + k.length());
    if (idx < 0) return false;
    idx++;
    while (idx < (int)body.length() &&
           (body[idx] == ' ' || body[idx] == '\t' || body[idx] == '\n' || body[idx] == '\r')) {
        idx++;
    }
    int start = idx;
    if (idx < (int)body.length() && body[idx] == '-') idx++;
    while (idx < (int)body.length() &&
           (isDigit((unsigned char)body[idx]) || body[idx] == '.')) {
        idx++;
    }
    if (idx == start || (idx == start + 1 && body[start] == '-')) return false;
    *out = body.substring(start, idx).toDouble();
    return true;
}

/* ==========================================================================
 * WiFi
 * ========================================================================== */
/* ==========================================================================
 * WiFi 配网（对齐 esp-ams-s3 的「先关热点、射频让给 STA」模型）
 *
 * 关键：ESP32 只有一个射频，AP 和 STA 共用。配网时若 SoftAP 还开着，
 * STA 连接永远停在 WIFI_REASON_NO_AP_FOUND，密码对错都一样（esp-ams-s3
 * 2026-09-17 实测结论）。所以流程必须是：
 *   1. 先停 SoftAP（把射频让给 STA）
 *   2. 关 STA 省电 esp_wifi_set_ps(WIFI_PS_NONE)
 *   3. WiFi.begin(ssid, pass) 连路由器
 *   4. 成功 → 保持 AP 关闭；失败 → 重新打开 AP（否则手机连不回来）
 * ========================================================================== */
static void wifi_stop_ap(void)
{
    /* ★ 对标 esp-ams-s3 stop_sta_reconnect：真把射频切走让给 STA，
       不能只 softAPdisconnect —— 单射频下 AP 还占着 STA 就 NO_AP_FOUND */
    if (s_ap_mode) {
        WiFi.softAPdisconnect();
        delay(200);
        WiFi.mode(WIFI_OFF);   /* 让射频完全下线，再切 STA */
        delay(200);
        s_ap_mode = false;
    }
}

static void wifi_start_ap(void)
{
    String ap_ssid = String(AP_SSID_PREFIX) + String((uint32_t)(ESP.getEfuseMac() & 0xFFFF), HEX);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid.c_str(), AP_PASSWORD);
    s_ap_mode = true;
    s_wifi_connected = false;
    s_net_mode = 1;
    log_i("配置热点已启动: %s / %s", ap_ssid.c_str(), AP_PASSWORD);
}

static bool wifi_try_connect(const char *ssid, const char *pass, bool provision)
{
    if (ssid == NULL || strlen(ssid) == 0) {
        log_w("wifi_try_connect: SSID 为空，跳过");
        return false;
    }

    if (provision) {
        wifi_stop_ap();
        delay(300);
    }

    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect();
    delay(200);
    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        s_wifi_connected = true;
        s_ap_mode = false;
        s_net_mode = 0;
        log_i("WiFi 已连接: %s / IP %s (PS=NONE, AP 已关)",
              ssid, WiFi.localIP().toString().c_str());
        return true;
    }

    log_w("WiFi 连接失败（SSID=%s, status=%d）", ssid, (int)WiFi.status());
    if (provision) {
        log_i("配网失败，重新打开配置热点");
        delay(300);
        wifi_start_ap();
    }
    return false;
}

static void wifi_init(void)
{
    s_wifi_prefs.begin("s3ams-wifi", true);
    String ssid = s_wifi_prefs.getString("ssid", "");
    String pass = s_wifi_prefs.getString("pass", "");
    s_wifi_prefs.end();

    if (ssid.length() > 0) {
        log_i("尝试连接已保存 WiFi: %s", ssid.c_str());
        if (wifi_try_connect(ssid.c_str(), pass.c_str(), false)) {
            log_i("WiFi 已连接，IP: %s", WiFi.localIP().toString().c_str());
            return;
        }
        log_w("WiFi 连接失败，启动配置热点");
    } else {
        log_i("未配置 WiFi，启动配置热点");
    }
    wifi_start_ap();
}

/* ==========================================================================
 * HTTP 处理
 * ========================================================================== */
static void handle_root(void)
{
    server.send_P(200, "text/html; charset=utf-8", WEB_INDEX_HTML);
}

static void handle_style(void)
{
    server.send_P(200, "text/css; charset=utf-8", WEB_STYLE_CSS);
}

static void handle_app_js(void)
{
    server.send_P(200, "application/javascript; charset=utf-8", WEB_APP_JS);
}

static void handle_status(void)
{
    int cur = get_now_filament_num();
    if (cur < 0 || cur >= 16) cur = -1;

    String json = "{";
    /* ===== 网络 ===== */
    json += "\"wifi_isconnected\":" + String(s_wifi_connected ? "true" : "false") + ",";
    json += "\"is_mqtt_con\":false,";  // 本设计用 485 直连，无 MQTT
    json += "\"ap_on\":" + String(s_ap_mode ? "true" : "false") + ",";
    json += "\"wifi_ssid\":\"" + (s_wifi_connected ? WiFi.SSID() : "") + "\",";
    json += "\"ip\":\"" + (s_wifi_connected ? WiFi.localIP().toString() : "") + "\",";
    json += "\"ap_ip\":\"" + (s_ap_mode ? WiFi.softAPIP().toString() : "") + "\",";
    json += "\"wifi_status_text\":\"485 直连打印机（无 MQTT)\",";
    json += "\"wifi_status\":" + String(s_wifi_connected ? "3" : "0") + ",";
    json += "\"rssi\":" + String(s_wifi_connected ? WiFi.RSSI() : 0) + ",";
    json += "\"mac\":\"" + WiFi.macAddress() + "\",";
    json += "\"mqtt_rx\":0,";
    json += "\"mqtt_configured\":false,";
    json += "\"mqtt_last_error\":\"\",";
    json += "\"ap_ssid\":\"" + (s_ap_mode ? String(AP_SSID_PREFIX) : "") + "\",";
    json += "\"ap_clients\":" + String(s_ap_mode ? WiFi.softAPgetStationNum() : 0) + ",";
    json += "\"ssids\":[],";
    json += "\"extruder_pin_used\":false,";
    json += "\"app_version\":\"" + String(APP_VERSION) + "\",";
    json += "\"part\":\"esp32s3\",";
    json += "\"build_time\":\"" + String(__TIME__) + "\",";
    json += "\"build_date\":\"" + String(__DATE__) + "\",";
    json += "\"comm_mode\":2,";  // 485 直连（自动）
    json += "\"comm_mode_text\":\"RS-485 直连打印机（BambuBus）\",";
    /* 上报给打印机的 AMS 版本号（v1.1.13）。
       打印机拿它跟白名单比对，对不上就弹「仅支持 AMS / AMS 2 Pro / AMS HT」。 */
    {
        uint32_t av = Bambubus_get_ams_fw_version();
        char avbuf[24];
        snprintf(avbuf, sizeof(avbuf), "%02u.%02u.%02u.%02u",
                 (unsigned)((av >> 24) & 0xFF), (unsigned)((av >> 16) & 0xFF),
                 (unsigned)((av >> 8) & 0xFF), (unsigned)(av & 0xFF));
        json += "\"ams_ver\":\"" + String(avbuf) + "\",";
        json += "\"ams_ver_raw\":" + String((unsigned long)av) + ",";
    }
    json += "\"wifi_on\":1,";  // WiFi 常开（本固件无 MQTT 模式）
    json += "\"act485\":" + String(ams_enable ? 1 : 0) + ",";  // 485 运动执行开关回显
    json += "\"jog_ms\":" + String(jog_time_ms) + ",";
    json += "\"jog_speed_pct\":" + String(jog_speed_pct) + ",";
    json += "\"pullback_sec\":" + String(settings_get_pullback_timeout_common()) + ",";
    json += "\"current_access\":" + String((cur >= 0) ? (cur + 1) : 0) + ",";
    json += "\"mem_free\":" + String(esp_get_free_heap_size()) + ",";
    /* 主循环任务栈的历史最小剩余字节（v1.1.7 加）：
       8KB 栈时 /log 请求会把栈用穿（Stack canary panic），
       现在栈加到 32KB，这个值能直接看出余量够不够 ——
       稳定运行后应 >20KB；若掉到 2KB 以下说明还要加栈。 */
    json += "\"stack_free\":" + String((unsigned)uxTaskGetStackHighWaterMark(NULL)) + ",";
    json += "\"mem_min_free\":0,";
    json += "\"uptime\":" + String((long)millis()) + ",";

    /* ===== 板型 ===== */
    json += "\"board\":{\"name\":\"ESP32-S3 AMS (8通道双微动)\"},";

    /* ===== AMS 状态 ===== */
    json += "\"ams\":{";
    json += "\"busy\":" + String(clutch_is_busy() ? "true" : "false") + ",";
    json += "\"state_text\":\"" + String(clutch_get_owner()) + "\",";
    json += "\"active_material\":" + String(clutch_active_channel()) + ",";
    json += "\"diag\":{\"conflicts\":" + String(clutch_conflict_count()) +
            ",\"target_switch\":" + String((unsigned long)amcu_clutch_switch_count()) +
            ",\"target_abort\":" + String((unsigned long)amcu_clutch_abort_count()) +
            ",\"exchange_ok\":0,\"exchange_fail\":0,\"last_error\":\"—\",\"last_error_ms\":-1}";
    json += "},";

    /* ===== 送料链诊断（v1.1.9 加）=====
       一次就能看出"该送料但电机不转"卡在哪一环：
         motion=1(need_send_out) 才该驱动；
         online=0 → 该料位被判离线，AMCU 会 drive_stop；
         printing=1 → 判成"打印中"，AMCU_motion 直接让出、不驱动；
         tripped=1 → 堵转 latch，状态机被禁止驱动（用 /current_clear 清）；
         jog=1 → 点动没结束，状态机让出；
         clutch_mask=0 而离合实际在响 → 引脚回读有问题（clutch_rb=0）。 */
    {
        int dn = get_now_filament_num();
        if (dn < 0 || dn >= 16) dn = 0;
        _filament_motion_state_set dm = get_filament_motion(dn);
        const char *dtxt = "waiting";
        switch (dm) {
        case need_pull_back: dtxt = "need_pull_back"; break;
        case need_send_out:  dtxt = "need_send_out";  break;
        case waiting:        dtxt = "waiting";        break;
        case act_send_mm:    dtxt = "act_send_mm";    break;
        case act_pull_mm:    dtxt = "act_pull_mm";    break;
        case select_pos:     dtxt = "select_pos";     break;
        case release_all:    dtxt = "release_all";    break;
        case cancel:         dtxt = "cancel";         break;
        }
        json += "\"feed\":{";
        json += "\"cur\":" + String(dn) + ",";
        json += "\"motion\":" + String((int)dm) + ",";
        json += "\"motion_text\":\"" + String(dtxt) + "\",";
        json += "\"online\":" + String(is_filament_online(dn) ? 1 : 0) + ",";
        json += "\"printing\":" + String(is_channel_printing(dn) ? 1 : 0) + ",";
        json += "\"tripped\":" + String(current_sense_tripped() ? 1 : 0) + ",";
        json += "\"jog\":" + String(amcu_jog_busy() ? 1 : 0) + ",";
        /* v1.1.16：点动目标与剩余时间。WEB 硬件调试"进料不吸合"时，
           对一下 jog_ch 和 clutch_mask 就知道是不是离合被别处抢走了：
           点通道 3 进料 → jog_ch 应为 3，clutch_mask 应为 0b100(bit2)。 */
        {
            int jch = 0, jdir = 0, jms = 0;
            amcu_jog_info(&jch, &jdir, &jms);
            json += "\"jog_ch\":" + String(jch) + ",";
            json += "\"jog_dir\":" + String(jdir) + ",";
            json += "\"jog_ms_left\":" + String(jms) + ",";
        }
        json += "\"clutch_mask\":" + String((unsigned)clutch_engaged_mask()) + ",";
        json += "\"clutch_rb\":" + String(clutch_readback_ok() ? 1 : 0) + ",";
        json += "\"motor_dir\":" + String((int)motor_get_dir()) + ",";
        json += "\"motor_pct\":" + String(motor_get_speed_pct()) + ",";
        json += "\"cmd\":" + String(get_cmd_type()) + ",";
        /* 计米（米）：v1.1.10 及更早版本单位错 1000 倍，这里暴露出来方便核对。
           送料 1 秒应约 0.05 米（50mm），若看到几十上百米就是又跑飞了。 */
        json += "\"meters\":" + String(get_filament_meters(dn), 4) + ",";
        /* 打印阶段细分（v1.1.12）：get_filament_motion() 把 before/on/stop_on_use
           全映射成 waiting，分不出当前在哪一步，这里单独给出来。
           phase=before_on_use 时 follow=1 表示"跟随送料"正在送料相位。 */
        const char *phase = "none";
        if (is_channel_before_on_use(dn))        phase = "before_on_use";
        else if (is_channel_on_use(dn))          phase = "on_use";
        else if (is_channel_stop_on_use(dn))     phase = "stop_on_use";
        json += "\"phase\":\"" + String(phase) + "\",";
        json += "\"follow\":" + String(amcu_follow_feed_active() ? 1 : 0);
        json += "},";
    }

    /* ===== 复位/诊断 ===== */
    json += "\"reset\":{\"cause\":\"—\",\"cause_desc\":\"—\",\"boot_count\":1,\"uptime_ms\":" + String((long)millis()) + "},";
    json += "\"boot_safety\":{\"ok\":true},";

    /* ===== 硬件概览 ===== */
    json += "\"hardware\":{\"channels\":" + String(BOARD_CHANNEL_COUNT) +
            ",\"engaged\":[" + String(clutch_active_channel()) + "]," +
            "\"limits\":[],\"motor_direction\":" + String((int)motor_get_dir()) +
            ",\"conflicts\":" + String(clutch_conflict_count()) + "},";

    /* ===== 电流采样（AT8236 ISEN / GPIO3）+ 堵转保护 ===== */
    json += "\"current\":{";
    json += "\"board\":" + String(BOARD_CURRENT_SENSE_ENABLED ? "true" : "false") + ",";
    if (current_sense_ready()) {
        json += "\"ready\":true,";
        json += "\"enabled\":" + String(current_sense_enabled() ? "true" : "false") + ",";
        json += "\"mv\":" + String(current_sense_mv()) + ",";
        json += "\"ma\":" + String(current_sense_ma()) + ",";
        json += "\"peak_mv\":" + String(current_sense_peak_mv()) + ",";
        json += "\"peak_ma\":" + String(current_sense_peak_ma()) + ",";
        json += "\"tripped\":" + String(current_sense_tripped() ? "true" : "false") + ",";
        json += "\"trips\":" + String((unsigned long)current_sense_trips()) + ",";
        json += "\"r_mohm\":" + String(current_sense_r_mohm()) + ",";
        /* V = I×R → mV/A 数值上就等于 R(mΩ)（200mΩ = 200mV/A） */
        json += "\"mv_per_a\":" + String(current_sense_r_mohm()) + ",";
        json += "\"stall_ma\":" + String(current_sense_stall_ma()) + ",";
        json += "\"stall_ms\":" + String(current_sense_stall_ms()) + ",";
        json += "\"blank_ms\":" + String(current_sense_blank_ms());
    } else {
        json += "\"ready\":false";
    }
    json += "},";

    /* ===== 传感器（双微动 ADC） ===== */
    json += "\"sensors\":{";
    json += "\"enabled\":" + String(sensor_enabled_mask()) + ",";
    json += "\"dual\":true,";
    /* v1.1.14：微动读取模式。digital = 临时直连（8 只独立微动两两配对），
       encoded = 正式电阻编码（1 个 ADC 脚带 2 只）。 */
    json += "\"mode\":\"" + String(SENSOR_TEMP_DIGITAL ? "digital" : "encoded") + "\",";
    json += "\"extruder\":false,";
    json += "\"ch\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        int stop_t = sensor_triggered(i, SENSOR_STOP) ? 1 : 0;
        int start_t = sensor_triggered(i, SENSOR_START) ? 1 : 0;
        json += "[" + String(stop_t) + "," + String(start_t) + "]";
    }
    json += "],";
    json += "\"pins\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
#if SENSOR_TEMP_DIGITAL
        /* 临时直连：通道 i(0..3) 的开始位 = 引脚 i，停止位 = 引脚 i+4。
           通道 4..7 没有独立引脚（它们的脚被当成停止位了）→ [-1,-1]，
           网页据此显示「无」，不会被误当成接线问题。 */
        if (i < BOARD_CHANNEL_COUNT / 2) {
            json += "[" + String(BOARD_SENSOR_PINS[i + BOARD_CHANNEL_COUNT / 2]) +
                    "," + String(BOARD_SENSOR_PINS[i]) + "]";
        } else {
            json += "[-1,-1]";
        }
#else
        json += "[" + String(BOARD_SENSOR_PINS[i]) + "," + String(BOARD_SENSOR_PINS[i]) + "]";
#endif
    }
    json += "],";
    json += "\"buf\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        json += sensor_channel_enabled(i) ? "1" : "0";
    }
    json += "],";
    json += "\"mv\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        json += String(sensor_combo_mv(i));
    }
    json += "],";
    json += "\"adc\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        json += String(sensor_combo_adc_unit(i));
    }
    json += "],";
    json += "\"deg\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        json += sensor_combo_degraded(i) ? "1" : "0";
    }
    json += "],";
    json += "\"job\":1,";   // 简化：认为有打印任务时微动才驱动
    json += "\"stage\":1,";
    json += "\"cur\":" + String(cur);
    json += "}";

    json += "}";
    server.send(200, "application/json; charset=utf-8", json);
}

static void handle_log(void)
{
    /* 行式缓冲直接导出 {"log":[...], "seq":[...], "mem_free":N}，
       与前端 renderLog 的增量渲染协议对齐（按 seq 只追加新行）。
       读的时候拿锁拷一份，JSON 拼接在锁外做。

       ★★★ v1.1.6 血的教训：这里原来是**栈上**数组
       char lines[48][160] ≈ 7.7KB。Arduino loopTask 栈只有 8KB，
       叠加 WebServer 的 handleClient→_handleRequest 调用链直接
       **栈溢出 panic**。症状极具误导性：
         - /log 一抓设备就重启（返回空/超时）
         - 更毒的是：网页 OTA 上传要几十秒，前端每 2 秒 pollLog 一次，
           上传期间第一次轮询就崩 → OTA 连接断 → "升级失败"
         - v1.1.2~1.1.5 的 OTA 失败、"/log 为空"都有它的份
       handle_log 只在 loopTask 单线程调用，改 static 零风险。 */
    static char     lines[LOG_LINE_CAP][LOG_LINE_LEN];
    static uint32_t seqs[LOG_LINE_CAP];
    static uint16_t reps[LOG_LINE_CAP];
    int count = 0;
    portENTER_CRITICAL(&s_log_mux);
    int start = (s_log_count < LOG_LINE_CAP) ? 0 : s_log_next;
    count = s_log_count;
    for (int i = 0; i < count; i++) {
        int idx = (start + i) % LOG_LINE_CAP;
        strncpy(lines[i], s_log_lines[idx], LOG_LINE_LEN);
        seqs[i] = s_log_seqs[idx];
        reps[i] = s_log_rep[idx];
    }
    portEXIT_CRITICAL(&s_log_mux);

    /* 一次 reserve 到位，避免 48 次 += 反复 realloc 拖慢弱网响应 */
    String json;
    json.reserve(LOG_LINE_CAP * (LOG_LINE_LEN + 24) + 64);
    json = "{\"log\":[";
    for (int i = 0; i < count; i++) {
        if (i) json += ",";
        json += "\"";
        /* 转义 " 与 \；控制字符换成空格（日志里偶有 \r 等，
           直接塞进 JSON 会让前端 JSON.parse 失败） */
        for (const char *p = lines[i]; *p; p++) {
            unsigned char c = (unsigned char)*p;
            if (c < 0x20) { json += ' '; continue; }
            if (c == '"' || c == '\\') json += '\\';
            json += (char)c;
        }
        /* 连续重复的消息合并显示为「… ×N」（rep 是第 1 次之后的重复数） */
        if (reps[i] > 0) json += " \xC3\x97" + String(reps[i] + 1);
        json += "\"";
    }
    json += "],\"seq\":[";
    for (int i = 0; i < count; i++) {
        if (i) json += ",";
        json += String(seqs[i]);
    }
    json += "],\"mem_free\":" + String(esp_get_free_heap_size()) + "}";
    server.send(200, "application/json; charset=utf-8", json);
}

static void handle_sensor_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    // 简单解析 channel 和 enable
    int channel = -1, enable = -1;
    int cidx = body.indexOf("\"channel\"");
    int eidx = body.indexOf("\"enable\"");
    if (cidx >= 0) channel = body.substring(cidx + 9, body.indexOf("}", cidx)).toInt();
    if (eidx >= 0) enable = body.substring(eidx + 8, body.indexOf("}", eidx)).toInt();

    bool ok = false;
    if (channel >= 1 && channel <= BOARD_CHANNEL_COUNT) {
        ok = sensor_set_channel_enabled(channel - 1, enable != 0);
    }
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

static void handle_wifi_scan(void)
{
    /* 关键：AP 模式下 scanNetworks() 直接返回 0。必须临时切到
       WIFI_APSTA / STA 才能扫到 2.4G 网络。扫完保持 STA 模式，
       方便后续配网（配网失败可继续扫，不必重启）。*/
    bool was_ap = s_ap_mode;
    int prev_mode = WiFi.getMode();
    if (prev_mode == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
        delay(300);  /* 等 STA 栈起来 */
    }
    int n = WiFi.scanNetworks(true, true, 0, 36, 120);  /* sort=true, show_hidden=true, 36ch */
    String json = "{\"aps\":[";
    for (int i = 0; i < n; i++) {
        if (i) json += ",";
        json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
                ",\"channel\":" + String(WiFi.channel(i)) +
                ",\"encrypted\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_WPA_PSK || WiFi.encryptionType(i) == WIFI_AUTH_WPA2_PSK ? "true" : "false") + "}";
    }
    json += "],\"count\":" + String(n) + "}";
    server.send(200, "application/json; charset=utf-8", json);
    /* 如果配网成功切到 STA 后，扫描结果已经没用了，恢复 AP 保持配网入口 */
    if (was_ap && !s_wifi_connected) {
        WiFi.mode(WIFI_AP);
    }
}

static void handle_wifi_connect(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false}");
        return;
    }
    String body = server.arg("plain");
    String name = "", password = "";
    /* "name": 共 7 字符（" n a m e " :），值前引号在 nidx+8
       （旧代码 +7 取到的是前引号本身 → name 解析成空 → 误判"连接失败"） */
    int nidx = body.indexOf("\"name\"");
    int pidx = body.indexOf("\"password\"");
    if (nidx >= 0) name = body.substring(nidx + 8, body.indexOf("\"", nidx + 8));
    /* "password": 共 11 字符（" p a s s w o r d " :），值前引号在 pidx+12
       （旧代码 +11 取到的是前引号本身 → password 解析成空） */
    if (pidx >= 0) password = body.substring(pidx + 12, body.indexOf("\"", pidx + 12));

    if (name.length() == 0 || name.length() > 32) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"SSID 长度必须在 1~32\"}");
        return;
    }

    /* 清掉旧的 SSID/密码 与 残留的 CCMP 状态，避免加密状态机串味 */
    s_wifi_prefs.begin("s3ams-wifi", false);
    s_wifi_prefs.remove("ssid");
    s_wifi_prefs.remove("pass");
    s_wifi_prefs.putString("ssid", name);
    s_wifi_prefs.putString("pass", password);
    s_wifi_prefs.end();

    /* ★ 对齐 esp-ams-s3 provision_task 的完整模型：
       1. 先 server.send 把 HTTP 响应发到手机 —— 射频一关手机立刻掉线，
          响应没发完用户就永远看不到结果（"点连接没反应"的真正来源）
       2. 优雅期：等响应真正写出 TCP 缓冲（esp-ams-s3 是 700ms，留足 1s）
       3. 再动射频（关 AP / 切 STA / 连路由器）—— 这时手机已拿到"配置已保存"，
          掉线也不会误以为"连接失败"
       4. 成功 → 保持 AP 关；失败 → wifi_try_connect 自动把 AP 开回来 */
    log_i("配网: 收到 %s，先回响应再切射频", name.c_str());
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"配置已保存，正在连接\"}");
    delay(1000);   /* 优雅期：让 HTTP 响应发出 */

    bool ok = wifi_try_connect(name.c_str(), password.c_str(), true);
    if (ok) {
        log_i("配网成功: 已连 %s / IP %s，稍后重启固化 STA 模式",
              name.c_str(), WiFi.localIP().toString().c_str());
        delay(500);
        ESP.restart();
    }
    /* 失败：wifi_try_connect 已把热点开回来，手机重连热点再试 */
}

static void handle_wifi_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false}");
        return;
    }
    String body = server.arg("plain");
    bool on = body.indexOf("\"on\":true") >= 0 || body.indexOf("\"on\":1") >= 0;
    if (!on) {
        WiFi.disconnect();
        s_wifi_connected = false;
        wifi_start_ap();
    }
    server.send(200, "application/json", "{\"ok\":true}");
}

/* ==========================================================================
 * OTA 网页升级（multipart 流式上传）
 *
 * ★ 为什么必须 multipart/form-data：
 *   如果浏览器用 application/octet-stream 裸 POST，arduino-esp32 的
 *   WebServer 会把**整个 body 一次性读进 RAM**（readBytesWithTimeout
 *   反复 realloc）。本板不开 PSRAM，堆只有几百 KB，1MB+ 的固件读到一半
 *   realloc 失败 → _parseRequest 返回 false → 连接被掐断 → 浏览器
 *   xhr.onerror "上传中断"（进度条还在走是因为发送端还没被通知）。
 *   multipart 走 _parseForm 流式路径：每 1436 字节触发一次
 *   server.upload() 回调，我们边收边写 flash，内存占用恒定。
 *   前端用 FormData 即可（勿手动设 Content-Type，边界要浏览器生成）。
 *
 * ★ 为什么必须用 4 参数 on() 注册（2026-10-25 血的教训）：
 *   server.on(uri, HTTP_POST, fn) 这 3 参数重载把上传回调注册成
 *   _fileUploadHandler，而它是 nullptr → FunctionRequestHandler::
 *   canUpload() 返回 false → _parseForm 里 START/WRITE/END 三个
 *   upload 事件**一个都不会送来**，1.18MB 被逐字节解析后全部丢弃，
 *   最后 POST 完成回调里 Update.end() 发现从未 begin → 返回 500
 *   "上传失败"。上传回调必须作为 on() 的第 4 个参数（或 onFileUpload）。
 *
 * ★ 为什么用 UPDATE_SIZE_UNKNOWN：
 *   multipart 的 Content-Length 含 boundary 头尾（比镜像大几百字节），
 *   拿它当 Update.begin 的 size 会让 _size 大于实际镜像；虽然
 *   end(true) 能兜住，但没必要赌，UNKNOWN 走"边写边擦"的成熟路径。
 * ========================================================================== */
static bool     s_ota_error = false;
static size_t   s_ota_written = 0;
static uint32_t s_ota_start_ms = 0;

/* --------------------------------------------------------------------------
 * OTA 期间的任务看门狗（TWDT）联动
 *
 * ★ v1.1.4 及以前「上传中断」的真正原因（2026-10-26 实锤）：
 *   main.cpp 用 esp_task_wdt_add(NULL) 把主循环任务订阅进了 10s 看门狗。
 *   而 multipart 上传时，server.handleClient() 会在**一次调用里**把整个
 *   1.8MB 收完（边收边写 flash，几十秒不返回）—— 主循环卡在里面，
 *   喂不了狗 → 第 10 秒 TWDT panic 重启 → 浏览器连接被reset →
 *   xhr.onerror「上传中断」。证据：OTA 测试期间设备 uptime 从 124s
 *   掉到 9s（中途重启了）。
 *
 *   对策：上传期间把主循环任务临时退出看门狗（START 时 delete），
 *   结束/中断时再加回来（END/ABORTED）；另外每写完一个数据块兜底
 *   喂一次狗（万一 delete 失败也死不了）；最后在 web_server_handle
 *   里加保险丝：20s 没有上传进度就恢复订阅，防客户端半途断开后
 *   系统长期裸奔。
 * ------------------------------------------------------------------------ */
static bool     s_ota_wdt_off = false;      /* true=主循环已退出 TWDT，需恢复 */
static uint32_t s_ota_last_prog_ms = 0;     /* 最近一次上传进度时刻 */

static void ota_wdt_off_enter(void)
{
    if (s_ota_wdt_off) return;
    s_ota_last_prog_ms = millis();
    esp_err_t err = esp_task_wdt_delete(NULL);   /* NULL = 当前任务 = 主循环 */
    if (err == ESP_OK) {
        s_ota_wdt_off = true;
        log_i("OTA: 上传期间主循环临时退出任务看门狗（%ums 起）",
              (unsigned)millis());
    } else {
        /* 没能退订（理论上只有本来就未订阅才会失败）→ 退化为每块喂狗 */
        log_w("OTA: 退出看门狗失败(%s)，改为每数据块喂狗", esp_err_to_name(err));
    }
}

static void ota_wdt_on_restore(void)
{
    if (!s_ota_wdt_off) return;
    s_ota_wdt_off = false;
    esp_task_wdt_add(NULL);   /* 重新订阅主循环任务 */
    log_i("OTA: 已恢复任务看门狗订阅");
}

static void handle_ota_upload(void)
{
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        s_ota_error = false;
        s_ota_written = 0;
        s_ota_start_ms = millis();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            Update.printError(Serial);
            log_e("OTA begin 失败");
            s_ota_error = true;
            return;
        }
        ota_wdt_off_enter();
        log_i("OTA 开始: %s（流式 multipart，UNKNOWN 尺寸）", upload.filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (s_ota_error) return;  /* begin 就失败了，丢弃后续数据 */
        size_t n = Update.write(upload.buf, upload.currentSize);
        if (n != upload.currentSize) {
            Update.printError(Serial);
            log_e("OTA 写入失败: 期望 %u 实写 %u（累计 %u）",
                  (unsigned)upload.currentSize, (unsigned)n, (unsigned)s_ota_written);
            Update.abort();
            s_ota_error = true;
            return;
        }
        s_ota_written += n;
        s_ota_last_prog_ms = millis();
        esp_task_wdt_reset();   /* 兜底喂狗（退订失败时靠这个续命） */
    } else if (upload.status == UPLOAD_FILE_END) {
        if (s_ota_error) {
            ota_wdt_on_restore();
            server.send(500, "application/json",
                        "{\"ok\":false,\"info\":\"OTA 失败（写入 flash 出错，见设备日志）\"}");
            return;
        }
        if (Update.end(true)) {
            log_i("OTA 完成: %u 字节，用时 %ums，重启中…",
                  (unsigned)s_ota_written,
                  (unsigned)(millis() - s_ota_start_ms));
            ota_wdt_on_restore();
            /* 前端 xhr.onload 解析 JSON 判断成败，别发纯文本 */
            server.send(200, "application/json",
                        "{\"ok\":true,\"info\":\"OTA 成功，设备将重启\"}");
            /* 主动把 TCP 发送缓冲推出去，再等一会儿才重启。
               只靠 delay() 不够：ESP.restart() 会直接把 socket 掐断，
               弱信号下浏览器连响应头都没读完就收到 RST →
               XHR onerror → 页面报"上传中断/升级失败"，但固件其实写进去了
               （2026-10-25 现场：报失败，刷新后版本号已经是新的）。
               前端另有"版本号比对"兜底，这里尽量减少误报。 */
            server.client().flush();
            delay(1500);
            ESP.restart();
        } else {
            Update.printError(Serial);
            log_e("OTA end 失败（累计写入 %u）", (unsigned)s_ota_written);
            ota_wdt_on_restore();
            server.send(500, "application/json",
                        "{\"ok\":false,\"info\":\"OTA 失败（Update.end 校验未过）\"}");
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        /* 客户端半途断开：library 走这条，END 不会来，必须在这里收尾，
           否则看门狗一直退订着 + flash 半边新固件没人管 */
        Update.abort();
        log_w("OTA: 客户端中断上传（已写入 %u 字节，已丢弃）",
              (unsigned)s_ota_written);
        ota_wdt_on_restore();
    }
}

/* POST 完成回调：响应已在 upload 回调的 END 分支发过，这里必须空手，
   再 send 一次会 "double response" 把连接搞坏 */
static void handle_ota_upload_done(void)
{
    /* 保留给将来要做"上传完成后统一回执"的场景；现在什么都不做 */
}

/* ==========================================================================
 * 485 原始报文（/bus485raw?since=seq）
 *   一行一帧：<seq> <t_ms> <RX|TX> <len> <hex>
 *   RX=打印机发来（前端 Cyan），TX=本机回复（前端 Green）
 *   since 只回新帧，PC 监控脚本轮询用；不带 since 回整个环形缓冲
 * ========================================================================== */
static void handle_bus485raw(void)
{
    uint32_t since = 0;
    if (server.hasArg("since")) {
        since = (uint32_t)strtoul(server.arg("since").c_str(), NULL, 10);
    }

    /* 首行是汇总，方便脚本/人一眼看到计数 */
    String out = "RX_TOTAL=" + String(bus485raw_rx_count()) +
                 " TX_TOTAL=" + String(bus485raw_tx_count()) +
                 " LAST_SEQ=" + String(bus485raw_last_seq()) + "\n";

    /* hex 展开后最大 ~32 帧 × 560 字节 ≈ 18KB，用堆上 String 不占静态 */
    char *buf = (char *)malloc(BUS485_RAW_CAP * 576 + 64);
    if (buf) {
        uint32_t first = 0;
        int n = bus485raw_dump(buf, BUS485_RAW_CAP * 576 + 64, since, &first);
        if (n > 0) out += String(buf);
        free(buf);
    }
    server.send(200, "text/plain; charset=utf-8", out);
}

static void handle_not_found(void)
{
    server.send(404, "text/plain", "Not Found");
}

/* 通用：返回成功（前端部分可选动作的后端占位，避免 404 噪声） */
static void handle_ok(void)
{
    server.send(200, "application/json", "{\"ok\":true}");
}

/* 复位离合冲突计数 */
static void handle_diag_reset(void)
{
    clutch_reset_conflict_count();
    amcu_clutch_reset_diag();
    server.send(200, "application/json", "{\"ok\":true,\"info\":\"已清除离合冲突/仲裁计数\"}");
}

/* 清零全部料位的计米。
 *
 * 为什么需要：v1.1.10 及更早版本 accumulate_feed_meters() 把「毫秒」当「秒」
 * 用，计米放大 1000 倍 —— 送料 3 秒就报 141 米、第二次 818 米。这些值已经
 * 写进 flash，而打印机按 meters 折算料盘余量，会把料当成「已用光」。
 * 刷入 v1.1.11 后调一次本接口即可恢复。 */
static void handle_meters_reset(void)
{
    for (int i = 0; i < 4 * MAX_AMS_NUM; i++) {
        reset_filament_meters(i);
    }
    Bambubus_set_need_to_save();
    log_i("计米已清零: 16 个料位 meters 全部归零");
    server.send(200, "application/json",
                "{\"ok\":true,\"info\":\"16 个料位计米已清零\"}");
}

/* 485 运动执行开关（Web 驱动 ams_enable）
   前端 post('/act485_set', {on:0|1})，on=1 打开、on=0 关闭。
   之前走 handle_ok 占位 → 根本没碰 ams_enable，勾选不生效。 */
static void handle_act485_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    int oidx = body.indexOf("\"on\"");
    int on = -1;
    if (oidx >= 0) on = body.substring(oidx + 4, body.indexOf("}", oidx)).toInt();

    amcu_set_ams_enable(on == 1);
    String resp = on == 1
        ? "{\"ok\":true,\"info\":\"485 运动执行已打开（打印机点进料/退料会驱动本机电机）\"}"
        : "{\"ok\":true,\"info\":\"485 运动执行已关闭（仅走协议，不驱动电机）\"}";
    server.send(200, "application/json", resp);
}

/* ==========================================================================
 * 电机电流采样 / 堵转保护（AT8236 ISEN · GPIO3）
 * ========================================================================== */

/* /current_set：{r_mohm, stall_ma, stall_ms, blank_ms} 全部可选；
   或 {reset:1} 恢复默认。返回夹取后实际生效的值。 */
static void handle_current_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");

    if (body.indexOf("\"reset\"") >= 0) {
        current_sense_reset_defaults();
    } else {
        int r   = current_sense_r_mohm();
        int sma = current_sense_stall_ma();
        int sms = current_sense_stall_ms();
        int bms = current_sense_blank_ms();
        bool changed = false;
        long v;
        double d;
        if (json_get_int(body, "r_mohm", &v))   { r = (int)v;   changed = true; }
        if (json_get_int(body, "stall_ma", &v)) { sma = (int)v; changed = true; }
        if (json_get_int(body, "stall_ms", &v)) { sms = (int)v; changed = true; }
        if (json_get_int(body, "blank_ms", &v)) { bms = (int)v; changed = true; }
        if (json_get_double(body, "stall_ma", &d)) { sma = (int)d; changed = true; }
        if (changed) {
            current_sense_set_config(r, sma, sms, bms);
        } else {
            server.send(200, "application/json", "{\"ok\":true,\"info\":\"没有要保存的项\"}");
            return;
        }
    }

    const char *info = current_sense_stall_ma() > 0
        ? "已保存并生效（堵转保护开启）"
        : "已保存并生效（阈值 0 = 只显示、不保护）";
    String resp = "{\"ok\":true,\"info\":\"" + String(info) + "\"" +
        ",\"r_mohm\":" + String(current_sense_r_mohm()) +
        ",\"stall_ma\":" + String(current_sense_stall_ma()) +
        ",\"stall_ms\":" + String(current_sense_stall_ms()) +
        ",\"blank_ms\":" + String(current_sense_blank_ms()) + "}";
    server.send(200, "application/json", resp);
}

/* /current_clear：清除堵转 latch + 峰值窗口 */
static void handle_current_clear(void)
{
    current_sense_clear();
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"已清除堵转 latch 与峰值（电机可再次驱动）\"}");
}

/* /current_channel_set：本机单电机共享，电流采样不区分通道 —— 收下但不假装生效 */
static void handle_current_channel_set(void)
{
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"本机单个共享电机，电流采样不区分通道\"}");
}

/* ==========================================================================
 * 手动点动（/jog_set 存参数，/hardware_test 执行一次）
 * ========================================================================== */

/* /jog_set：{seconds, speed_pct} 各自可选，没传的保持原值 */
static void handle_jog_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    int ms = jog_time_ms;
    int pct = jog_speed_pct;
    bool changed = false;
    long v;
    double d;
    if (json_get_double(body, "seconds", &d)) { ms = (int)(d * 1000.0); changed = true; }
    if (json_get_int(body, "speed_pct", &v))  { pct = (int)v; changed = true; }

    if (changed) jog_config_set(ms, pct);

    String resp = String("{\"ok\":true,\"info\":\"点动参数已保存\"") +
        ",\"jog_ms\":" + String(jog_time_ms) +
        ",\"jog_speed_pct\":" + String(jog_speed_pct) + "}";
    server.send(200, "application/json", resp);
}

/* /hardware_test：{channel, direction} —— 按点动参数驱动一次（非阻塞，到时自停） */
static void handle_hardware_test(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long ch = 0, dir = 0;
    json_get_int(body, "channel", &ch);
    json_get_int(body, "direction", &dir);

    if (ch < 1 || ch > BOARD_CHANNEL_COUNT || (dir != 1 && dir != -1)) {
        server.send(400, "application/json",
            "{\"ok\":false,\"info\":\"参数错误：channel 1~8，direction ±1\"}");
        return;
    }
    if (current_sense_tripped()) {
        server.send(200, "application/json",
            "{\"ok\":false,\"info\":\"堵转保护已动作（latch），请先「清除故障/峰值」\"}");
        return;
    }
    if (amcu_jog_busy() || clutch_is_busy()) {
        server.send(200, "application/json",
            "{\"ok\":false,\"info\":\"上一次动作尚未结束，请稍候\"}");
        return;
    }

    bool ok = amcu_jog_start((int)ch, (int)dir, jog_time_ms, jog_speed_pct);
    if (!ok) {
        server.send(200, "application/json", "{\"ok\":false,\"info\":\"点动启动失败\"}");
        return;
    }
    String resp = "{\"ok\":true,\"info\":\"通道" + String((int)ch) +
        (dir == 1 ? " 进料" : " 退料") + " " + String(jog_time_ms) +
        "ms @ " + String(jog_speed_pct) + "%（duty " +
        String((unsigned)motor_pct_to_duty(jog_speed_pct)) + "/255）\"}";
    server.send(200, "application/json", resp);
}

/* /comm_set：本固件固定 RS-485 直连打印机（BambuBus），没有可切的通信模式 */
static void handle_comm_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long mode = 2;
    json_get_int(body, "mode", &mode);
    log_i("comm_set: 请求模式 %ld（本固件固定 RS-485 直连，无其他模式）", mode);
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"本固件固定 RS-485 直连打印机（BambuBus），无需切换通信模式\"}");
}

/* /ams_ver_set：改上报给打印机的 AMS 版本号（v1.1.13）
 *
 * 前端 post('/ams_ver_set', {raw: 16778839})，raw 是 4 字节版本号按小端拼成的
 * uint32（0x01000657 = 01.00.06.87 = 16778839）。
 *
 * 为什么做成可改：不同打印机固件的白名单不一样，换个打印机可能要换版本号。
 * 改完**立即生效**（下一次 0x103 查询就用新值），不用重启。
 */
static void handle_ams_ver_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long raw = 0;
    if (!json_get_int(body, "raw", &raw)) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"缺少 raw\"}");
        return;
    }
    Bambubus_set_ams_fw_version((uint32_t)raw);

    uint32_t v = Bambubus_get_ams_fw_version();
    char msg[96];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"ver\":\"%02u.%02u.%02u.%02u\",\"info\":\"已保存，下一次版本查询即生效\"}",
             (unsigned)((v >> 24) & 0xFF), (unsigned)((v >> 16) & 0xFF),
             (unsigned)((v >> 8) & 0xFF), (unsigned)(v & 0xFF));
    server.send(200, "application/json", msg);
}

/* /pullback_set：改退料（自动换料退料）超时时间（v1.1.18）
 *
 * 前端 post('/pullback_set', {sec: 4})，单位秒，范围 3~30。
 * 本硬件是单电机 + 8 离合，全局一个值，这里统一写 16 路并持久化。
 * 改完立即生效（下一次退料就用新值），不用重启。 */
static void handle_pullback_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long sec = 0;
    if (!json_get_int(body, "sec", &sec)) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"缺少 sec\"}");
        return;
    }
    int saved = settings_set_pullback_timeout_all((int)sec);
    if (saved < 0) {
        server.send(200, "application/json",
            "{\"ok\":false,\"info\":\"退料时间需在 3~30 秒之间\"}");
        return;
    }
    char msg[96];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"sec\":%d,\"info\":\"退料时间已保存（所有通道统一）\"}", saved);
    server.send(200, "application/json", msg);
}

/* ==========================================================================
 * 启动
 * ========================================================================== */
void web_server_start(void)
{
    s_boot_time = millis();

    wifi_init();

    server.on("/", handle_root);
    server.on("/style.css", handle_style);
    server.on("/app.js", handle_app_js);
    server.on("/status", handle_status);
    server.on("/log", handle_log);
    server.on("/sensor_set", HTTP_POST, handle_sensor_set);
    server.on("/wifi_scan", HTTP_POST, handle_wifi_scan);
    server.on("/wifi_connect", HTTP_POST, handle_wifi_connect);
    server.on("/wifi_set", HTTP_POST, handle_wifi_set);
    server.on("/ota_upload", HTTP_POST, handle_ota_upload_done, handle_ota_upload);
    server.on("/diag_reset", HTTP_POST, handle_diag_reset);
    server.on("/meters_reset", HTTP_POST, handle_meters_reset);
    server.on("/current_set", HTTP_POST, handle_current_set);
    server.on("/current_clear", HTTP_POST, handle_current_clear);
    server.on("/current_channel_set", HTTP_POST, handle_current_channel_set);
    server.on("/jog_set", HTTP_POST, handle_jog_set);
    server.on("/hardware_test", HTTP_POST, handle_hardware_test);
    server.on("/comm_set", HTTP_POST, handle_comm_set);
    server.on("/ams_ver_set", HTTP_POST, handle_ams_ver_set);
    server.on("/pullback_set", HTTP_POST, handle_pullback_set);
    server.on("/assist_set", HTTP_POST, handle_ok);
    server.on("/retract_set", HTTP_POST, handle_ok);
    server.on("/temper_set", HTTP_POST, handle_ok);
    server.on("/access_set", HTTP_POST, handle_ok);
    server.on("/color_set", HTTP_POST, handle_ok);
    server.on("/act485_set", HTTP_POST, handle_act485_set);
    server.on("/bus485raw", HTTP_GET, handle_bus485raw);
    server.onNotFound(handle_not_found);

    server.begin();
    log_i("Web 服务器已启动: http://%s",
          (s_wifi_connected ? WiFi.localIP().toString() : WiFi.softAPIP().toString()).c_str());
}

void web_server_handle(void)
{
    server.handleClient();

    /* OTA 看门狗保险丝：客户端半途断开时 upload 的 END 不会触发
       （library 走 UPLOAD_FILE_ABORTED，已在那里恢复；这里再兜一层）。
       主循环恢复后若 20s 没有新的上传进度，就把看门狗订阅加回来，
       避免系统长期裸奔；真卡死时 10s 后 panic 自恢复。 */
    if (s_ota_wdt_off && (millis() - s_ota_last_prog_ms > 20000)) {
        log_w("OTA: 上传疑似已中断，恢复任务看门狗");
        ota_wdt_on_restore();
    }
}

int web_net_mode(void)
{
    return s_net_mode;
}
