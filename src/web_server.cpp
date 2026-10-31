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

/* ---- MQTT 配置（v1.1.19）----
 * 本固件走 RS-485 直连打印机（BambuBus），MQTT 参数**只保存备用**，
 * 不真正去连 Broker。存 NVS 命名空间 "s3ams-mqtt"，掉电保留。 */
static char s_mq_host[64]   = "";
static char s_mq_serial[32] = "";
static char s_mq_user[32]   = "bblp";
static char s_mq_pass[40]   = "";
static char s_mq_cid[40]    = "";
static int  s_mq_port       = 8883;
static bool s_mq_configured = false;

static void mqtt_cfg_load(void)
{
    Preferences p;
    if (!p.begin("s3ams-mqtt", true)) return;
    String host   = p.getString("host", "");
    String serial = p.getString("serial", "");
    String user   = p.getString("user", "bblp");
    String pass   = p.getString("pass", "");
    String cid    = p.getString("cid", "");
    s_mq_port = p.getInt("port", 8883);
    p.end();
    snprintf(s_mq_host,   sizeof(s_mq_host),   "%s", host.c_str());
    snprintf(s_mq_serial, sizeof(s_mq_serial), "%s", serial.c_str());
    snprintf(s_mq_user,   sizeof(s_mq_user),   "%s", user.c_str());
    snprintf(s_mq_pass,   sizeof(s_mq_pass),   "%s", pass.c_str());
    snprintf(s_mq_cid,    sizeof(s_mq_cid),    "%s", cid.c_str());
    s_mq_configured = (s_mq_host[0] != '\0');
}

static void mqtt_cfg_save(void)
{
    Preferences p;
    if (!p.begin("s3ams-mqtt", false)) return;
    p.putString("host", s_mq_host);
    p.putString("serial", s_mq_serial);
    p.putString("user", s_mq_user);
    p.putString("pass", s_mq_pass);
    p.putString("cid", s_mq_cid);
    p.putInt("port", s_mq_port);
    p.end();
    s_mq_configured = (s_mq_host[0] != '\0');
}

/* ---- 通道映射 / 颜色（v1.1.20）----
 * access_list[i] = 料盘位 i 对应的打印机通道（1..8，身份映射为默认）；
 * color_list[i]  = 该料盘位的颜色（RGB，0xRRGGBB）。
 * 存 NVS "s3ams-access"，掉电保留。
 * ★ 之前固件**完全不存**这两个数组，导致网页「通道设置」选完颜色
 *   两秒就被 /status 轮询重建回默认色 —— 本次补上持久化。 */
static int      s_access_list[16];
static uint32_t s_color_list[16];
static const uint32_t k_default_colors[8] = {
    0xd94f4f, 0x3f9d4f, 0x3f6fd9, 0xd9a93f,
    0x7a4fd9, 0x2fb8a5, 0xe57ad9, 0xd98a3f
};

static void access_cfg_reset(void)
{
    for (int i = 0; i < 16; i++) {
        s_access_list[i] = (i < BOARD_CHANNEL_COUNT) ? (i + 1) : 0;
        s_color_list[i]  = (i < 8) ? k_default_colors[i] : 0x888888;
    }
}

static void access_cfg_load(void)
{
    access_cfg_reset();
    Preferences p;
    if (!p.begin("s3ams-access", true)) return;
    if (p.getBytesLength("al") == sizeof(s_access_list)) {
        p.getBytes("al", s_access_list, sizeof(s_access_list));
    }
    if (p.getBytesLength("cl") == sizeof(s_color_list)) {
        p.getBytes("cl", s_color_list, sizeof(s_color_list));
    }
    p.end();
    /* 合法性兜底：通道号必须在 1..16 */
    for (int i = 0; i < 16; i++) {
        if (s_access_list[i] < 1 || s_access_list[i] > 16) s_access_list[i] = i + 1;
    }
}

static void access_cfg_save(void)
{
    Preferences p;
    if (!p.begin("s3ams-access", false)) return;
    p.putBytes("al", s_access_list, sizeof(s_access_list));
    p.putBytes("cl", s_color_list, sizeof(s_color_list));
    p.end();
}

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
static const char *APP_VERSION = "1.1.28-s3ams";

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

/* 取字符串字段：{"key":"value"} → out（截断到 maxlen-1，处理 \" 和 \\ 转义） */
static bool json_get_str(const String &body, const char *key, char *out, size_t maxlen)
{
    if (!out || maxlen == 0) return false;
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
    if (idx >= (int)body.length() || body[idx] != '"') return false;
    idx++;
    size_t n = 0;
    while (idx < (int)body.length() && body[idx] != '"') {
        char c = body[idx];
        if (c == '\\' && idx + 1 < (int)body.length()) {
            idx++;
            c = body[idx];
            if (c == 'n') c = '\n';
            else if (c == 't') c = '\t';
        }
        if (n + 1 < maxlen) out[n++] = c;
        idx++;
    }
    out[n] = '\0';
    return true;
}

/* 取整数数组：{"key":[1,2,3]} → out[]，返回元素个数（最多 maxn） */
static int json_get_int_array(const String &body, const char *key, long *out, int maxn)
{
    if (!out || maxn <= 0) return 0;
    String k = String("\"") + key + "\"";
    int idx = body.indexOf(k);
    if (idx < 0) return 0;
    idx = body.indexOf('[', idx + k.length());
    if (idx < 0) return 0;
    int end = body.indexOf(']', idx);
    if (end < 0) return 0;
    int n = 0;
    int i = idx + 1;
    while (i < end && n < maxn) {
        while (i < end && (body[i] == ' ' || body[i] == ',' ||
                           body[i] == '\t' || body[i] == '\n' || body[i] == '\r')) {
            i++;
        }
        if (i >= end) break;
        bool neg = false;
        if (body[i] == '-') { neg = true; i++; }
        long v = 0;
        bool any = false;
        while (i < end && isDigit((unsigned char)body[i])) {
            v = v * 10 + (body[i] - '0');
            i++;
            any = true;
        }
        if (any) out[n++] = neg ? -v : v;
        else i++;
    }
    return n;
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
/* ★★★ v1.1.28：WiFi 等待期间必须继续服务 485 ★★★
 *
 * 现场（2026-10-31）：间歇性 [0500-40A3] AMS(或AMSlite) A 通信异常，
 * 反复出现、换哪个版本都可能中招，和改了哪一行代码无关。
 *
 * 根因：setup() 的顺序是
 *     AMCU_init();        ← 485 在这里初始化好
 *     web_server_start(); ← wifi_init() → wifi_try_connect() **阻塞等 WiFi**
 *     loop();             ← 到这一步才开始跑 AMCU_run() → BambuBus_run()
 *
 * 而 wifi_try_connect() 里的等待上限是 WIFI_TIMEOUT_MS = **25000ms**：
 *     while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS)
 *         delay(200);
 * 这期间主循环一次都没跑 → 485 一个字节都不处理、一个应答都不发 →
 * 打印机等不到 AMS-A 的应答 → 直接报 [0500-40A3]。
 *
 * 为什么表现为"间歇"：WiFi 1~3 秒连上时，落在打印机重试窗口内，看着正常；
 * 连得慢（信号弱 / DHCP 慢 / 路由器刚上电）就必然超时。这也解释了
 * "同一份固件有时正常有时报错"、"刷完重启就报、再重启又好"。
 *
 * 修法：把所有"等待类"延时都换成这个 —— 每 5ms 跑一次 BambuBus_run()，
 * 485 全程在线。BambuBus_run() 只做收发与状态机记账、不驱动电机，
 * 开机阶段调用是安全的。 */
static void wifi_wait(uint32_t ms)
{
    const uint32_t start = millis();
    while (millis() - start < ms) {
        BambuBus_run();   /* ★ 485 必须一直在应答 */
        delay(5);
    }
}

static void wifi_stop_ap(void)
{
    /* ★ 对标 esp-ams-s3 stop_sta_reconnect：真把射频切走让给 STA，
       不能只 softAPdisconnect —— 单射频下 AP 还占着 STA 就 NO_AP_FOUND */
    if (s_ap_mode) {
        WiFi.softAPdisconnect();
        wifi_wait(200);        /* v1.1.28：原来是裸 delay，485 会失联 */
        WiFi.mode(WIFI_OFF);   /* 让射频完全下线，再切 STA */
        wifi_wait(200);
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
        wifi_wait(300);        /* v1.1.28：原来是裸 delay */
    }

    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect();
    wifi_wait(200);            /* v1.1.28：原来是裸 delay */
    WiFi.begin(ssid, pass);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
        /* ★★★ v1.1.28 关键修复 ★★★
         * 这里原来只有 delay(200) —— 最长 25 秒完全不处理 485，
         * 打印机等不到应答就报 [0500-40A3]。改成 wifi_wait()，
         * 每 200ms 里都在跑 BambuBus_run()。 */
        wifi_wait(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        s_wifi_connected = true;
        s_ap_mode = false;
        s_net_mode = 0;
        /* v1.1.28：把耗时打出来 —— 这个数就是"485 曾失联多久"。
         * >3000ms 说明 WiFi 慢是 40A3 的直接诱因（现在已在等待期间服务 485，
         * 不会因此丢应答，但保留这个数便于判断路由器状态）。 */
        log_i("WiFi 已连接: %s / IP %s (PS=NONE, AP 已关, 连接耗时 %ums)",
              ssid, WiFi.localIP().toString().c_str(),
              (unsigned)(millis() - start));
        return true;
    }

    log_w("WiFi 连接失败（SSID=%s, status=%d）", ssid, (int)WiFi.status());
    if (provision) {
        log_i("配网失败，重新打开配置热点");
        wifi_wait(300);        /* v1.1.28：原来是裸 delay */
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

/* ★ v1.1.28：主循环最大阻塞间隔（在 main.cpp 里统计）
 * 用来判断 40A3 是不是"主循环卡住导致 485 丢字节"造成的。 */
extern uint32_t amcu_loop_max_gap_ms(void);
extern void amcu_loop_gap_reset(void);

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
    json += "\"mqtt_configured\":" + String(s_mq_configured ? "true" : "false") + ",";
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
    /* v1.1.26：自动送料 PWM（WEB「参数设置」可调，范围 40~100） */
    json += "\"feed_pwm\":" + String(settings_get_feed_pwm()) + ",";
    /* v1.1.27：缓冲送料连续运行上限（秒，5~120） */
    json += "\"feed_timeout\":" + String(settings_get_feed_timeout()) + ",";
    json += "\"current_access\":" + String((cur >= 0) ? (cur + 1) : 0) + ",";
    /* v1.1.20：通道映射 + 颜色（网页「通道设置」用；之前完全没输出） */
    json += "\"access_list\":[";
    for (int ai = 0; ai < BOARD_CHANNEL_COUNT; ai++) {
        if (ai) json += ",";
        json += String(s_access_list[ai]);
    }
    json += "],";
    json += "\"color_list\":[";
    for (int ai = 0; ai < BOARD_CHANNEL_COUNT; ai++) {
        if (ai) json += ",";
        json += String((unsigned long)s_color_list[ai]);
    }
    json += "],";
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

    /* ===== v1.1.25：485 链路诊断 =====
     * 打印机报 [0500-40A3] AMS A 通信异常时，看这一段就能定性：
     *   rx_bytes 不涨              → 物理层没数据（线松/焊点虚/收发器坏/A-B 反）
     *   rx_bytes 涨 + crc_err 猛涨 → 干扰或共地不良（电机电源纹波常见）
     *   rx_frames 涨 + tx_402=0    → 我们没应答序列号查询（协议分支问题）
     *   tx_402 涨 + 仍报 40A3      → 应答内容不对（台号/长度/型号指纹）
     *   last_rx_age_ms 很大        → 打印机没在跟我们说话
     */
    {
        uint32_t rb = 0, rf = 0, ce = 0, ov = 0, t402 = 0, t103 = 0, t06 = 0, lrx = 0;
        bambubus_485_stats(&rb, &rf, &ce, &ov, &t402, &t103, &t06, &lrx);
        long age = (lrx == 0) ? -1 : (long)(millis() - lrx);
        json += ",\"link\":{\"rx_bytes\":" + String((unsigned long)rb) +
                ",\"rx_frames\":" + String((unsigned long)rf) +
                ",\"crc_err\":" + String((unsigned long)ce) +
                ",\"overflow\":" + String((unsigned long)ov) +
                ",\"tx_402\":" + String((unsigned long)t402) +
                ",\"tx_103\":" + String((unsigned long)t103) +
                ",\"tx_06\":" + String((unsigned long)t06) +
                ",\"last_rx_age_ms\":" + String(age) +
                ",\"loop_max_gap_ms\":" + String((unsigned long)amcu_loop_max_gap_ms()) + "}";
    }
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
        /* v1.1.23：最近一次收到的 statu 位掩码。流量校准阶段"微动没反应"时
           看它就知道打印机到底把这一路当成什么状态（0x0F=使用中、0x03=送料、
           0x09=进入使用、0x00/0x01=空闲）。 */
        json += "\"statu\":" + String((unsigned)get_channel_last_statu(dn)) + ",";
        json += "\"active\":" + String(is_channel_active_recently(dn, 10000) ? 1 : 0) + ",";
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
        /* ★ v1.1.26：最近一次电机运行的电流剖析 —— 标定堵转阈值用
         *   start_peak = 启动段(前 800ms)峰值，正常是稳态的 3~8 倍
         *   run_avg    = 稳态均值，正常送料时的典型值
         *   run_peak   = 稳态峰值，堵转时这里会明显抬高
         *   max        = 全程最大
         * 判读：堵转阈值建议取 run_peak 的 2 倍左右，且明显低于 start_peak。 */
        motor_current_stat_t st;
        if (motor_get_current_stat(&st)) {
            json += ",\"last\":{\"valid\":true";
            json += ",\"dir\":" + String((int)st.dir);
            json += ",\"pct\":" + String(st.pct);
            json += ",\"ms\":" + String((unsigned long)st.duration_ms);
            json += ",\"start_peak\":" + String(st.start_peak_ma);
            json += ",\"run_avg\":" + String(st.run_avg_ma);
            json += ",\"run_peak\":" + String(st.run_peak_ma);
            json += ",\"max\":" + String(st.max_ma);
            json += "}";
        } else {
            json += ",\"last\":{\"valid\":false}";
        }
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
    /* ★ v1.1.24：8 只微动脚的**原始** digitalRead 电平（1=HIGH 松开 / 0=LOW 压下）。
       不受去抖、不受启用掩码、不受通道映射影响 —— 用来把
       「引脚没信号（硬件）」和「有信号但被软件挡（掩码/去抖）」分开。
       pins 给出对应的 GPIO 号，便于对照原理图。 */
    json += "\"raw\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) json += ",";
        json += String(sensor_raw_level(i));
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
    /* 简单解析 channel 和 enable
     *
     * ★ v1.1.24：原来找的是 "\"enable\""，但前端 app.js 发的字段名是
     *   **"enabled"**（`{channel:ch, func:..., enabled:0/1}`）。
     *   `"enable"`（带引号）在 `"enabled"` 里**匹配不上**（e 后面是 d 不是 "），
     *   于是 eidx 恒为 -1、enable 恒为 -1、`enable != 0` 恒为 true →
     *   缓冲送料开关**只能开、关不掉**。
     *   这里两种写法都认。 */
    int channel = -1, enable = -1;
    int cidx = body.indexOf("\"channel\"");
    int eidx = body.indexOf("\"enabled\"");
    if (eidx < 0) eidx = body.indexOf("\"enable\"");
    if (cidx >= 0) channel = body.substring(cidx + 9, body.indexOf("}", cidx)).toInt();
    if (eidx >= 0) {
        int off = body.indexOf("\"enabled\"") >= 0 ? 10 : 9;   /* 跳过 "enabled" / "enable" */
        enable = body.substring(eidx + off, body.indexOf("}", eidx)).toInt();
    }

    bool ok = false;
    if (channel >= 1 && channel <= BOARD_CHANNEL_COUNT) {
        ok = sensor_set_channel_enabled(channel - 1, enable != 0);
    }
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

/* ★ v1.1.24：微动原始电平诊断页（纯 JSON，浏览器直接打开就能看）
 *
 *   http://192.168.2.100/sensor_raw
 *
 * 返回：
 *   pins  —— 8 只微动对应的 GPIO 号
 *   raw   —— 8 只微动的原始电平（1=HIGH 松开 / 0=LOW 压下）
 *   trig  —— 去抖后的触发状态（按"开始位/停止位"配对后的通道视图）
 *   mask  —— 缓冲送料启用掩码（0x0F = 前 4 路都开）
 *   deb   —— 去抖计数（0..3，到了 3 才认可）
 *
 * 判读：
 *   按下微动 raw 从 1 变 0 → 硬件没问题，问题在软件（掩码/映射）
 *   按下 raw 不变（一直是 1）→ 微动没接通 / GND 没共
 *   一直 0（松开也不回 1）  → 引脚被拉死：常闭微动、短路、或引脚被占用 */
static void handle_sensor_raw(void)
{
    String j = "{\"mode\":\"";
    j += SENSOR_TEMP_DIGITAL ? "digital" : "encoded";
    j += "\",\"mask\":" + String((unsigned)sensor_enabled_mask());
    j += ",\"pins\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) j += ",";
        j += String(sensor_pin_of(i));
    }
    j += "],\"raw\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i) j += ",";
        j += String(sensor_raw_level(i));
    }
    j += "],\"trig\":[";
    for (int i = 0; i < BOARD_CHANNEL_COUNT / 2; i++) {
        if (i) j += ",";
        j += "[";
        j += String(sensor_triggered(i, SENSOR_STOP) ? 1 : 0);
        j += ",";
        j += String(sensor_triggered(i, SENSOR_START) ? 1 : 0);
        j += "]";
    }
    j += "]}";
    server.send(200, "application/json; charset=utf-8", j);
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
    bambubus_485_stats_reset();   /* v1.1.25：485 计数也清零，便于改接线后对照 */
    amcu_loop_gap_reset();        /* v1.1.28：主循环最大间隔也重新统计 */
    server.send(200, "application/json", "{\"ok\":true,\"info\":\"已清除离合冲突/仲裁计数、485 链路计数与主循环间隔\"}");
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
/* ★ v1.1.27：/feed_timeout_set {"sec":n} —— 缓冲送料连续运行上限
 *
 * 现场要解决的问题：打印中「开始微动」一直压着（停止微动不触发 / 微动卡死），
 * 缓冲逻辑会一直送料，电机长时间空转，料被顶进挤出机。
 * 到点强制停，等微动松开或停止位触发后重新计时。
 *
 * 默认 30 秒（实测正常补料一次 7~15 秒，留一倍余量），范围 5~120。 */
static void handle_feed_timeout_set(void)
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
    int saved = settings_set_feed_timeout((int)sec);
    if (saved < 0) {
        server.send(200, "application/json",
            "{\"ok\":false,\"info\":\"送料超时需在 5~120 秒之间\"}");
        return;
    }
    char msg[112];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"sec\":%d,\"info\":\"缓冲送料超时已保存\"}", saved);
    server.send(200, "application/json", msg);
}

/* ★ v1.1.26：/feed_pwm_set {"pct":n} —— 自动送料（微动缓冲送料）的电机 PWM
 *
 * 现场为什么要可调：本硬件电机 PWM 低于 80 起不来，但不同电机/减速箱、
 * 不同耗材阻力下的"合适速度"不一样。满速送料容易把缓冲器顶得过满
 * （顶到停止位后压轮打滑），太慢又跟不上打印机拉料。
 * 范围 40~100，默认 100。改完立即生效并写 NVS。 */
static void handle_feed_pwm_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long pct = 0;
    if (!json_get_int(body, "pct", &pct)) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"缺少 pct\"}");
        return;
    }
    int saved = settings_set_feed_pwm((int)pct);
    if (saved < 0) {
        server.send(200, "application/json",
            "{\"ok\":false,\"info\":\"送料 PWM 需在 40~100 之间（实测低于 80 电机起不来）\"}");
        return;
    }
    char msg[96];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"pct\":%d,\"info\":\"自动送料 PWM 已保存\"}", saved);
    server.send(200, "application/json", msg);
}

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

/* /get_mqtt_info（GET）：回传已保存的 MQTT 参数（出于安全不回传访问码） */
static void handle_get_mqtt_info(void)
{
    String json = String("{\"host\":\"") + s_mq_host +
        "\",\"serial\":\"" + s_mq_serial +
        "\",\"user\":\"" + s_mq_user +
        "\",\"port\":" + String(s_mq_port) +
        ",\"client_id\":\"" + s_mq_cid + "\"}";
    server.send(200, "application/json", json);
}

/* /mqtt_connect（POST）：保存 MQTT 参数到 NVS。
 * 本固件走 RS-485 直连打印机，这里**只保存**，不真正连 MQTT Broker。 */
static void handle_mqtt_connect(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    json_get_str(body, "host",      s_mq_host,   sizeof(s_mq_host));
    json_get_str(body, "serial",    s_mq_serial, sizeof(s_mq_serial));
    json_get_str(body, "user",      s_mq_user,   sizeof(s_mq_user));
    json_get_str(body, "client_id", s_mq_cid,    sizeof(s_mq_cid));
    /* 访问码：留空 = 不修改 */
    char pw[40];
    if (json_get_str(body, "password", pw, sizeof(pw)) && pw[0] != '\0') {
        snprintf(s_mq_pass, sizeof(s_mq_pass), "%s", pw);
    }
    long port = 0;
    if (json_get_int(body, "port", &port) && port > 0 && port < 65536) {
        s_mq_port = (int)port;
    }
    mqtt_cfg_save();
    log_i("MQTT 配置已保存: host=%s port=%d（本固件走 RS-485，不连接）",
          s_mq_host, s_mq_port);
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"MQTT 参数已保存（本固件走 RS-485 直连，不会用它连接）\"}");
}

/* /stop（POST）：紧急停止 —— 停电机 + 断离合 + 取消点动 */
static void handle_stop(void)
{
    amcu_jog_stop();          /* 先取消点动（它会停电机 + 断离合） */
    AMCU_drive_stop();        /* 再兜底停一次 */
    clutch_release_all();
    log_w("★ 收到「紧急停止」：电机已停、离合已全断");
    server.send(200, "application/json",
        "{\"ok\":true,\"info\":\"已紧急停止（电机停、离合全断）\"}");
}

/* /ap_set（POST）：{on:0|1} 开关配置热点 */
static void handle_ap_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long on = 0;
    json_get_int(body, "on", &on);
    if (on) {
        wifi_start_ap();
        server.send(200, "application/json", "{\"ok\":true,\"info\":\"配置热点已打开\"}");
    } else {
        wifi_stop_ap();
        server.send(200, "application/json", "{\"ok\":true,\"info\":\"配置热点已关闭\"}");
    }
}

/* /access_set（POST）：保存通道映射 + 颜色（v1.1.20 真实现）
 * 前端 body: {access_list:[1..8], color_list:[0xRRGGBB, ...]} */
static void handle_access_set(void)
{
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
        return;
    }
    String body = server.arg("plain");
    long al[16], cl[16];
    int na = json_get_int_array(body, "access_list", al, 16);
    int nc = json_get_int_array(body, "color_list", cl, 16);
    if (na <= 0 && nc <= 0) {
        server.send(400, "application/json",
            "{\"ok\":false,\"err\":\"缺少 access_list / color_list\"}");
        return;
    }
    for (int i = 0; i < 16 && i < na; i++) {
        if (al[i] >= 1 && al[i] <= 16) s_access_list[i] = (int)al[i];
    }
    for (int i = 0; i < 16 && i < nc; i++) {
        s_color_list[i] = (uint32_t)(cl[i] & 0xFFFFFF);
    }
    access_cfg_save();
    log_i("通道设置已保存: access=%d 项 color=%d 项", na, nc);
    server.send(200, "application/json", "{\"ok\":true,\"info\":\"通道与颜色已保存\"}");
}

/* 本固件不支持的可选动作：明确回 ok:false，避免 404 让前端弹「请求失败」 */
static void handle_unsupported(void)
{
    server.send(200, "application/json",
        "{\"ok\":false,\"info\":\"本固件（RS-485 直连）不支持该操作\"}");
}

/* ==========================================================================
 * 启动
 * ========================================================================== */
void web_server_start(void)
{
    s_boot_time = millis();
    mqtt_cfg_load();     /* v1.1.19：读回已保存的 MQTT 参数（只用于显示/保存） */
    access_cfg_load();   /* v1.1.20：读回通道映射 + 颜色 */

    wifi_init();

    server.on("/", handle_root);
    server.on("/style.css", handle_style);
    server.on("/app.js", handle_app_js);
    server.on("/status", handle_status);
    server.on("/log", handle_log);
    server.on("/sensor_set", HTTP_POST, handle_sensor_set);
    server.on("/sensor_raw", HTTP_GET, handle_sensor_raw);
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
    server.on("/feed_pwm_set", HTTP_POST, handle_feed_pwm_set);   /* v1.1.26 */
    server.on("/feed_timeout_set", HTTP_POST, handle_feed_timeout_set);  /* v1.1.27 */
    server.on("/assist_set", HTTP_POST, handle_ok);
    server.on("/retract_set", HTTP_POST, handle_ok);
    server.on("/temper_set", HTTP_POST, handle_ok);
    server.on("/access_set", HTTP_POST, handle_access_set);
    server.on("/color_set", HTTP_POST, handle_ok);
    server.on("/act485_set", HTTP_POST, handle_act485_set);
    server.on("/bus485raw", HTTP_GET, handle_bus485raw);
    /* v1.1.19：新控制台（模板版）需要的接口。
       能真实现的真实现，其余明确回「不支持」，避免 404 让前端弹「请求失败」。 */
    server.on("/get_mqtt_info", HTTP_GET, handle_get_mqtt_info);
    server.on("/mqtt_connect", HTTP_POST, handle_mqtt_connect);
    server.on("/stop", HTTP_POST, handle_stop);
    server.on("/ap_set", HTTP_POST, handle_ap_set);
    server.on("/boot_clear", HTTP_GET, handle_unsupported);
    server.on("/creep_set", HTTP_POST, handle_unsupported);
    server.on("/assist_test", HTTP_POST, handle_unsupported);
    server.on("/extruder_src_set", HTTP_POST, handle_unsupported);
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
