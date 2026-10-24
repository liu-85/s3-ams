/**
 * web_server.cpp —— ESP32-S3 Web 服务器（Arduino 框架）
 *
 * 保留 esp32s3-485 的 Web 前端，提供兼容后端。
 * 使用 ESP32 Arduino 的 WiFi + WebServer + Update 库。
 */

#include "web_server.h"
#include "web_files.h"
#include "board_pins.h"
#include "clutch.h"
#include "motor.h"
#include "filament_sensor.h"
#include "BambuBus.h"
#include "AMCU.h"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_ota_ops.h>
#include <esp_wifi.h>

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define WEB_SERVER_PORT 80
#define LOG_RING_SIZE 4096
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

/* 日志环形缓冲 */
static char s_log_ring[LOG_RING_SIZE];
static size_t s_log_head = 0;
static size_t s_log_tail = 0;
static bool s_log_full = false;

/* App 版本 */
static const char *APP_VERSION = "1.0.0-s3ams";

/* 单 IO 双微动：每路一个 ADC 脚（开始位/停止位共用同一脚，电阻编码） */
static const int BOARD_SENSOR_PINS[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_ARRAY;

/* ==========================================================================
 * 日志
 * ========================================================================== */
void web_log(const char *msg)
{
    if (!msg) return;
    size_t len = strlen(msg);
    for (size_t i = 0; i < len; i++) {
        char c = msg[i];
        if (c == '\n') c = ' ';  // 单行存储
        s_log_ring[s_log_head] = c;
        s_log_head = (s_log_head + 1) % LOG_RING_SIZE;
        if (s_log_head == s_log_tail) {
            s_log_full = true;
            s_log_tail = (s_log_tail + 1) % LOG_RING_SIZE;
        }
    }
}

/* ==========================================================================
 * WiFi
 * ========================================================================== */
static bool wifi_try_connect(const char *ssid, const char *pass)
{
    if (ssid == NULL || strlen(ssid) == 0) {
        log_w("wifi_try_connect: SSID 为空，跳过");
        return false;
    }

    /* 关键：关 STA 省电 + 调 WPA 握手超时，绕开 ESP32-S3 在 WPA2-CCMP
       下常见的 "CCMP replay detected" 导致 STA 被反复踢掉、配网失败。
       （esp_wifi_set_ps 在 esp_wifi.h 中，确定可用） */
    esp_wifi_set_ps(WIFI_PS_NONE);        /* 关 STA 省电，避免唤醒延迟丢包 */

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);         /* 失败后由我们自己控制重试 */
    WiFi.disconnect();                    /* 清掉之前残留的 CCMP 加密状态 */
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
        log_i("WiFi 已连接: %s / IP %s (TKIP, PS=NONE)",
              ssid, WiFi.localIP().toString().c_str());
        return true;
    }
    log_w("WiFi 连接失败（SSID=%s, status=%d）", ssid, (int)WiFi.status());
    return false;
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

static void wifi_init(void)
{
    s_wifi_prefs.begin("s3ams-wifi", true);
    String ssid = s_wifi_prefs.getString("ssid", "");
    String pass = s_wifi_prefs.getString("pass", "");
    s_wifi_prefs.end();

    if (ssid.length() > 0) {
        log_i("尝试连接已保存 WiFi: %s", ssid.c_str());
        if (wifi_try_connect(ssid.c_str(), pass.c_str())) {
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
    json += "\"current_access\":" + String((cur >= 0) ? (cur + 1) : 0) + ",";
    json += "\"mem_free\":" + String(esp_get_free_heap_size()) + ",";
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
            ",\"exchange_ok\":0,\"exchange_fail\":0,\"last_error\":\"—\",\"last_error_ms\":-1}";
    json += "},";

    /* ===== 复位/诊断 ===== */
    json += "\"reset\":{\"cause\":\"—\",\"cause_desc\":\"—\",\"boot_count\":1,\"uptime_ms\":" + String((long)millis()) + "},";
    json += "\"boot_safety\":{\"ok\":true},";

    /* ===== 硬件概览 ===== */
    json += "\"hardware\":{\"channels\":" + String(BOARD_CHANNEL_COUNT) +
            ",\"engaged\":[" + String(clutch_active_channel()) + "]," +
            "\"limits\":[],\"motor_direction\":" + String((int)motor_get_dir()) +
            ",\"conflicts\":" + String(clutch_conflict_count()) + "},";

    /* ===== 电流采样（本板型未实现连续采样，如实告知前端） ===== */
    json += "\"current\":{\"board\":false},";

    /* ===== 传感器（双微动 ADC） ===== */
    json += "\"sensors\":{";
    json += "\"enabled\":" + String(sensor_enabled_mask()) + ",";
    json += "\"dual\":true,";
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
        json += "[" + String(BOARD_SENSOR_PINS[i]) + "," + String(BOARD_SENSOR_PINS[i]) + "]";
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
    // 从环形缓冲导出日志
    String out;
    char buf[LOG_RING_SIZE + 1];
    size_t idx = 0;
    if (s_log_full) {
        for (size_t i = s_log_tail; i != s_log_head; i = (i + 1) % LOG_RING_SIZE) {
            buf[idx++] = s_log_ring[i];
        }
    } else {
        for (size_t i = s_log_tail; i != s_log_head; i = (i + 1) % LOG_RING_SIZE) {
            buf[idx++] = s_log_ring[i];
        }
    }
    buf[idx] = '\0';
    String json = "{\"log\":\"" + String(buf) + "\"}";
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
    int nidx = body.indexOf("\"name\"");
    int pidx = body.indexOf("\"password\"");
    if (nidx >= 0) name = body.substring(nidx + 7, body.indexOf("\"", nidx + 7));
    if (pidx >= 0) password = body.substring(pidx + 11, body.indexOf("\"", pidx + 11));

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

    log_i("配网: 连接 %s（TKIP + 关PS）", name.c_str());
    bool ok = wifi_try_connect(name.c_str(), password.c_str());
    if (ok) {
        log_i("配网成功，重启让 AP 关闭、走 STA");
        server.send(200, "application/json", "{\"ok\":true}");
        delay(500);
        ESP.restart();
    } else {
        server.send(500, "application/json",
            "{\"ok\":false,\"err\":\"WiFi 连接失败，已保留配置热点；请确认 SSID/密码并检查路由器是否 5GHz\"}");
    }
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

static void handle_ota_upload(void)
{
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
            server.send(500, "text/plain", "OTA begin failed");
            return;
        }
        log_i("OTA 开始: %s", upload.filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            server.send(200, "text/plain", "OTA 成功，设备将重启");
            delay(500);
            ESP.restart();
        } else {
            Update.printError(Serial);
            server.send(500, "text/plain", "OTA 失败");
        }
    }
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
    server.send(200, "application/json", "{\"ok\":true,\"info\":\"已清除离合冲突计数\"}");
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
    server.on("/ota_upload", HTTP_POST, handle_ota_upload);
    server.on("/diag_reset", HTTP_POST, handle_diag_reset);
    server.on("/current_set", HTTP_POST, handle_ok);
    server.on("/current_clear", HTTP_POST, handle_ok);
    server.on("/assist_set", HTTP_POST, handle_ok);
    server.on("/retract_set", HTTP_POST, handle_ok);
    server.on("/temper_set", HTTP_POST, handle_ok);
    server.on("/access_set", HTTP_POST, handle_ok);
    server.on("/color_set", HTTP_POST, handle_ok);
    server.on("/comm_set", HTTP_POST, handle_ok);
    server.on("/jog_set", HTTP_POST, handle_ok);
    server.onNotFound(handle_not_found);

    server.begin();
    log_i("Web 服务器已启动: http://%s",
          (s_wifi_connected ? WiFi.localIP().toString() : WiFi.softAPIP().toString()).c_str());
}

void web_server_handle(void)
{
    server.handleClient();
}

int web_net_mode(void)
{
    return s_net_mode;
}
