/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
#include "main.h"
#include "AMCU.h"
#include "settings_storage.h"
#include "BambuBus.h"
#include "web_server.h"
#include <Arduino.h>
#include <esp_task_wdt.h>

// 看门狗（ESP32 硬件看门狗，超时 10 秒）
#define WDT_TIMEOUT_MS 10000

static void watchdog_init() {
    esp_task_wdt_config_t cfg = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,  // 不监控空闲任务
        .trigger_panic = true,
    };
    esp_task_wdt_init(&cfg);
    esp_task_wdt_add(NULL);  // 监控当前任务
}

static void watchdog_feed() {
    esp_task_wdt_reset();
}

// Web 服务器已在 web_server.h 中声明

void setup() {
    Serial.begin(115200);
    delay(1000);

    /* 最早装上日志 hook：之后的 log_i/log_w/log_e 都会同步一份到
       网页"设备日志"面板（串口输出不变）。晚了就漏启动日志。 */
    web_log_install_hook();

    log_i("========================================");
    log_i("  s3-ams - ESP32-S3 AMS 固件");
    log_i("  合并项目：Tianshu 协议 + esp32s3-485 硬件");
    log_i("========================================");

    watchdog_init();

    // 初始化设置存储
    settings_storage_init();

    // 初始化 AMCU（含传感器、电机、离合、BambuBus）
    AMCU_init();

    // 启动 Web 服务器（WiFi + HTTP）
    web_server_start();

    log_i("系统初始化完成，进入主循环");
}

void loop() {
    static unsigned long last_wdt = 0;

    watchdog_feed();

    // 核心业务逻辑（含 BambuBus / 缓冲器微动 / AMCU_motion）
    AMCU_run();

    // Web 服务器请求处理
    web_server_handle();

    // 看门狗喂狗（也在循环里喂，确保主循环运行正常）
    if (millis() - last_wdt >= 1000) {
        last_wdt = millis();
        watchdog_feed();
    }

    delay(10);  // 小延迟，让出 CPU 给其他任务（WiFi/Web）
}
