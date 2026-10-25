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

/* ★★★ v1.1.7：主循环任务栈 8KB → 32KB
 *
 * 症状：v1.1.6 修掉 handle_log 的 7.7KB 栈数组后，/log 请求照样 panic：
 *   Guru Meditation Error: Core 1 panic'ed (Unhandled debug exception).
 *   Debug exception reason: Stack canary watchpoint triggered (loopTask)
 * 回溯 loop() → handleClient() → _handleRequest() → handle_log()，
 * 现场 SP 已越过栈底 652 字节（实测用量 ≈8.8KB > 8KB）。
 *
 * 原因不是某一个函数有大数组，而是 LTO 把 WebServer 的
 * _handleRequest + send + String 拼装整条链全内联，几十个小帧
 * 累积吃掉 8.8KB。Arduino core 的 loopTask 栈由
 * getArduinoLoopTaskStackSize() 决定（core/main.cpp 里是
 * __attribute__((weak))，默认 CONFIG_ARDUINO_LOOP_STACK_SIZE=8192），
 * 这里用同名强符号覆盖成 32KB。S3 有 320KB RAM，多 24KB 无压力。
 *
 * 为什么不在 platformio.ini 加 -DCONFIG_ARDUINO_LOOP_STACK_SIZE：
 * sdkconfig.h 在命令行宏之后 include，会把命令行值重新覆盖回 8192。 */
size_t getArduinoLoopTaskStackSize(void) {
    return 32768;
}

static void watchdog_init() {
    esp_task_wdt_config_t cfg = {
        .timeout_ms = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,  // 不监控空闲任务
        .trigger_panic = true,
    };
    /* ★ v1.1.7：Arduino 核心启动时已按默认参数（5s + 监控 idle 任务）
       初始化过 TWDT，直接 esp_task_wdt_init 会失败：
         E task_wdt: esp_task_wdt_init(515): TWDT already initialized
       于是我们想要的 10s/不监控 idle 配置根本没生效。先反初始化再重来。 */
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_deinit();
        err = esp_task_wdt_init(&cfg);
    }
    if (err != ESP_OK) {
        log_e("TWDT 初始化失败: %s（继续运行，看门狗可能是默认配置）",
              esp_err_to_name(err));
    }
    esp_task_wdt_add(NULL);  // 监控当前任务
    log_i("任务看门狗: %dms 超时，只监控主循环任务", WDT_TIMEOUT_MS);
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
    log_i("主循环任务栈: %u 字节（%u KB）",
          (unsigned)getArduinoLoopTaskStackSize(),
          (unsigned)(getArduinoLoopTaskStackSize() / 1024));

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
