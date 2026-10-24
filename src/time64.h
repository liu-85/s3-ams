#pragma once
#include <Arduino.h>

/* 返回毫秒级时间戳（uint64_t）。
 * ESP32 的 millis() 是 uint32_t，这里直接提升到 64 位。
 * 对于本项目的时间差计算（秒级超时），32 位不会溢出。 */
uint64_t get_time64(void);
