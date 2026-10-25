/**
 * bus485_raw.h —— 485 原始报文环形记录（打印机 ↔ AMS 双向）
 *
 * 从上电第一帧就开始记，环形覆盖。给两个消费者：
 *   1. 网页 /bus485raw（浏览器直接看 hex dump）
 *   2. PC 监控脚本 tools/mon485.bat（轮询 ?since=seq 只取新帧）
 *
 * 帧格式（文本，一行一顿）：
 *   <seq> <t_ms> <RX|TX> <len> <hex...>
 *   RX = 打印机 → 本机；TX = 本机 → 打印机
 */

#ifndef BUS485_RAW_H
#define BUS485_RAW_H

#include <Arduino.h>

#define BUS485_RAW_CAP    32    /* 环形帧数 */
#define BUS485_RAW_MAXLEN 256   /* 每帧最多记录字节（超出截断） */

/* dir: 0=RX(打印机发来的) 1=TX(本机回的) */
void bus485raw_record(int dir, const uint8_t *data, size_t len);

/* 生成文本快照到 buf，返回长度；first_seq 出参=缓冲里最旧的 seq（0=空）。
   since > 0 时只输出 seq > since 的帧。 */
int bus485raw_dump(char *buf, size_t buflen, uint32_t since, uint32_t *first_seq);

uint32_t bus485raw_rx_count(void);
uint32_t bus485raw_tx_count(void);
uint32_t bus485raw_last_seq(void);

#endif /* BUS485_RAW_H */
