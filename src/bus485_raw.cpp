/**
 * bus485_raw.cpp —— 485 原始报文环形记录
 */

/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
#include "bus485_raw.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

typedef struct {
    uint32_t seq;
    uint32_t t_ms;
    uint8_t  dir;                    /* 0=RX 1=TX */
    uint8_t  len;
    uint8_t  data[BUS485_RAW_MAXLEN];
} raw_frame_t;

static raw_frame_t s_frames[BUS485_RAW_CAP];
static int      s_next = 0;      /* 下一个写入位置 */
static int      s_count = 0;     /* 已存帧数 */
static uint32_t s_seq = 0;
static uint32_t s_rx = 0;
static uint32_t s_tx = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void bus485raw_record(int dir, const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    portENTER_CRITICAL(&s_mux);

    raw_frame_t *f = &s_frames[s_next];
    f->seq = ++s_seq;
    f->t_ms = millis();
    f->dir = (dir == 1) ? 1 : 0;
    f->len = (len > BUS485_RAW_MAXLEN) ? BUS485_RAW_MAXLEN : (uint8_t)len;
    memcpy(f->data, data, f->len);

    s_next = (s_next + 1) % BUS485_RAW_CAP;
    if (s_count < BUS485_RAW_CAP) s_count++;
    if (f->dir) s_tx++; else s_rx++;

    portEXIT_CRITICAL(&s_mux);
}

int bus485raw_dump(char *buf, size_t buflen, uint32_t since, uint32_t *first_seq)
{
    if (!buf || buflen == 0) return 0;
    buf[0] = '\0';
    if (first_seq) *first_seq = 0;

    /* 锁内拷出索引与内容指针快照，hex 转换在锁外做（保持临界区短） */
    int idx[BUS485_RAW_CAP];
    int n = 0;
    portENTER_CRITICAL(&s_mux);
    int start = (s_count < BUS485_RAW_CAP) ? 0 : s_next;
    for (int i = 0; i < s_count; i++) {
        raw_frame_t *f = &s_frames[(start + i) % BUS485_RAW_CAP];
        if (since > 0 && f->seq <= since) continue;
        idx[n++] = (start + i) % BUS485_RAW_CAP;
    }
    if (first_seq && n > 0) *first_seq = s_frames[idx[0]].seq;
    portEXIT_CRITICAL(&s_mux);

    static const char hexd[] = "0123456789ABCDEF";
    int off = 0;
    for (int i = 0; i < n && off < (int)buflen - 8; i++) {
        raw_frame_t *f = &s_frames[idx[i]];
        off += snprintf(buf + off, buflen - off, "%u %u %s %u ",
                        (unsigned)f->seq, (unsigned)f->t_ms,
                        f->dir ? "TX" : "RX", (unsigned)f->len);
        for (int j = 0; j < f->len && off < (int)buflen - 4; j++) {
            buf[off++] = hexd[f->data[j] >> 4];
            buf[off++] = hexd[f->data[j] & 0x0F];
        }
        buf[off++] = '\n';
    }
    buf[off] = '\0';
    return off;
}

uint32_t bus485raw_rx_count(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t v = s_rx;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

uint32_t bus485raw_tx_count(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t v = s_tx;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

uint32_t bus485raw_last_seq(void)
{
    portENTER_CRITICAL(&s_mux);
    uint32_t v = s_seq;
    portEXIT_CRITICAL(&s_mux);
    return v;
}
