/* ams_log.h 必须第一个 include：劫持 log_i/log_w/log_e，
   让串口日志同步进网页「设备日志」面板（详见头文件注释）。 */
#include "ams_log.h"
#include "time64.h"

uint64_t get_time64(void)
{
    return (uint64_t)millis();
}
