/*
 * sensors.h: Sensor handling for esp32c3
 *
 * Author: CowboyTim
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 */

#ifndef _SENSORS_H
#define _SENSORS_H

// Logging setup for esp32c3

#define SUPPORT_PLUGINS

#ifndef LOGUART
#define LOGUART
#endif // LOGUART
#undef LOGUART

#ifndef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_AT
#endif // BLUETOOTH_UART_AT

#ifndef SUPPORT_ESP_LOG_INFO
#define SUPPORT_ESP_LOG_INFO
#endif // SUPPORT_ESP_LOG_INFO

#ifndef TIMELOG
#define TIMELOG
#endif // TIMELOG
#undef TIMELOG

#ifndef LOOP_DELAY
#define LOOP_DELAY
#endif // LOOP_DELAY

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "sensors"
#endif // DEFAULT_HOSTNAME

#ifndef UART_AT
#define UART_AT
#endif // UART_AT

#ifndef SUPPORT_UART1
#define SUPPORT_UART1
#endif // SUPPORT_UART1
#undef SUPPORT_UART1

#ifndef SUPPORT_BLE_UART1
#define SUPPORT_BLE_UART1
#endif // SUPPORT_BLE_UART1
#undef SUPPORT_BLE_UART1

#ifndef SUPPORT_GPIO
#define SUPPORT_GPIO
#endif // SUPPORT_GPIO
#undef SUPPORT_GPIO

#ifndef SUPPORT_WIFI
#define SUPPORT_WIFI
#endif // SUPPORT_WIFI

#ifdef SUPPORT_WIFI

// WiFi support enabled, enable related features if not explicitly disabled
#ifndef WIFI_WPS
#define WIFI_WPS
#endif // WIFI_WPS

#undef SUPPORT_TCP_SERVER

#undef SUPPORT_TCP

#undef SUPPORT_TLS

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP

#ifndef SUPPORT_NTP
#define SUPPORT_NTP
#endif // SUPPORT_NTP

#ifndef SUPPORT_MDNS
#define SUPPORT_MDNS
#endif // SUPPORT_MDNS

#endif // SUPPORT_WIFI

/*
 * Sensor support configuration
 */

#undef SUPPORT_S8
#define SUPPORT_SE95
#define SUPPORT_DHT11
#define SUPPORT_LDR
#define SUPPORT_MQ135
#define SUPPORT_APDS9930
#define SUPPORT_BME280
#define SUPPORT_BH1750

#define NR_OF_SENSORS 14
#define SENSORS_OUTBUFFER_SIZE  128

#include <Arduino.h>

// implemented in sensors/sensors.ino
namespace PLUGINS {
  void initialize();
  void setup();
  void loop_pre();
  void loop_post();
  long max_sleep_time();
  void clear_config();
  const char * at_cmd_handler(const char *at_cmd);
  const char * at_get_help_string();
}

// from main sensors/esp-at.ino, declare extern
extern uint8_t inbuf[];
extern const uint8_t *inbuf_max;
extern size_t inlen;

namespace SENSORS {

typedef struct sensor_c_t {
  uint8_t enabled = 0;
  unsigned long v_intv = 1000;
} sensor_c_t;

/* sensors/plugin config */
typedef struct s_cfg_t {
  // 16 chars + null terminator, default "unknown"
  char kvmkey[17]      = "unknown";
  uint8_t log_uart     = 0;
  #ifdef SUPPORT_MQ135
  double mq135_r0      = 0;
  double mq135_rl      = 0;
  #endif // SUPPORT_MQ135
  sensor_c_t sensor_cfg[NR_OF_SENSORS] = {0};
} sensors_cfg_t;

typedef struct sensor_r_t {
  const char name[32] = {0};
  const char key[32] = {0};
  const char unit_fmt[24] = {0};
  const char out_buf[32] = {0};
  void *userdata = NULL;
  sensor_c_t *cfg = NULL;
  void   (*init_function)(sensor_r_t*);
  void   (*pre_function)(sensor_r_t*);
  int8_t (*value_function)(sensor_r_t*, double*);
  void   (*post_function)(sensor_r_t*);
  void   (*destroy_function)(sensor_r_t*);
} sensor_r_t;

/* all sensors runtime */
extern sensor_r_t all_sensors[NR_OF_SENSORS];

/* main config */
sensors_cfg_t cfg = {
  .kvmkey     = "unknown",
  .log_uart   = 0,
  #ifdef SUPPORT_MQ135
  .mq135_r0   = 0, // default R0 for MQ-135
  .mq135_rl   = 0, // default RL for MQ-135
  #endif // SUPPORT_MQ135
  .sensor_cfg = {0}
};


} // namespace SENSORS
#endif // _SENSORS_H
