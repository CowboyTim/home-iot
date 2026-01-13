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

/*
 * Sensor support configuration
 *
 */

#define SUPPORT_PLUGINS
#define SUPPORT_S8
#define SUPPORT_SE95
#define SUPPORT_DHT11
#define SUPPORT_LDR
#define SUPPORT_MQ135
#define SUPPORT_APDS9930
#define SUPPORT_BME280
#define SUPPORT_BH1750
#define SUPPORT_DS18B20
#define SUPPORT_MAX30105
#define SUPPORT_NTC
#define SUPPORT_WIFI
#define SUPPORT_ESP_LOG_INFO
#define UART_AT
#define BLUETOOTH_UART_AT
#define LOOP_DELAY

#undef LOGUART
#undef TIMELOG
#undef SUPPORT_GPIO

// Disable UART1 support on S8 builds, as S8 uses UART1 for communication
#ifdef SUPPORT_S8
#undef SUPPORT_BLE_UART1
#undef SUPPORT_UART1
#endif

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "sensors"
#endif

#ifdef SUPPORT_WIFI
#undef SUPPORT_TCP_SERVER
#undef SUPPORT_TCP
#undef SUPPORT_TLS
#define WIFI_WPS
#define SUPPORT_UDP
#define SUPPORT_NTP
#define SUPPORT_MDNS
#endif

#define NR_OF_SENSORS 15

// implement PLUGINS namespace functions
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
  unsigned long v_intv = 0;
  uint8_t enabled = 0;
  uint8_t init_done = 0;
} sensor_c_t;

/* sensors/plugin config */
typedef struct s_cfg_t {
  // 15 chars + null terminator, default "unknown"
  char kvmkey[16]   = {0};
  uint8_t log_uart  = 0;
  uint8_t log_time  = 0;
  char time_fmt[32] = "%Y-%m-%d %H:%M:%S";
  sensor_c_t sensor_cfg[NR_OF_SENSORS] = {0};
} sensors_cfg_t;

typedef struct sensor_r_t {
  const char name[32] = {0};
  const char key[32] = {0};
  const char unit_fmt[12] = {0};
  void *userdata = NULL;
  sensor_c_t *cfg = NULL;
  void   (*init_function)(sensor_r_t*);
  void   (*pre_function)(sensor_r_t*);
  int8_t (*value_function)(sensor_r_t*, float*);
  void   (*post_function)(sensor_r_t*);
  void   (*destroy_function)(sensor_r_t*);
} sensor_r_t;

/* all sensors runtime */
extern sensor_r_t all_sensors[NR_OF_SENSORS];

} // namespace SENSORS
#endif // _SENSORS_H
