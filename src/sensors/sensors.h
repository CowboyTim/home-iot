/*
 * sensors.h: Sensor handling for esp32c3 with DHT11, SE95, S8, LDR, MQ-135, APDS-9930
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

// Logging setup for esp32c3

#ifndef _SENSORS_H
#define _SENSORS_H

#include <Arduino.h>

#define SUPPORT_PLUGINS
#define UART_AT
#define BLUETOOTH_UART_AT
#define SUPPORT_WIFI
#define SUPPORT_NTP
#define SUPPORT_MDNS
#define SUPPORT_ESP_LOG_INFO
#define WIFI_WPS
#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "sensors"
#endif // DEFAULT_HOSTNAME

#undef SUPPORT_UART1
#undef SUPPORT_BLE_UART1
#define SUPPORT_UDP
#undef SUPPORT_TCP
#undef SUPPORT_TLS
#undef SUPPORT_TCP_SERVER
#undef LOGUART

#define S8
#define SE95
#define DHT11
#define LDR
#undef APDS9930   // TODO: implement software + hardware
#define MQ135

#define NR_OF_SENSORS 8
#define SENSORS_OUTBUFFER_SIZE  128

#ifdef SE95
#include <Wire.h>
#define I2C_DAT   6
#define I2C_CLK   7
#define SE95_I2C_ADDRESS      0x48 // default I2C address for SE95
#define SE95_TEMPERATURE      0x00 // Command to read temperature from SE95 sensor
#define SE95_CONFIGURATION    0x01 // Command to configure SE95 sensor
#define SE95_THYST            0x02 // Command to store the hysteresis threshold
#define SE95_TOS              0x03 // Command to store the overtemperature shutdown threshold
#define SE95_ID               0x05 // Command to read the ID of the SE95 sensor
#endif // SE95

#ifdef S8
/* Sensair S8 LP sensor for CO2 */
#include "s8_uart.h"
#endif // S8
       //
#ifdef DHT11
#include <DFRobot_DHT11.h>
#endif // DHT11

namespace PLUGINS {
  extern void setup();
  extern void loop_pre();
  extern void loop_post();
}

extern uint8_t inbuf[];
extern const uint8_t *inbuf_max;
extern size_t inlen;

namespace SENSORS {

#ifdef SE95
// re-export Wire for sensors.cpp in this SENSORS namespace, note that "Wire"
// is a global extern object
TwoWire Wire = Wire;
#endif // SE95

typedef struct sensor_t {
  const char name[32] = {0};
  const char unit_fmt[24] = {0};
  const char key[32] = {0};
  const char out_buf[32] = {0};
  uint8_t  enabled = 0;
  void *userdata = NULL;
  unsigned long v_intv = 1000;
  unsigned long l_intv = 0;
  void   (*init_function)(const sensor_t*);
  void   (*pre_function)(const sensor_t*);
  double (*value_function)(const sensor_t*);
  void   (*post_function)(const sensor_t*);
  void   (*destroy_function)(const sensor_t*);
} sensor_t;

} // namespace SENSORS
#endif // _SENSORS_H
