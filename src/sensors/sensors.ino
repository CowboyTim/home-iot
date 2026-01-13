/*
 * sensors.cpp: Sensor handling for ESP32/ESP8266 DHT11/BME280 logger
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

// Logging setup for esp32c3: before any other include wrt arduino workflow

// Setup

#include <sensors.h>
#include <common.h>

#ifdef SUPPORT_DHT11
#include <DHT.h>
#endif // SUPPORT_DHT11

#if defined(SUPPORT_SE95) || defined(SUPPORT_BME280) || defined(SUPPORT_BMP280) || defined(SUPPORT_APDS9930) || defined(SUPPORT_BH1750)
#define I2C_SDA     GPIO_NUM_6 // SDA: GPIO_NUM_8 -> same as LED
#define I2C_SCL     GPIO_NUM_7 // SCL: GPIO_NUM_9
#define I2C_BUS_NUM 0
#define I2C_FREQ    100000L // 100kHz
#define I2C_TIMEOUT 100     // 100ms
#ifdef NO_GLOBAL_WIRE
#include <Wire.h>
#else
// "localize" NO_GLOBAL_WIRE to only this include
#define NO_GLOBAL_WIRE
#include <Wire.h>
#undef NO_GLOBAL_WIRE
#endif
#endif

#if defined(SUPPORT_DS18B20)
#include <OneWire.h>
#include <DallasTemperature.h>
#endif // SUPPORT_DS18B20

#if defined(SUPPORT_NTC) || defined(SUPPORT_LDR) || defined(SUPPORT_MQ135)
#include "driver/gpio.h"
#include "hal/adc_types.h"
#include "soc/adc_channel.h"
#include "esp_adc/adc_oneshot.h"
#include "soc/clk_tree_defs.h"
#define ADC_BITS   ADC_BITWIDTH_12 // 12-bit ADC
#define ADC_MAX               4095 // 12-bit ADC max value: 2^12 - 1 = 4095
#define ADC_MREF_VOLTAGE   2800.0f // ESP32 ADC reference voltage in mV for attenuation 12dB, in mV
#define ADC_NEEDED
#endif // SUPPORT_NTC || SUPPORT_LDR || SUPPORT_MQ135


#ifdef SUPPORT_S8
#include "s8_uart.h"
#endif

namespace SENSORS {

// sensors/plugin config
ALIGN(4) sensors_cfg_t cfg = {
  .kvmkey     = {0},
  .log_uart   = 0,
  .log_time   = 0,
  .time_fmt   = "%Y-%m-%d %H:%M:%S",
  .sensor_cfg = {0}
};

// used alot in this file
ALIGN(4) esp_err_t esp_ok;

// last interval counters
RTC_DATA_ATTR long l_intv_counters[NR_OF_SENSORS] = {0};

// DS18B20 temperature sensor for use with MQ135 temp compensation
#ifdef SUPPORT_DS18B20
RTC_DATA_ATTR unsigned long last_read_time = 0;
RTC_DATA_ATTR float last_temperature = 0.0f;
#endif // SUPPORT_DS18B20

#if defined(ADC_NEEDED)
// see https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_oneshot.html
// note that ADC1 has A0-A4 and ADC2 has A5, but ADC2 is in use by WiFi, so we only use ADC1 pins A0-A4 here
#define ADC1_PINS 5 // A0-A4
RTC_DATA_ATTR int8_t setup_adc[ADC1_PINS] = {0}; // ADC setup flag, 0=not setup, 1=setup done, -1=setup in progress
adc_oneshot_unit_handle_t adc_handle[ADC1_PINS];
adc_oneshot_unit_init_cfg_t adc_init_config[ADC1_PINS];
adc_channel_t adc_channel[ADC1_PINS];

NOINLINE
int8_t initialize_adc(uint8_t pin){
  if(setup_adc[pin] == 0){
    LOG("[ADC] set ADC resolution to %d bits", ADC_BITS);
    esp_ok = adc_oneshot_io_to_channel((int)pin, (adc_unit_t *)&adc_init_config[pin].unit_id, (adc_channel_t *)&adc_channel[pin]);
    if(esp_ok != ESP_OK){
      LOG("[ADC] Failed to map pin %d to ADC channel, err: %d", pin, esp_err_to_name(esp_ok));
      return -1;
    } else {
      // channel or ADC1 not yet initialized, so initialize
      esp_ok = adc_oneshot_new_unit(&adc_init_config[pin], &adc_handle[pin]);
      if(esp_ok == ESP_OK){
        // map the pin to ADC channel
        if(esp_ok != ESP_OK){
          LOG("[ADC] Failed to map pin %d to ADC channel, err: %d", pin, esp_err_to_name(esp_ok));
          return -1;
        }
        adc_oneshot_chan_cfg_t config = {
          .atten = ADC_ATTEN_DB_12,
          .bitwidth = ADC_BITS,
        };
        esp_ok = adc_oneshot_config_channel(adc_handle[pin], adc_channel[pin], &config);
        if(esp_ok != ESP_OK){
          LOG("[ADC] Failed to set ADC resolution to %d bits, err: %d", ADC_BITS, esp_err_to_name(esp_ok));
          return -1;
        }
        LOG("[ADC] Created new ADC handle %d, channel %d for pin %d", adc_handle[pin], adc_channel[pin], pin);
      } else {
        if(esp_ok != ESP_ERR_NOT_FOUND){
          LOG("[ADC] Failed to create ADC unit handle, err: %d", esp_err_to_name(esp_ok));
          return -1;
        } else {
          // copy over existing handle from another pin
          uint8_t found = 0;
          for(uint8_t i = 0; i < ADC1_PINS; i++){
            // find out whether the ADC1 channel is already initialized for another pin
            if((i != pin) && (setup_adc[i] == 1) && (adc_init_config[i].unit_id == adc_init_config[pin].unit_id)){
              adc_handle[pin] = adc_handle[i];
              LOG("[ADC] Re-used existing ADC handle %d, channel %d for pin %d", adc_handle[pin], adc_channel[pin], pin);
              found = 1;
              break;
            }
          }
          if(found == 0){
            LOG("[ADC] Failed to find existing ADC handle for channel %d for pin %d", adc_channel[pin], pin);
            return -1;
          }
        }
      }
    }
    setup_adc[pin] = 1;
  }

  // configure pin as input
  LOG("[ADC] set pin %d as input", pin);
  esp_ok = gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  if(esp_ok != ESP_OK){
    LOG("[ADC] Failed to set pin %d as input, err: %d", pin, esp_err_to_name(esp_ok));
    return -1;
  }

  // Explicitly disable internal pull resistors
  LOG("[ADC] disable internal pull resistors on pin %d", pin);
  esp_ok = gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
  if(esp_ok != ESP_OK){
    LOG("[ADC] Failed to disable internal pull resistors on pin %d, err: %d", pin, esp_err_to_name(esp_ok));
    return -1;
  }
  return 1;
}

NOINLINE
float get_adc_average(uint8_t samples, uint8_t pin){
  if(samples == 0)
    samples = 1;
  int avg_adc = 0.0;
  for(uint8_t i = 0; i < samples; i++) {
    int adc = 0.0;
    esp_ok = adc_oneshot_read(adc_handle[pin], adc_channel[pin], &adc);
    if(esp_ok != ESP_OK){
      LOG("[ADC] Failed to read ADC on pin %d, err: %d", pin, esp_err_to_name(esp_ok));
      continue;
    }
    avg_adc += adc;
    if(i < (samples - 1)){
      delayMicroseconds(10);
      doYIELD;
    }
  }
  float avg_adc_f = (float)avg_adc;
  avg_adc_f /= (float)samples;
  avg_adc_f /= (float)ADC_MAX;
  avg_adc_f *= ADC_MREF_VOLTAGE;
  D("[ADC] Average ADC value on pin %d: %f", pin, avg_adc_f);
  avg_adc_f /= 1000.0f; // convert mV to V
  return avg_adc_f;
}

NOINLINE
int8_t set_pin_high(uint8_t pin){
  esp_ok = gpio_set_level((gpio_num_t)pin, 1);
  if(esp_ok != ESP_OK){
    LOG("[ADC] Failed to set pin %d high, err: %d", pin, esp_err_to_name(esp_ok));
    return -1;
  }
  ets_delay_us(50);
  return 1;
}

NOINLINE
int8_t set_pin_low(uint8_t pin){
  esp_ok = gpio_set_level((gpio_num_t)pin, 0);
  if(esp_ok != ESP_OK){
    LOG("[ADC] Failed to set pin %d low, err: %d", pin, esp_err_to_name(esp_ok));
    return -1;
  }
  return 1;
}

NOINLINE
int8_t initialize_pin_out(uint8_t pin){
  esp_ok = gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
  if(esp_ok != ESP_OK)
    return -1;

  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << pin),         // Bit mask for the pin
    .mode         = GPIO_MODE_OUTPUT,      // Set as output
    .pull_up_en   = GPIO_PULLUP_DISABLE,   // Disable internal pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable internal pull-down
    .intr_type    = GPIO_INTR_DISABLE      // Disable interrupts
  };
 
  esp_ok = gpio_config(&io_conf);
  if(esp_ok != ESP_OK)
    return -1;
  return 1;
}

#endif

#if defined(SUPPORT_SE95) || defined(SUPPORT_BME280) || defined(SUPPORT_BMP280) || defined(SUPPORT_APDS9930) || defined(SUPPORT_BH1750)
// re-export Wire for sensors.cpp in this SENSORS namespace, note that "Wire"
// is a global extern object
TwoWire Wire = TwoWire(I2C_BUS_NUM);
uint8_t i2c_initialized = 0; // 0=not initialized, 1=initialized, 2=failed

NOINLINE
int8_t i2c_initialize(){
  if(i2c_initialized == 1)
    return 1;  // already initialized
  if(i2c_initialized == 2)
    return -1; // failed previously
  if(!Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ)){
    LOG("[SENSORS] I2C hardware init failed!");
    i2c_initialized = 2; // failed
    return -1;
  }
  Wire.setTimeout(I2C_TIMEOUT);
  LOG("[SENSORS] I2C initialized on SDA: %d, SCL: %d", I2C_SDA, I2C_SCL);
  i2c_initialized = 1;
  return 1;
}

NOINLINE
int8_t i2c_ping(uint8_t addr) {
  if(i2c_initialize() == -1)
    return -1;
  Wire.beginTransmission(addr);
  if(Wire.endTransmission() != ESP_OK){
    LOG("[I2C] Ping to address 0x%02X failed", addr);
    return -1;
  }
  return 1;
}

NOINLINE
int8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t value){
  if(i2c_initialize() == -1)
    return -1;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  if(Wire.endTransmission() != ESP_OK)
    return -1;
  return 1;
}

NOINLINE
int8_t i2c_write(uint8_t addr, uint8_t value){
  if(i2c_initialize() == -1)
    return -1;
  Wire.beginTransmission(addr);
  Wire.write(value);
  if(Wire.endTransmission() != ESP_OK)
    return -1;
  return 1;
}

NOINLINE
int8_t i2c_write16(uint8_t addr, uint8_t reg, uint16_t value){
  if(i2c_initialize() == -1)
    return -1;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // MSB
  Wire.write(value & 0xFF);        // LSB
  if(Wire.endTransmission() != ESP_OK)
    return -1;
  return 1;
}

NOINLINE
uint8_t i2c_read8(uint8_t addr, uint8_t reg){
  if(i2c_initialize() == -1)
    return 0xFF;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission() != ESP_OK)
    return 0xFF;
  Wire.requestFrom(addr, 1);
  if(Wire.available())
    return Wire.read();
  else
    return 0xFF;
}

NOINLINE
uint16_t i2c_read16(uint8_t addr, uint8_t reg, bool big_endian){
  if(i2c_initialize() == -1)
    return 0xFFFF;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission() != ESP_OK)
    return 0xFFFF;
  Wire.requestFrom(addr, 2);
  uint8_t b1, b2;
  // first byte
  if(Wire.available())
    b1 = Wire.read();
  else
    return 0xFFFF;
  // second byte
  if(Wire.available())
    b2 = Wire.read();
  else
    return 0xFFFF;
  if(big_endian)
    return ((b1 << 8) | b2);
  else
    return ((b2 << 8) | b1);
}

NOINLINE
uint16_t i2c_read16raw(uint8_t addr) {
  if(i2c_initialize() == -1)
    return 0xFFFF;

  // No Wire.beginTransmission/write needed here!
  // Just request the 2 bytes directly.
  uint8_t count = Wire.requestFrom(addr, (uint8_t)2);
  if(count != 2)
    return 0xFFFF;

  uint8_t b1 = Wire.read(); // High Byte
  uint8_t b2 = Wire.read(); // Low Byte

  return ((uint16_t)b1 << 8) | b2;
}

// read 16-bit big-endian
NOINLINE
uint16_t i2c_read16be(uint8_t addr, uint8_t reg){
  return i2c_read16(addr, reg, true);
}

// read 16-bit little-endian
NOINLINE
uint16_t i2c_read16le(uint8_t addr, uint8_t reg){
  return i2c_read16(addr, reg, false);
}


#endif

#define CFG_PARTITION     "esp-at"
#define CFG_NAMESPACE     "sensors"
#define CFG_STORAGE       "main"

nvs_handle_t nvs_handle;

NOINLINE
void CFG_SAVE_NS(const char *st, void *p, size_t s) {
  LOG("[CFG] Saving config for key '%s' to NVS", st);
  if(p == NULL || s == 0)
    return;
  CFG::SAVE_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE, st, p, s);
  return;
}

NOINLINE
void CFG_SAVE() {
  LOG("[CFG] Saving config to NVS: '%s', '%s'", CFG_PARTITION, CFG_NAMESPACE);
  CFG::SAVE_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE, CFG_STORAGE, (void *)&SENSORS::cfg, sizeof(SENSORS::cfg));
  return;
}

NOINLINE
void CFG_INIT() {
  LOG("[CFG] Initializing NVS storage for '%s', '%s'", CFG_PARTITION, CFG_NAMESPACE);
  if(!CFG::INIT_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE)) {
    LOG("[CFG] Failed to initialize NVS storage for '%s', '%s': %s", CFG_PARTITION, CFG_NAMESPACE, esp_err_to_name(esp_ok));
    ESP.restart();
  } else {
    LOG("[CFG] NVS storage initialized for '%s', '%s'", CFG_PARTITION, CFG_NAMESPACE);
  }
  return;
}

NOINLINE
void CFG_LOAD(const char *st, void *p, size_t s) {
    LOG("[CFG] Loading config for key '%s' from NVS", st);
    CFG::LOAD_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE, st, p, s);
    return;
}

NOINLINE
void CFG_CLEAR(const char *st) {
    LOG("[CFG] Clearing config for key '%s' from NVS", (st == NULL) ? "ALL_KEYS" : st);
    if(st == NULL){
      for(uint8_t i = 0; i < NR_OF_SENSORS; i++)
        CFG::CLEAR_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE, SENSORS::all_sensors[i].key);
      return;
    }
    CFG::CLEAR_H(&nvs_handle, CFG_PARTITION, CFG_NAMESPACE, st);
    return;
}

NOINLINE
char* at_cmd_check(const char *cmd, const char *at_cmd, unsigned short at_len) {
  unsigned short l = strlen(cmd); /* AT+<cmd>=, or AT, or AT+<cmd>? */
  if(at_len >= l && strncmp(cmd, at_cmd, l) == 0) {
    if(*(cmd+l-1) == '=') {
      return (char *)at_cmd+l;
    } else {
      return (char *)at_cmd;
    }
  }
  return NULL;
}


// DHT11 Sensor
#define SENSOR_DHT11_HUMIDITY    {.name = "DHT11 Humidity", .key = "humidity",}
#define SENSOR_DHT11_TEMPERATURE {.name = "DHT11 Temperature", .key = "temperature",}
#ifdef SUPPORT_DHT11
#define SENSOR_DHT11_HUMIDITY \
    {\
      .name = "DHT11 Humidity",\
      .key  = "dht11_humidity",\
      .unit_fmt = "%%,%.0f",\
      .init_function = init_dht11,\
      .value_function = dht11_fetch_humidity,\
    }
#define SENSOR_DHT11_TEMPERATURE \
    {\
      .name = "DHT11 Temperature",\
      .key  = "dht11_temperature",\
      .unit_fmt = "°C,%.2f",\
      .init_function = init_dht11,\
      .value_function = dht11_fetch_temperature,\
    }

#define DODHT(s) ((DHT*)(s->userdata))

#define DHTPIN  A0     // GPIO_NUM_0/A0 pin for DHT11

// last humidity for MQ135 compensation
RTC_DATA_ATTR float last_humidity = 0.0f;

int8_t dht11_fetch_humidity(sensor_r_t *s, float *humidity){
  if(humidity == NULL)
    return -1;
  // fetch humidity from DHT11
  float h = (float)DODHT(s)->readHumidity(false);
  D("[DHT11] humidity: %f %%", h);
  if(isnan(h) || h < 0.0f || h > 100.0f){
    LOG("[DHT11] humidity invalid or out of range, returning 0: %.2f", h);
    *humidity = 0.0f;
    return -1;
  }
  *humidity = h;

  // for MQ135 air quality sensor compensation
  last_humidity = h;
  return 1;
}

int8_t dht11_fetch_temperature(sensor_r_t *s, float *temperature){
  if(temperature == NULL)
    return -1;
  // fetch temperature from DHT11
  float t = (float)DODHT(s)->readTemperature(false);
  D("[DHT11] temperature: %f °C", t);
  if(isnan(t) || t < 0.0f || t > 50.0f){
    LOG("[DHT11] temperature invalid or out of range, returning 0: %.2f", t);
    *temperature = 0.0f;
    return -1;
  }
  *temperature = t;
  return 1;
}

void init_dht11(sensor_r_t *s){
  // initialize DHT11 sensor
  s->userdata = new DHT(DHTPIN, DHT11);
  DODHT(s)->begin();
  LOG("[DHT11] initialized on pin %d", DHTPIN);
}

#endif // SUPPORT_DHT11

// BME280 Sensor
#define SENSOR_BME280_HUMIDITY    {.name = "BME280 Humidity", .key = "bme280_humidity",}
#define SENSOR_BME280_TEMPERATURE {.name = "BME280 Temperature", .key = "bme280_temperature",}
#define SENSOR_BME280_PRESSURE    {.name = "BME280 Pressure", .key = "bme280_pressure",}
#ifdef SUPPORT_BME280
#define SENSOR_BME280_HUMIDITY \
    {\
      .name = "BME280 Humidity",\
      .key  = "bme280_humidity",\
      .unit_fmt = "%%,%.0f",\
      .init_function = init_bme280,\
      .pre_function = pre_bme280,\
      .value_function = bme280_fetch_humidity,\
      .post_function = post_bme280,\
    }
#define SENSOR_BME280_TEMPERATURE \
    {\
      .name = "BME280 Temperature",\
      .key  = "bme280_temperature",\
      .unit_fmt = "°C,%.2f",\
      .init_function = init_bme280,\
      .pre_function = pre_bme280,\
      .value_function = bme280_fetch_temperature,\
      .post_function = post_bme280,\
    }
#define SENSOR_BME280_PRESSURE \
    {\
      .name = "BME280 Pressure",\
      .key  = "bme280_pressure",\
      .unit_fmt = "hPa,%.2f",\
      .init_function = init_bme280,\
      .pre_function = pre_bme280,\
      .value_function = bme280_fetch_pressure,\
      .post_function = post_bme280,\
    }

#define BME280_I2C_ADDRESS 0x76
#define BME280_ID_REG      0xD0

// Trimming parameters
struct {
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} cal;

RTC_DATA_ATTR float last_bme280_humidity = 0.0f;
RTC_DATA_ATTR float last_bme280_temperature = 0.0f;
RTC_DATA_ATTR float last_bme280_pressure = 0.0f;

int8_t bme280_fetch_humidity(sensor_r_t *s, float *humidity){
  if(humidity == NULL)
    return -1;
  float humidity_raw = 0.0f;

  LOG("[BME280] humidity: %f %%", humidity_raw);
  if(isnan(humidity_raw) || humidity_raw < 0.0f || humidity_raw > 100.0f){
    LOG("[BME280] humidity invalid or out of range: %.2f", humidity_raw);
    *humidity = 0.0f;
    return -1;
  }
  *humidity = humidity_raw;
  last_bme280_humidity = humidity_raw;
  return 1;
}

int8_t bme280_fetch_temperature(sensor_r_t *s, float *temperature){
  if(temperature == NULL)
    return -1;
  float temperature_raw = 0.0f;

  LOG("[BME280] temperature: %f °C", temperature_raw);
  if(isnan(temperature_raw)){
    LOG("[BME280] temperature invalid or out of range: %.2f", temperature_raw);
    *temperature = 0.0f;
    return -1;
  }
  *temperature = temperature_raw;
  last_bme280_temperature = temperature_raw;
  return 1;
}

int8_t bme280_fetch_pressure(sensor_r_t *s, float *pressure){
  if(pressure == NULL)
    return -1;
  float pressure_raw = 0.0f;

  LOG("[BME280] pressure: %f hPa", pressure_raw);
  if(isnan(pressure_raw)){
    LOG("[BME280] pressure invalid or out of range: %.2f", pressure_raw);
    *pressure = 0.0f;
    return -1;
  }
  *pressure = pressure_raw;
  last_bme280_pressure = pressure_raw;
  return 1;
}

void pre_bme280(sensor_r_t *s){
}

void post_bme280(sensor_r_t *s){
}

void init_bme280(sensor_r_t *s){
  if(i2c_ping(BME280_I2C_ADDRESS) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[BME280] sensor not found on I2C address 0x%02X", BME280_I2C_ADDRESS);
    return;
  }

  // read ID
  uint8_t id = i2c_read8(BME280_I2C_ADDRESS, BME280_ID_REG) & 0xFF;
  if(id != 0x60){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[BME280] sensor not found, read id: 0x%02X", id);
    return;
  }
  LOG("[BME280] initialized on I2C address 0x%02X, id: 0x%02X", BME280_I2C_ADDRESS, id);

  // read Calibration Data
  cal.dig_T1 = i2c_read16be(BME280_I2C_ADDRESS, 0x88);
  cal.dig_T2 = i2c_read16be(BME280_I2C_ADDRESS, 0x8A);
  cal.dig_T3 = i2c_read16be(BME280_I2C_ADDRESS, 0x8C);
  cal.dig_P1 = i2c_read16be(BME280_I2C_ADDRESS, 0x8E);
  cal.dig_P2 = i2c_read16be(BME280_I2C_ADDRESS, 0x90);
  cal.dig_P3 = i2c_read16be(BME280_I2C_ADDRESS, 0x92);
  cal.dig_P4 = i2c_read16be(BME280_I2C_ADDRESS, 0x94);
  cal.dig_P5 = i2c_read16be(BME280_I2C_ADDRESS, 0x96);
  cal.dig_P6 = i2c_read16be(BME280_I2C_ADDRESS, 0x98);
  cal.dig_P7 = i2c_read16be(BME280_I2C_ADDRESS, 0x9A);
  cal.dig_P8 = i2c_read16be(BME280_I2C_ADDRESS, 0x9C);
  cal.dig_P9 = i2c_read16be(BME280_I2C_ADDRESS, 0x9E);

  // configure sensor
  // ctrl_meas, Temperature oversampling x2, Pressure x16, Normal mode
  if(i2c_write(BME280_I2C_ADDRESS, 0xF4, 0x57) == -1)
    LOG("[BME280] failed to write ctrl_meas");
}

#endif // SUPPORT_BME280

/*
  KY-018 LDR Lux Meter also wrong pinout, should be:
  Connections:
  S (Signal) -> GND
  Middle (VCC) -> 5V
  - (GND) -> Signal to ADC pin (A1)
*/
// LDR Sensor
#define SENSOR_LDR {.name = "LDR Illuminance", .key = "ldr_illuminance",}
#ifdef SUPPORT_LDR
#define SENSOR_LDR \
    {\
      .name = "LDR Illuminance",\
      .key  = "ldr_illuminance",\
      .unit_fmt = "lx,%.0f",\
      .init_function = init_ldr_adc,\
      .value_function = fetch_ldr_adc,\
    }

#define LDRADCPIN    A1          // GPIO_NUM_1/A1 pin for LDR, same as NTCADCPIN, don't use together
#define LDRVCCPIN    GPIO_NUM_10 // GPIO_NUM_10 pin to power the LDR

#define LDR_KEY      "ldr"
#define LDR_CFG(k)   SENSORS::ldr_cfg.k
#define LDR_SAVE()   CFG_SAVE_NS(LDR_KEY, (void *)&SENSORS::ldr_cfg, sizeof(SENSORS::ldr_cfg));

typedef struct ldr_cfg_t {
  float vcc           = 3.3f;     // Vcc for the voltage divider
  float divider_r     = 10000.0f; // 10k ohm divider resistor
  float ema_alpha     = 0.2f;     // Smoothing factor (0.1 to 0.3 is typical)
  float r10           = 10000.0f; // Reference resistance at 1 lux
  float gamma         = 0.7f;     // Gamma value for the LDR
} ldr_cfg_t;
ALIGN(4) ldr_cfg_t ldr_cfg = {
  .vcc         = 3.3f,
  .divider_r   = 10300.0f,
  .ema_alpha   = 0.2f,
  .r10         = 10000.0f,
  .gamma       = 0.7f,
};

int8_t fetch_ldr_adc(sensor_r_t *s, float *ldr_value){
  if(ldr_value == NULL)
    return -1;

  // Set LDR Vcc pin HIGH to power the LDR
  if(set_pin_high(LDRVCCPIN) == -1)
    return -1;
  
  // fetch ADC value
  float v_out = get_adc_average(10, LDRADCPIN);
  D("[LDR] value: %.5f V", v_out);

  if(set_pin_low(LDRVCCPIN) == -1)
    return -1;

  // use the LDR_DIVIDER_R and Vcc to convert ADC voltage to illuminance in lux
  // Standard wiring: R_divider is to VCC, LDR is to GND
  // R_ntc = R_fixed * (V_out / (Vcc - V_out))
  float r_ldr = LDR_CFG(divider_r) * (v_out / (LDR_CFG(vcc) - v_out));
  D("[LDR] V_out: %.5f, R_ldr: %.2f Ohms", v_out, r_ldr);

  // Standard LDR formula: Lux = A * R_LDR ^ B
  // float lux = LDR_A / (r_ldr / LDR_B);
  // Alternative formula with Gamma correction
  float lux = 10.0 * pow((LDR_CFG(r10) / r_ldr), (1.0 / LDR_CFG(gamma)));

  *ldr_value = lux;
  return 1;
}

float calibrate_ldr_r10_gamma(float known_lux){
  // Calibrate R10 and GAMMA based on known lux and measured lux
  // known_lux = 10 * (R10 / R_ldr)^(1/GAMMA)
  // Rearranged to find R10:
  // R10 = R_ldr * (known_lux / 10) ^ GAMMA
  float v_out = 0.0f;
  if(set_pin_high(LDRVCCPIN) == -1){
    LOG("[LDR] Failed to set Vcc pin high for calibration");
  } else {
    v_out = get_adc_average(10, LDRADCPIN);
    if(set_pin_low(LDRVCCPIN) == -1){
      LOG("[LDR] Failed to set Vcc pin low after calibration");
    }
  }
  float r_ldr = LDR_CFG(divider_r) * (v_out / (LDR_CFG(vcc) - v_out));
  float new_r10 = r_ldr * pow((known_lux / 10.0f), LDR_CFG(gamma));
  LOG("[LDR] Calibrated R10: %.2f Ohms based on lux: %.2f, using Rldr: %f, Rdivider: %f, Vcc: %f, Vldr: %f, gamma: %f", new_r10, known_lux, r_ldr, LDR_CFG(divider_r), LDR_CFG(vcc), v_out, LDR_CFG(gamma));
  return new_r10;
}

void init_ldr_adc(sensor_r_t *s){
  // load LDR config
  CFG_LOAD(LDR_KEY, (void *)&ldr_cfg, sizeof(ldr_cfg));

  // initialize ADC for LDR
  if(initialize_adc(LDRADCPIN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[LDR] Failed to initialize ADC on pin %d", LDRADCPIN);
    return;
  }

  // initialize the Vcc GPIO pin out to power the LDR
  LOG("[LDR] set pin %d as Vcc OUT", LDRVCCPIN);
  if(initialize_pin_out(LDRVCCPIN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[LDR] Failed to set pin %d as Vcc OUT", LDRVCCPIN);
    return;
  }
  LOG("[LDR] initialized on pin %d", LDRADCPIN);
}

NOINLINE
const char *atcmd_sensors_ldr(const char *atcmdline, unsigned short cmd_len) {
  char *p = NULL;
  if(p = at_cmd_check("AT+LDR_VCC?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(LDR_CFG(vcc));
  } else if(p = at_cmd_check("AT+LDR_VCC=", atcmdline, cmd_len)){
    float new_vcc = strtof(p, NULL);
    if(new_vcc < 0.0f || new_vcc > 5.0f)
      return AT_R("+ERROR: invalid VCC value 0-5 V");
    LOG("[LDR] Setting VCC to %f V", new_vcc);
    LDR_CFG(vcc) = new_vcc;
    LDR_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LDR_DIVIDER_R?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(LDR_CFG(divider_r));
  } else if(p = at_cmd_check("AT+LDR_DIVIDER_R=", atcmdline, cmd_len)){
    float new_r = strtof(p, NULL);
    if(new_r < 0.0f)
      return AT_R("+ERROR: invalid divider resistance value");
    LOG("[LDR] Setting divider resistance to %f Ohm", new_r);
    LDR_CFG(divider_r) = new_r;
    LDR_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LDR_EMA_ALPHA?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(LDR_CFG(ema_alpha));
  } else if(p = at_cmd_check("AT+LDR_EMA_ALPHA=", atcmdline, cmd_len)){
    float new_ema_alpha = strtof(p, NULL);
    if(new_ema_alpha < 0.0f || new_ema_alpha > 1.0f)
      return AT_R("+ERROR: invalid EMA alpha value 0-1");
    LOG("[LDR] Setting EMA alpha to %f", new_ema_alpha);
    LDR_CFG(ema_alpha) = new_ema_alpha;
    LDR_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LDR_CALIBRATE_LUX=", atcmdline, cmd_len)){
    float known_lux = strtof(p, NULL);
    if(known_lux <= 0.0f)
      return AT_R("+ERROR: invalid known lux value");
    LOG("[LDR] Calibrating LDR with known lux value %f lx", known_lux);
    float r_ldr_r10 = calibrate_ldr_r10_gamma(known_lux);
    if(r_ldr_r10 < 0.0f)
      return AT_R("+ERROR: failed to calibrate LDR");
    LDR_CFG(r10) = r_ldr_r10;
    LDR_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LDR_R10?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(LDR_CFG(r10));
  } else if(p = at_cmd_check("AT+LDR_R10=", atcmdline, cmd_len)){
    float new_r10 = strtof(p, NULL);
    if(new_r10 <= 0.0f)
      return AT_R("+ERROR: invalid R10 value");
    LOG("[LDR] Setting R10 to %f Ohm", new_r10);
    LDR_CFG(r10) = new_r10;
    LDR_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LDR_GAMMA?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(LDR_CFG(gamma));
  } else if(p = at_cmd_check("AT+LDR_GAMMA=", atcmdline, cmd_len)){
    float new_gamma = strtof(p, NULL);
    if(new_gamma <= 0.0f)
      return AT_R("+ERROR: invalid gamma value");
    LOG("[LDR] Setting gamma to %f", new_gamma);
    LDR_CFG(gamma) = new_gamma;
    LDR_SAVE();
    return AT_R_OK;
  }
  return AT_R("+ERROR: unknown command");
}
#endif // SUPPORT_LDR

// NTC Sensor
// This is actually the KY-013 NTC Thermistor module, see https://sensorkit.joy-it.net/en/sensors/ky-013
// but: https://forum.arduino.cc/t/solved-ky-013-analog-temperature-sensor-incorrect-readings/203719
// errornous pinout, should be: left to right: GND (label S) | VCC | Signal (to ADC) (labelled -)
#define SENSOR_NTC_TEMPERATURE {.name = "NTC Temperature", .key = "ntc_temperature",}
#ifdef SUPPORT_NTC
#define SENSOR_NTC_TEMPERATURE \
    {\
      .name = "NTC Temperature",\
      .key  = "ntc_temperature",\
      .unit_fmt = "°C,%.4f",\
      .init_function = init_ntc_adc,\
      .value_function = fetch_ntc_temperature,\
    }

#define NTCADCPIN    A1          // GPIO_NUM_1/A1 pin for NTC, same as LDRADCPIN, don't use together
#define NTCVCCPIN    GPIO_NUM_10 // GPIO_NUM_10 pin to power the NTC

#define NTC_KEY      "ntc"
#define NTC_CFG(k)   SENSORS::ntc_cfg.k
#define NTC_SAVE()   CFG_SAVE_NS(NTC_KEY, (void *)&SENSORS::ntc_cfg, sizeof(SENSORS::ntc_cfg));

typedef struct ntc_cfg_t {
  float vcc           = 3.3f;     // Vcc for the voltage divider
  float divider_r     = 10250.0f; // 10k ohm divider resistor
  float beta          = 3950.0f;  // Beta coefficient of the NTC
  float r_nominal     = 10000.0f; // Nominal resistance at 25 C
  float t_nominal     = 25.0f;    // 25 C
  float ema_alpha     = 0.2f;     // Smoothing factor (0.1 to 0.3 is typical)
} ntc_cfg_t;
ALIGN(4) ntc_cfg_t ntc_cfg = {
  .vcc         = 3.3f,
  .divider_r   = 10250.0f,
  .beta        = 3950.0f,
  .r_nominal   = 10000.0f,
  .t_nominal   = 25.0f,
  .ema_alpha   = 0.2f,
};

int8_t fetch_ntc_temperature(sensor_r_t *s, float *temperature){
  if(temperature == NULL)
    return -1;

  // Set NTC Vcc pin HIGH to power the NTC
  if(set_pin_high(NTCVCCPIN) == -1)
    return -1;

  // Get average voltage in Volts (as float)
  float v_out = get_adc_average(10, NTCADCPIN);
  if(v_out >= NTC_CFG(vcc))
    v_out = NTC_CFG(vcc) - 0.0001f; // avoid division by zero

  // Set NTC Vcc pin LOW to save power and avoid heating the NTC
  if(set_pin_low(NTCVCCPIN) == -1)
    return -1;
  
  // Standard KY-013 wiring: R_divider is to VCC, NTC is to GND
  // R_ntc = R_fixed * (V_out / (Vcc - V_out))
  float r_ntc = NTC_CFG(divider_r) * (v_out / (NTC_CFG(vcc) - v_out));
  D("[NTC] V_out: %.2f, R_ntc: %.2f Ohms", v_out, r_ntc);

  // Steinhart-Hart / B-parameter
  float steinhart;
  steinhart  = log(r_ntc / NTC_CFG(r_nominal));
  steinhart /= NTC_CFG(beta);
  steinhart += 1.0f / (NTC_CFG(t_nominal) + 273.15f);
  steinhart  = (1.0f / steinhart) - 273.15f;

  D("[NTC] temperature: %.2f °C", steinhart);

  // simple low-pass filter to smooth the temperature readings using exponential moving average (EMA)
  static float filtered_temp = -100.0f; 
  if (filtered_temp < -90.0f) {
    filtered_temp = steinhart; // Initial reading
  } else {
    filtered_temp = (NTC_CFG(ema_alpha) * steinhart) + (1.0f - NTC_CFG(ema_alpha)) * filtered_temp;
  }

  // Return the filtered temperature
  *temperature = filtered_temp;
  return 1;
}

void init_ntc_adc(sensor_r_t *s){
  // load NTC config
  CFG_LOAD(NTC_KEY, (void *)&ntc_cfg, sizeof(ntc_cfg));

  // initialize ADC for NTC
  if(initialize_adc(NTCADCPIN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[NTC] Failed to initialize ADC on pin %d", NTCADCPIN);
    return;
  }

  // initialize the Vcc GPIO pin out to power the NTC
  LOG("[NTS] set pin %d as Vcc OUT", NTCVCCPIN);
  if(initialize_pin_out(NTCVCCPIN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[NTC] Failed to set pin %d as Vcc OUT", NTCVCCPIN);
    return;
  }
  LOG("[NTC] initialized on pin %d", NTCADCPIN);
}

NOINLINE
const char *atcmd_sensors_ntc(const char *atcmdline, size_t cmd_len){
  char *p = NULL;
  if(p = at_cmd_check("AT+NTC_VCC?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(vcc));
  } else if(p = at_cmd_check("AT+NTC_VCC=", atcmdline, cmd_len)){
    float new_vcc = strtof(p, NULL);
    if(new_vcc < 0.0f || new_vcc > 5.0f)
      return AT_R("+ERROR: invalid VCC value 0-5 V");
    LOG("[NTC] Setting VCC to %f V", new_vcc);
    NTC_CFG(vcc) = new_vcc;
    NTC_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTC_DIVIDER_R?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(divider_r));
  } else if(p = at_cmd_check("AT+NTC_DIVIDER_R=", atcmdline, cmd_len)){
    float new_r = strtof(p, NULL);
    if(new_r < 0.0f)
      return AT_R("+ERROR: invalid divider resistance value");
    LOG("[NTC] Setting divider resistance to %f Ohm", new_r);
    NTC_CFG(divider_r) = new_r;
    NTC_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTC_BETA?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(beta));
  } else if(p = at_cmd_check("AT+NTC_BETA=", atcmdline, cmd_len)){
    float new_beta = strtof(p, NULL);
    if(new_beta < 0.0f)
      return AT_R("+ERROR: invalid beta value");
    LOG("[NTC] Setting beta to %f", new_beta);
    NTC_CFG(beta) = new_beta;
    NTC_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTC_R_NOMINAL?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(r_nominal));
  } else if(p = at_cmd_check("AT+NTC_R_NOMINAL=", atcmdline, cmd_len)){
    float new_r_nominal = strtof(p, NULL);
    if(new_r_nominal < 0.0f)
      return AT_R("+ERROR: invalid nominal resistance value");
    LOG("[NTC] Setting nominal resistance to %f Ohm", new_r_nominal);
    NTC_CFG(r_nominal) = new_r_nominal;
    NTC_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTC_T_NOMINAL?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(t_nominal));
  } else if(p = at_cmd_check("AT+NTC_T_NOMINAL=", atcmdline, cmd_len)){
    float new_t_nominal = strtof(p, NULL);
    LOG("[NTC] Setting nominal temperature to %f C", new_t_nominal);
    NTC_CFG(t_nominal) = new_t_nominal;
    NTC_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTC_EMA_ALPHA?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(NTC_CFG(ema_alpha));
  } else if(p = at_cmd_check("AT+NTC_EMA_ALPHA=", atcmdline, cmd_len)){
    float new_ema_alpha = strtof(p, NULL);
    if(new_ema_alpha < 0.0f || new_ema_alpha > 1.0f)
      return AT_R("+ERROR: invalid EMA alpha value 0-1");
    LOG("[NTC] Setting EMA alpha to %f", new_ema_alpha);
    NTC_CFG(ema_alpha) = new_ema_alpha;
    NTC_SAVE();
    return AT_R_OK;
  }
  return AT_R("+ERROR: unknown command");
}
#endif // SUPPORT_NTC

// MQ-135 Air Quality Sensor
#define SENSOR_MQ135 {.name = "MQ-135 Air Quality", .key = "mq135_air_quality",}
#ifdef SUPPORT_MQ135
#define SENSOR_MQ135 \
    {\
      .name = "MQ-135 Air Quality",\
      .key  = "mq135_air_quality",\
      .unit_fmt = "ppm,%.0f",\
      .init_function = init_mq135_adc,\
      .value_function = fetch_mq135_adc,\
      .destroy_function = destroy_mq135_adc,\
    }

#define MQ135PIN           A2  // GPIO_NUM_2/A2 pin for MQ-135
#define MQ135_AVG_NR        50 // number of samples to average from ADC, higher = more stable, don't go too high or uint32_t overflow

#define MQ135_WARMUP_TIME    30000  // MQ-135 warm-up time in ms

#define MQ135_KEY      "mq135"
#define MQ135_CFG(k)   SENSORS::mq135_cfg.k
#define MQ135_SAVE()   CFG_SAVE_NS(MQ135_KEY, (void *)&SENSORS::mq135_cfg, sizeof(SENSORS::mq135_cfg));

#ifdef SUPPORT_S8
extern int8_t fetch_s8_co2(sensor_r_t *s, float *co2_ppm);
#endif

typedef struct mq135_cfg_t {
  float r0            = 0.0f;
  float rl            = 0.0f;
  float reference_ppm = 428.54f;
  float a             = 110.47f;
  float b             = -2.862f;
  float vcc           = 5.0f;
} mq135_cfg_t;
ALIGN(4) mq135_cfg_t mq135_cfg = {
  .r0                 = 0.0f,    // default R0 for MQ-135
  .rl                 = 0.0f,    // default RL for MQ-135
  .reference_ppm      = 428.54f, // default ppm for calibration
  .a                  = 110.47f, // default curve coefficient a (default for CO2)
  .b                  = -2.862f, // default curve coefficient b (default for CO2)
  .vcc                = 5.0f,    // default VCC voltage
};

RTC_DATA_ATTR unsigned long mq135_startup_time = 0;

float mq135_adc_to_ppm(float mq135_r0, float mq135_rl, float adc_value) {
  float RS = ((MQ135_CFG(vcc) - adc_value) / adc_value ) * mq135_rl; // in kOhm

  #if defined(SUPPORT_DHT11) && defined(SUPPORT_DS18B20)
  // apply a correction based on temperature and humidity if available
  if(last_read_time != 0 && millis() - last_read_time < 60000){ // valid DHT11 reading within last 60s
    if (last_temperature == 0.0f)
      last_temperature = 25.0f; // default to 25C if no temperature available
    float cf = 0.00035f * pow(last_temperature, 2) - 0.019f * last_temperature + 1.224f;
    if(last_humidity != 0.0f)
      cf += (last_humidity - 33.0f) * -0.0018f;
    RS /= cf;
  }
  #endif // SUPPORT_DHT11

  float RATIO = RS / mq135_r0;
  float ppm = MQ135_CFG(a) * pow(RATIO, MQ135_CFG(b));
  D("[MQ-135] PPM: %f, Value: %f V, R0: %f kOhm, RL: %f kOhm, RS: %f kOhm, R: %f", ppm, adc_value, mq135_r0, mq135_rl, RS, RATIO);
  return ppm;
}

NOINLINE
void calibrate_mq135_r0() {
  // fetch average value
  float adc_value = get_adc_average(MQ135_AVG_NR, MQ135PIN);
  LOG("[MQ-135] Calibration value: %f V", adc_value);

  // Voltage divider, in kOhm
  float RS = ((MQ135_CFG(vcc) - adc_value) / adc_value ) * MQ135_CFG(rl);

  #if defined(SUPPORT_DHT11) && defined(SUPPORT_DS18B20)
  // apply a correction based on temperature and humidity if available
  if(last_read_time != 0 && millis() - last_read_time < 60000){ // valid DHT11 reading within last 60s
    if (last_temperature == 0.0f)
      last_temperature = 25.0f; // default to 25C if no temperature available
    float cf = 0.00035f * pow(last_temperature, 2) - 0.019f * last_temperature + 1.224f;
    if(last_humidity != 0.0f)
      cf += (last_humidity - 33.0f) * -0.0018f;
    RS /= cf;
  }
  #endif // SUPPORT_DHT11

  float reference_ppm = MQ135_CFG(reference_ppm);
  if(reference_ppm <= 0.0f){
    LOG("[MQ-135] invalid ppm value: %f", reference_ppm);
    return;
  }

  // If we have an S8 SensAir, use that value as reference
  #ifdef SUPPORT_S8
  float s8_co2_ppm = 0.0f;
  int8_t ok = fetch_s8_co2(NULL, &s8_co2_ppm);
  if(ok == 1){
    LOG("[MQ-135] Using S8 SensAir CO2 ppm value %f as reference for calibration", s8_co2_ppm);
    reference_ppm = s8_co2_ppm;
  }
  #endif

  float R0 = RS / pow((reference_ppm / MQ135_CFG(a)), (1.0f / MQ135_CFG(b)));
  LOG("[MQ-135] Calibration value: Voltage: %f V, RL: %f kOhm, RS: %f kOhm, R0: %f kOhm", adc_value, MQ135_CFG(rl), RS, R0);
  MQ135_CFG(r0) = R0;
  return;
}

int8_t fetch_mq135_adc(sensor_r_t *s, float *ppm){
  if(ppm == NULL)
    return -1;
  if(millis() - mq135_startup_time < MQ135_WARMUP_TIME){
    LOG("[MQ-135] sensor warming up, not ready yet, ttl: %d ms", MQ135_WARMUP_TIME - (millis() - mq135_startup_time));
    return -1; // sensor warming up
  }

  // fetch average value
  float avg_adc = get_adc_average(MQ135_AVG_NR, MQ135PIN);
  D("[MQ-135] value: %f V", avg_adc);

  // convert to PPM using MQ-135 formula R0/RL and curve
  float R0 = MQ135_CFG(r0);
  float RL = MQ135_CFG(rl);
  *ppm = mq135_adc_to_ppm(R0, RL, avg_adc);
  return 1;
}

void init_mq135_adc(sensor_r_t *s){
  // Load MQ-135 config
  CFG_LOAD(MQ135_KEY, (void *)&mq135_cfg, sizeof(mq135_cfg));

  // initialize ADC for MQ-135
  if(initialize_adc(MQ135PIN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[MQ-135] Failed to initialize ADC on pin %d", MQ135PIN);
    return;
  }

  // read initial R0 and RL from config
  float R0 = MQ135_CFG(r0);
  float RL = MQ135_CFG(rl);
  D("[MQ-135] initialized on pin %d, R0: %0.f Ohm, RL: %0.f", R0, RL);

  // wait for MQ-135 to stabilize
  if(mq135_startup_time == 0)
    mq135_startup_time = millis();
}

NOINLINE
const char *atcmd_sensors_mq135(const char *atcmdline, size_t cmd_len){
  char *p = NULL;
  if(p = at_cmd_check("AT+MQ135_R0?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(r0));
  } else if(p = at_cmd_check("AT+MQ135_CALIBRATE", atcmdline, cmd_len)){
    if(MQ135_CFG(rl) <= 0.0f)
      return AT_R("+ERROR: invalid MQ135 RL value, set RL first");
    if(millis() - mq135_startup_time < MQ135_WARMUP_TIME){
      char ttl_msg[100] = {0};
      unsigned long ttl_ms = MQ135_WARMUP_TIME - (millis() - mq135_startup_time);
      snprintf(ttl_msg, sizeof(ttl_msg), "+ERROR: MQ135 sensor warming up, not ready yet, ttl: %lu ms", ttl_ms);
      return AT_R_S(String(ttl_msg));
    }
    calibrate_mq135_r0();
    MQ135_SAVE();
    return AT_R_DOUBLE(MQ135_CFG(r0));
  } else if(p = at_cmd_check("AT+MQ135_R0=", atcmdline, cmd_len)){
    float new_r0 = strtof(p, NULL);
    if(new_r0 < 1.0f || new_r0 > 1000.0f)
      return AT_R("+ERROR: invalid R0 value 1-1000 kOhm");
    LOG("[MQ-135] Setting R0 to %f kOhm", new_r0);
    MQ135_CFG(r0) = new_r0;
    MQ135_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_RL?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(rl));
  } else if(p = at_cmd_check("AT+MQ135_RL=", atcmdline, cmd_len)){
    float new_rl = strtof(p, NULL);
    if(new_rl < 0.001f || new_rl > 100000.0f)
      return AT_R("+ERROR: invalid RL value 0.001-10000 kOhm");
    LOG("[MQ-135] Setting RL to %f kOhm", new_rl);
    MQ135_CFG(rl) = new_rl;
    MQ135_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_REFERENCE_PPM?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(reference_ppm));
  } else if(p = at_cmd_check("AT+MQ135_REFERENCE_PPM=", atcmdline, cmd_len)){
    float new_ppm = strtof(p, NULL);
    if(new_ppm < 0.0f || new_ppm > 100000.0f)
      return AT_R("+ERROR: invalid PPM value 0-100000 ppm");
    LOG("[MQ-135] Setting PPM to %f ppm", new_ppm);
    MQ135_CFG(reference_ppm) = new_ppm;
    MQ135_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_A?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(a));
  } else if(p = at_cmd_check("AT+MQ135_A=", atcmdline, cmd_len)){
    float new_a = strtof(p, NULL);
    if(new_a < 0.001f || new_a > 10000.0f)
      return AT_R("+ERROR: invalid A coefficient value 0.001-10000");
    LOG("[MQ-135] Setting A coefficient to %f", new_a);
    MQ135_CFG(a) = new_a;
    MQ135_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_B?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(b));
  } else if(p = at_cmd_check("AT+MQ135_B=", atcmdline, cmd_len)){
    float new_b = strtof(p, NULL);
    if(new_b < -10.0f || new_b > 10.0f)
      return AT_R("+ERROR: invalid B coefficient value -10 to 10");
    LOG("[MQ-135] Setting B coefficient to %f", new_b);
    MQ135_CFG(b) = new_b;
    MQ135_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_VCC?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(MQ135_CFG(vcc));
  } else if(p = at_cmd_check("AT+MQ135_VCC=", atcmdline, cmd_len)){
    float new_vcc = strtof(p, NULL);
    if(new_vcc < 3.0f || new_vcc > 5.5f)
      return AT_R("+ERROR: invalid VCC voltage value 3.0-5.5V");
    LOG("[MQ-135] Setting VCC to %f V", new_vcc);
    MQ135_CFG(vcc) = new_vcc;
    MQ135_SAVE();
    return AT_R_OK;
  }
  return AT_R("+ERROR: unknown command");
}

void destroy_mq135_adc(sensor_r_t *s){
  // free userdata
  if(s->userdata != NULL){
    free(s->userdata);
    s->userdata = NULL;
  }
}

#endif // SUPPORT_MQ135

// APDS-9930/APDS-9960 Sensor
#define SENSOR_APDS9930_ALS        {.name = "APDS Ambient Light Sensor", .key = "apds_als",}
#define SENSOR_APDS9930_PROXIMITY  {.name = "APDS Proximity", .key = "apds_proximity",}
#ifdef SUPPORT_APDS9930
#define SENSOR_APDS9930_ALS \
    {\
      .name = "APDS Ambient Light Sensor",\
      .key  = "apds_als",\
      .unit_fmt = "lx,%.0f",\
      .init_function = init_apds9930,\
      .value_function = fetch_apds_als,\
    }
#define SENSOR_APDS9930_PROXIMITY \
    {\
      .name = "APDS Proximity",\
      .key  = "apds_proximity",\
      .unit_fmt = "cm,%.0f",\
      .init_function = init_apds9930,\
      .value_function = fetch_apds_proximity,\
    }

// see https://docs.broadcom.com/wcs-public/products/data-sheets--technical-specifications/data-sheet/736/749/av02-3190en_ds_apds-9930_2015-11-13.pdf
#define APDS99xx_I2C_ADDRESS 0x39 // default I2C address for APDS-9930/APDS-9960
#define APDS9930_CMD         0x80
#define APDS9930_ENABLE      (APDS9930_CMD | 0x00) // Enable register
#define APDS9930_ID          (APDS9930_CMD | 0x12) // ID register
#define APDS9930_CH0DATAL    (APDS9930_CMD | 0x14) // ALS channel 0 data low register
#define APDS9930_CH0DATAH    (APDS9930_CMD | 0x15) // ALS channel 0 data high register
#define APDS9930_CH1DATAL    (APDS9930_CMD | 0x16) // ALS channel 1 data low register
#define APDS9930_CH1DATAH    (APDS9930_CMD | 0x17) // ALS channel 1 data high register
#define APDS9930_PDATAL      (APDS9930_CMD | 0x18) // Proximity data low byte register
#define APDS9930_PDATAH      (APDS9930_CMD | 0x19) // Proximity data high byte register
#define APDS9930_ATIME       (APDS9930_CMD | 0x01) // ALS ADC time register
#define APDS9930_PTIME       (APDS9930_CMD | 0x02) // Proximity ADC time register
#define APDS9930_WTIME       (APDS9930_CMD | 0x03) // Wait time register
#define APDS9930_PPULSE      (APDS9930_CMD | 0x0E) // Proximity pulse count and length
#define APDS9930_CONTROL     (APDS9930_CMD | 0x0F) // Control register (gain settings)
#define APDS9930_AUTO_INC    0x20
#define APDS9930_CH0DATAL    (APDS9930_CMD | APDS9930_AUTO_INC | 0x14)
#define APDS9930_CH1DATAL    (APDS9930_CMD | APDS9930_AUTO_INC | 0x16)
#define APDS9930_PDATAL      (APDS9930_CMD | APDS9930_AUTO_INC | 0x18)
#define APDS9930_PICLEAR     (APDS9930_CMD | 0xE5) // Proximity interrupt clear

#define APDS99xx_ENABLE_PON  0x01 // Bit to power on
#define APDS99xx_ENABLE_AEN  0x02 // Bit to enable ALS
#define APDS99xx_ENABLE_PEN  0x04 // Bit to enable Proximity
#define APDS99xx_ENABLE_WEN  0x08 // Bit to enable wait timer


float lpc = nanf("0x12345");

int8_t fetch_apds_als(sensor_r_t *s, float *illuminance){
  if(illuminance == NULL)
    return -1;
  
  // Read both channels (16-bit)
  uint16_t ch0 = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_CH0DATAL); // Visible + IR
  uint16_t ch1 = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_CH1DATAL); // IR only

  // Check for read errors
  if (ch0 == 0xFFFF || ch1 == 0xFFFF)
    return -1;

  // Calculate Lux using the datasheet coefficients
  float lux1 = (ch0 - 1.862f * ch1);
  float lux2 = (0.746f * ch0 - 1.291f * ch1);
  
  float final_lux = (lux1 > lux2) ? lux1 : lux2;
  final_lux *= lpc;

  // Clean up negative values (happens in very dark/pure IR environments)
  if (final_lux < 0)
    final_lux = 0.0f;

  D("[APDS] Lux: %.2f, CH0: %u, CH1: %u, Lux: %.2f", final_lux, ch0, ch1);
  *illuminance = final_lux;
  return 1;
}

int8_t fetch_apds_proximity(sensor_r_t *s, float *proximity_value){
  if(proximity_value == NULL)
    return -1;

  // On APDS-9930, Proximity data is 16-bit (PDATAH:PDATAL)
  // Register 0x18 is PDATAL, 0x19 is PDATAH
  uint16_t prox = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_PDATAL);
  if(prox == 0xFFFF) {
    LOG("[APDS] Proximity read failed");
    return -1;
  }

  // don't allow 0
  if(prox == 0)
    prox = 1;

  // APDS-9930 proximity is 10-bit (0-1023)
  // Low values (< 500) typically mean nothing is near
  // High values (> 700) mean object is very close
  // Just return the raw value for now - calibrate based on your setup
  *proximity_value = (float)prox;

  // estimate distance in cm (very rough estimate)
  float distance = sqrt(1000 / *proximity_value);

  D("[APDS] prox raw: %u, distance: %0.2f", prox, distance);
  return 1;
}

void init_apds9930(sensor_r_t *s){
  // If already initialized, skip
  if(!isnan(lpc) && lpc != nanf("0x12345"))
    return;

  // ping
  if(i2c_ping(APDS99xx_I2C_ADDRESS) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[APDS] sensor not found on I2C address 0x%02X", APDS99xx_I2C_ADDRESS);
    return;
  }

  // read ID
  uint8_t id = i2c_read8(APDS99xx_I2C_ADDRESS, APDS9930_ID);
  if(id != 0x39 && id != 0xAB){ // APDS-9930 ID = 0x39, APDS-9960 ID = 0xAB
    LOG("[APDS] sensor not found on address 0x%02X, read id: 0x%02X", APDS99xx_I2C_ADDRESS, id);
  }
  LOG("[APDS] sensor found, read id: 0x%02X, type: %s", id, id == 0x39 ? "APDS-9930" : (id == 0xAB ? "APDS-9960" : "Unknown"));

  // powed down the device
  if(i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ENABLE, 0x00) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[APDS] failed to power down the device");
    return;
  }

  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ATIME,  0xDB);  // 101ms, 0x00=699ms, 0xDB=101ms, 0xC0=175ms, 0xF6=27.3ms, 0xFF=2.73ms
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_PTIME,  0xFF);  // 2.73ms recommended
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_WTIME,  0xFF);  // 2.73ms, 0x00=699, 0xb6=202, 0xff=2.73ms
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_PPULSE, 0x10);  // 16 pulses, 8us pulse length

  // note that 100mA and 8 pulses is recommended
  uint8_t PDRIVE = 0x2 << 6; // 25mA (lower power to reduce crosstalk)
  uint8_t PDIODE = 0x1 << 4; // Use Channel 0
  uint8_t PGAIN  = 0x1 << 2; // 2x Gain
  uint8_t AGAIN  = 0x1 << 0; // 8x Gain
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_CONTROL, PDRIVE | PDIODE | PGAIN | AGAIN);

  // Gain and Integration Time scaling
  // Based on your init: ALS Gain = 16x, ATIME = 100ms
  float als_it = 101.0f;  // ALS Integration time in ms -> 0x00=699, 0xC0=175, 0xDB=101, 0xF6=27.3, 0xFF=2.73
  float a_gain =   8.0f;  // ALS Gain
  float DF = 52.0f; // Device Factor from datasheet for APDS-9930
  float GA = 0.49f; // Glass Attenuation Factor, Open Air = 0.49
  lpc = GA * DF / (als_it * a_gain);

  // 1. Power ON (Wait for internal oscillator to stabilize)
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ENABLE, APDS99xx_ENABLE_PON);

  // 2. Clear any lingering interrupts just in case
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_PICLEAR, 0x00);

  // 3. Enable ALS and Proximity (and WEN if you need it)
  // Note: If you want interrupts, add | 0x20 here
  uint8_t final_enable = APDS99xx_ENABLE_PON | 
                         APDS99xx_ENABLE_AEN | 
                         APDS99xx_ENABLE_WEN |
                         APDS99xx_ENABLE_PEN; 

  // power on the device
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ENABLE, final_enable);
  LOG("[APDS] initialized on I2C address 0x%02X, id: 0x%02X", APDS99xx_I2C_ADDRESS, id);

  return;
}
#endif // SUPPORT_APDS9930

// SenseAir S8 NDIR CO2 Sensor
#define SENSOR_S8 {.name = "S8 CO2", .key = "s8_co2",}
#ifdef SUPPORT_S8
#define SENSOR_S8 \
    {\
      .name = "S8 CO2",\
      .key  = "s8_co2",\
      .unit_fmt = "ppm,%.0f",\
      .init_function = init_s8,\
      .value_function = fetch_s8_co2,\
    }

uint8_t is_calibrating = 0;
S8_UART *sensor_S8 = NULL;
S8_sensor sensor;

NOINLINE
void init_s8(sensor_r_t *s) {
  LOG("[S8] Initializing SenseAir S8 NDIR CO2 sensor");
  UART1.begin(S8_BAUDRATE, SERIAL_8N1, 1, 0);
  sensor_S8 = new S8_UART(UART1);

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  if(len == 0){
    LOG("[S8] CO2 sensor not found!");
    delete sensor_S8;
    sensor_S8 = NULL;
    s->cfg->enabled = 0; // Disable in config
    return;
  }

  // Show basic S8 sensor info
  LOG("[S8] SenseAir S8 NDIR CO2 sensor");
  LOG("[S8] Firmware version: %s", sensor.firm_version);
  sensor.sensor_id = sensor_S8->get_sensor_ID();
  LOG("[S8] Sensor ID: 0x%x", sensor.sensor_id);
  sensor.sensor_type_id = sensor_S8->get_sensor_type_ID();
  LOG("[S8] Sensor type ID: 0x%x", sensor.sensor_type_id);
  sensor.map_version = sensor_S8->get_memory_map_version();
  LOG("[S8] Sensor memory map ID: 0x%x", sensor.sensor_type_id);
  sensor.abc_period = sensor_S8->get_ABC_period();
  if(sensor.abc_period > 0){
    LOG("[S8] ABC (automatic background calibration) period: %d hours", sensor.abc_period);
  } else {
    LOG("[S8] ABC (automatic calibration) is disabled");
  }
  LOG("[S8] Setting ABC to 0 and wait 1s");
  sensor.abc_period = sensor_S8->set_ABC_period(0);
  delay(1000);
  sensor.abc_period = sensor_S8->get_ABC_period();
  if(sensor.abc_period > 0){
    LOG("[S8] ABC (automatic background calibration) period: %d hours", sensor.abc_period);
  } else {
    LOG("[S8] ABC (automatic calibration) is disabled (0s)");
  }

  // Check the health of the sensor
  LOG("[S8] Checking the health of the sensor");
  sensor.meter_status = sensor_S8->get_meter_status();
  if(sensor.meter_status & S8_MASK_METER_ANY_ERROR) {
    LOG("[S8] One or more errors detected!");
    if(sensor.meter_status & S8_MASK_METER_FATAL_ERROR)
      LOG("[S8] Fatal error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OFFSET_REGULATION_ERROR)
      LOG("[S8] Offset regulation error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_ALGORITHM_ERROR)
      LOG("[S8] Algorithm error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OUTPUT_ERROR)
      LOG("[S8] Output error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_SELF_DIAG_ERROR)
      LOG("[S8] Self diagnostics error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OUT_OF_RANGE)
      LOG("[S8] Out of range in sensor!");
    if(sensor.meter_status & S8_MASK_METER_MEMORY_ERROR)
      LOG("[S8] Memory error in sensor!");
  } else {
    LOG("[S8] The sensor is OK");
  }
  return;
}

NOINLINE
int8_t fetch_s8_co2(sensor_r_t *s, float *co2){
  if(co2 == NULL)
    return -1;
  // Fetch CO2 value from S8 sensor
  if(sensor_S8 == NULL) {
    LOG("[S8] sensor not initialized");
    return -1;
  }

  // check if calibration is in progress
  if(is_calibrating == 1){
    sensor.ack = sensor_S8->get_acknowledgement();
    if(sensor.ack & S8_MASK_CO2_BACKGROUND_CALIBRATION){
      LOG("[S8] Background calibration is finished");
      is_calibrating = 0;
    } else {
      LOG("[S8] Calibration in progress");
      return -1;
    }
  }

  // read CO2 value
  sensor.co2 = sensor_S8->get_co2();
  D("[S8] CO2 ppm: %d", sensor.co2);
  *co2 = (float)sensor.co2;
  return 1;
}

NOINLINE
char * calibrate_s8(){
  if(sensor_S8 == NULL) {
    LOG("[S8] sensor not initialized");
    return AT_R("+ERROR: sensor not initialized");
  }
  if(is_calibrating == 1){
    LOG("[S8] Calibration already in progress");
    sensor.ack = sensor_S8->get_acknowledgement();
    if(sensor.ack & S8_MASK_CO2_BACKGROUND_CALIBRATION)
      is_calibrating = 0;
    return AT_R("+ERROR: calibration already in progress");
  }
  LOG("[S8] Starting calibration...");
  if(!sensor_S8->manual_calibration()){
    LOG("[S8] Calibration command failed!");
    return AT_R("+ERROR: calibration command failed");
  }
  is_calibrating = 1;
  LOG("[S8] Calibration command sent");
  return AT_R_OK;
}

NOINLINE
const char *atcmd_sensors_s8(const char *atcmdline, size_t cmd_len){
  char *p = NULL;
  if(p = at_cmd_check("AT+S8_CALIBRATE_ZERO", atcmdline, cmd_len)){
    return calibrate_s8();
  }
  return AT_R("+ERROR: unknown command");
}
#endif // SUPPORT_S8

// SE95 Temperature Sensor
#define SENSOR_SE95_TEMPERATURE {.name = "SE95 Temperature", .key = "se95_temperature",}
#ifdef SUPPORT_SE95
#define SENSOR_SE95_TEMPERATURE \
    {\
      .name = "SE95 Temperature",\
      .key  = "se95_temperature",\
      .unit_fmt = "°C,%.5f",\
      .init_function = init_se95,\
      .value_function = fetch_se95_temperature,\
    }

#define SE95_I2C_ADDRESS      0x48 // default I2C address for SE95
#define SE95_TEMPERATURE      0x00 // Command to read temperature from SE95 sensor
#define SE95_CONFIGURATION    0x01 // Command to configure SE95 sensor
#define SE95_THYST            0x02 // Command to store the hysteresis threshold
#define SE95_TOS              0x03 // Command to store the overtemperature shutdown threshold
#define SE95_ID               0x05 // Command to read the ID of the SE95 sensor

void init_se95(sensor_r_t *s) {
  if(i2c_ping(SE95_I2C_ADDRESS) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[SE95] sensor not found");
    return;
  }
  LOG("[SE95] initialized on I2C address 0x%02X", SE95_TEMPERATURE);
  return;
}

int8_t fetch_se95_temperature(sensor_r_t *s, float *temperature){
  // Convert the raw temperature data
  // MSB=0, +(TEMP * 0.03125)
  // MSB=1, -(TEMP two complement) * 0.03125
  // get 13 bits, shift right 3
  uint16_t raw_temp = i2c_read16be(SE95_I2C_ADDRESS, SE95_TEMPERATURE);
  if(raw_temp == 0xFFFF){
    LOG("[SE95] failed to read temperature");
    return -1;
  }
  raw_temp >>= 3;
  float temp = (float)raw_temp * 0.03125f;
  D("[SE95] temperature: %.2f °C", temp);
  *temperature = temp;
  return 1;
}
#endif // SUPPORT_SE95

// BH1750 Digital Light Sensor
#define SENSOR_BH1750_ILLUMINANCE {.name = "BH1750 Illuminance", .key = "bh1750_illuminance",}
#ifdef SUPPORT_BH1750
#define SENSOR_BH1750_ILLUMINANCE \
    {\
      .name = "BH1750 Illuminance",\
      .key  = "bh1750_illuminance",\
      .unit_fmt = "lx,%.2f",\
      .init_function = init_bh1750,\
      .value_function = fetch_bh1750_illuminance,\
    }

#define BH1750_I2C_ADDRESS       0x23 // default I2C address for BH1750 (ADDR pin LOW)
#define BH1750_POWER_DOWN        0x00 // No active state
#define BH1750_POWER_ON          0x01 // Waiting for measurement command
#define BH1750_RESET             0x07 // Reset data register value
#define BH1750_CONTINUOUS_HIGH_RES_MODE   0x10 // Continuous H-Resolution mode (1 lx resolution, 120ms measurement time)
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2 0x11 // Continuous H-Resolution mode 2 (0.5 lx resolution, 120ms measurement time)
#define BH1750_CONTINUOUS_LOW_RES_MODE    0x13 // Continuous L-Resolution mode (4 lx resolution, 16ms measurement time)
#define BH1750_ONE_TIME_HIGH_RES_MODE     0x20 // One time H-Resolution mode
#define BH1750_ONE_TIME_HIGH_RES_MODE_2   0x21 // One time H-Resolution mode 2
#define BH1750_ONE_TIME_LOW_RES_MODE      0x23 // One time L-Resolution mode

RTC_DATA_ATTR float last_bh1750_illuminance = 0.0f;

void init_bh1750(sensor_r_t *s) {
  if(i2c_ping(BH1750_I2C_ADDRESS) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[BH1750] sensor not found on I2C address 0x%02X", BH1750_I2C_ADDRESS);
    return;
  }

  // Power on the sensor
  i2c_write(BH1750_I2C_ADDRESS, BH1750_POWER_ON);
  i2c_write(BH1750_I2C_ADDRESS, BH1750_RESET);

  // Trigger first measurement in One-Time High Resolution Mode
  if(i2c_write(BH1750_I2C_ADDRESS, BH1750_ONE_TIME_HIGH_RES_MODE) == -1){
    s->cfg->enabled = 0;
    LOG("[BH1750] failed to set measurement mode");
    return;
  }

  LOG("[BH1750] initialized on I2C address 0x%02X", BH1750_I2C_ADDRESS);
}

int8_t fetch_bh1750_illuminance(sensor_r_t *s, float *illuminance){
  if(illuminance == NULL)
    return -1;

  // Read 2 bytes from sensor, this is from the previous measurement command
  uint16_t raw_value = i2c_read16raw(BH1750_I2C_ADDRESS);
  if(raw_value == 0xFFFF){
    LOG("[BH1750] failed to read data from sensor");
    *illuminance = last_bh1750_illuminance;
    return -1;
  }

  // Immediately trigger the NEXT measurement so it's ready for the next fetch
  // This allows the sensor to work in the background without blocking your main
  // loop later
  i2c_write(BH1750_I2C_ADDRESS, BH1750_ONE_TIME_HIGH_RES_MODE);

  // Convert to lux (divide by 1.2 as per datasheet)
  float lux = raw_value / 1.2f;

  if(lux < 0.0f || lux > 65535.0f){
    LOG("[BH1750] illuminance out of range: %.2f", lux);
    *illuminance = last_bh1750_illuminance;
    return -1;
  }

  D("[BH1750] illuminance: %.2f lx", lux);
  *illuminance = lux;
  last_bh1750_illuminance = lux;
  return 1;
}
#endif // SUPPORT_BH1750

#define SENSOR_DS18B20_TEMPERATURE {.name = "DS18B20 Temperature", .key = "ds18b20_temperature",}
#ifdef SUPPORT_DS18B20
#define SENSOR_DS18B20_TEMPERATURE \
    {\
      .name = "DS18B20 Temperature",\
      .key  = "ds18b20_temperature",\
      .unit_fmt = "°C,%.4f",\
      .init_function = init_ds18b20,\
      .value_function = fetch_ds18b20_temperature,\
    }

#define ONEWIRE_BUS_PIN GPIO_NUM_4 // GPIO where the DS18B20 is connected

enum ds18b20_state_t {
  DS_IDLE,
  DS_REQUESTING
};

OneWire oneWire; // Create OneWire instance
ds18b20_state_t sensorState = DS_IDLE;
DallasTemperature ds18b20;
DeviceAddress ds18b20_address;

void init_ds18b20(sensor_r_t *s) {
  // Initialize the OneWire pin
  oneWire.begin(ONEWIRE_BUS_PIN); 
  LOG("[DS18B20] Initialized OneWire on pin %d", ONEWIRE_BUS_PIN);

  // Link the ds18b20 object to our specific OneWire instance
  ds18b20.setOneWire(&oneWire);

  // Start the sensor library
  ds18b20.begin();

  // Look for the first sensor on the bus and save its address
  if (!ds18b20.getAddress(ds18b20_address, 0)) {
    // Set to non-blocking mode
    ds18b20.setWaitForConversion(false);
    // Set resolution to 12 bits (can be 9, 10, 11, or 12)
    ds18b20.setResolution(ds18b20_address, 12);
  
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[DS18B20] ERROR: Could not find OneWire address for DS18B20 at Index 0");
  } else {
    LOG("[DS18B20] Found DS18B20 sensor with address: %16X", *((uint64_t*)ds18b20_address));
  }
}

int8_t fetch_ds18b20_temperature(sensor_r_t *s, float *temperature){
  if(temperature == NULL)
    return -1;

  if(sensorState == DS_IDLE){
    // Request temperature measurement
    ds18b20.requestTemperatures(); 
    sensorState = DS_REQUESTING;
    LOG("[DS18B20] Requested temperature measurement");
    return -1; // not ready yet
  }
  if(!ds18b20.isConversionComplete()){
    LOG("[DS18B20] Temperature conversion not ready yet");
    return -1; // not ready yet
  }

  // Fetch temperature in Celsius
  float tempC = (float)ds18b20.getTempC(ds18b20_address);
  // Check for disconnected sensor, note that DEVICE_DISCONNECTED_C returns -127.0°C
  if(tempC <= -126.0f || tempC >= 184.0f) { 
    LOG("[DS18B20] ERROR: Sensor disconnected or invalid read");
    return -1;
  }
  // Request temperature measurement, takes ~0ms, this is for the next run
  ds18b20.requestTemperatures(); 
  sensorState = DS_REQUESTING;

  D("[DS18B20] temperature: %.4f °C", tempC);
  *temperature = tempC;

  // for MQ-135 compensation
  last_temperature = tempC;
  return 1;
}
#endif // SUPPORT_DS18B20

sensor_r_t all_sensors[NR_OF_SENSORS] = {
    SENSOR_DHT11_HUMIDITY,
    SENSOR_DHT11_TEMPERATURE,
    SENSOR_BME280_HUMIDITY,
    SENSOR_BME280_TEMPERATURE,
    SENSOR_BME280_PRESSURE,
    SENSOR_LDR,
    SENSOR_NTC_TEMPERATURE,
    SENSOR_MQ135,
    SENSOR_APDS9930_ALS,
    SENSOR_APDS9930_PROXIMITY,
    SENSOR_S8,
    SENSOR_SE95_TEMPERATURE,
    SENSOR_BH1750_ILLUMINANCE,
    SENSOR_DS18B20_TEMPERATURE,
};

NOINLINE
void initialize(){
  // sensors timer init on first boot, as initialize() is only called once
  // the l_intv_counters is on RTC and will keep its value over deep sleep
  // cycles for correct interval timing
  for(int i = 0; i < NR_OF_SENSORS; i++)
    l_intv_counters[i] = millis();
}

NOINLINE
void setup(){

  // initialize config system, loads partition/namespace handle
  CFG_INIT();

  // load main config
  CFG_LOAD(CFG_STORAGE, &SENSORS::cfg, sizeof(SENSORS::cfg));

  // config check for interval, when 0, assume 1000ms
  for(int i = 0; i < NR_OF_SENSORS; i++){
    sensor_r_t *s = &SENSORS::all_sensors[i];

    // at runtime, link cfg to sensor
    s->cfg = &SENSORS::cfg.sensor_cfg[i];

    // set default interval if 0 or too low
    if(s->cfg->v_intv == 0)
      s->cfg->v_intv = 1000;
    if(s->cfg->v_intv < 100)
      s->cfg->v_intv = 100;

    // call init function
    LOG("[SENSORS] Setting up sensor index:%d, name:%s", i, s->name);
    if(s->init_function != NULL){
      // call function
      s->init_function(s);
    } else {
      if(s->value_function == NULL)
        LOG("[SENSORS] Sensor index:%d, name:%s not configured, skipping setup", i, s->name);
    }
  }
}

NOINLINE
void sensors_loop(){
  // loop through sensors and call pre function
  for(int i = 0; i < NR_OF_SENSORS; i++){
    sensor_r_t *s = &SENSORS::all_sensors[i];
    if(s->cfg->enabled == 0)
      continue;
    doYIELD;
    if(s->pre_function == NULL)
      continue;
    LOGFLUSH();
    s->pre_function(s);
  }

  // loop through sensors and check if we need to fetch & log
  for(int i = 0; i < NR_OF_SENSORS; i++){
    sensor_r_t *s = &SENSORS::all_sensors[i];
    if(s->cfg->enabled == 0)
      continue;
    doYIELD;
    if(s->value_function == NULL)
      continue;
    if(millis() - l_intv_counters[i] <= s->cfg->v_intv)
      continue;

    // update last interval counter
    l_intv_counters[i] = millis();

    // fetch current sensor value
    float current_v = 0.0f;
    int8_t ok = s->value_function(s, &current_v);
    if(ok < 0){
      LOG("[SENSORS] ERROR: failed to fetch value for sensor %s, skipping", s->key);
      continue;
    }

    // Validate the value - reject NaN and infinity
    if(isnan(current_v) || isinf(current_v)){
      LOG("[SENSORS] ERROR: invalid value (NaN or infinity) for sensor %s, skipping", s->key);
      continue;
    }
    LOG("[SENSORS][%s]: value %.5f", s->key, current_v);

    // format sensor value string
    ALIGN(4) static char sv_str[32] = {0};
    int h_strl = snprintf((char *)&sv_str, sizeof(sv_str), s->unit_fmt, current_v);
    if(h_strl < 0){
      LOG("[SENSORS] ERROR: snprintf failed for sensor %s: %s", s->key, strerror(errno));
      continue;
    }
    D("[SENSORS] Sensor %s value '%s'", s->key, sv_str);

    // prepare output buffer
    ALIGN(4) static char ou_buf[128] = {0};
    memset(ou_buf, 0, sizeof(ou_buf));
    int s_strl = -1;
    if(SENSORS::cfg.log_time && SENSORS::cfg.time_fmt != NULL && strlen(SENSORS::cfg.time_fmt) > 0){
      if(SENSORS::cfg.kvmkey == NULL || strlen(SENSORS::cfg.kvmkey) == 0){
        s_strl = snprintf(ou_buf, sizeof(ou_buf), "%s,%s*%s\r\n", COMMON::PT(SENSORS::cfg.time_fmt), s->key, sv_str);
      } else {
        s_strl = snprintf(ou_buf, sizeof(ou_buf), "%s,%s:%s*%s\r\n", COMMON::PT(SENSORS::cfg.time_fmt), SENSORS::cfg.kvmkey, s->key, sv_str);
      }
    } else {
      if(SENSORS::cfg.kvmkey == NULL || strlen(SENSORS::cfg.kvmkey) == 0){
        s_strl = snprintf(ou_buf, sizeof(ou_buf), "%s*%s\r\n", s->key, sv_str);
      } else {
        s_strl = snprintf(ou_buf, sizeof(ou_buf), "%s:%s*%s\r\n", SENSORS::cfg.kvmkey, s->key, sv_str);
      }
    }
    if(s_strl < 0){
      LOG("[SENSORS] ERROR: snprintf failed to format output for sensor %s: %s", s->key, strerror(errno));
      continue;
    }

    // output over UART?
    if(SENSORS::cfg.log_uart){
      for(size_t i = 0; i < s_strl; i++)
        Serial.write(ou_buf[i]);
      Serial.flush();
    }
    // copy over to "inbuf" from esp-at.ino
    uint8_t *b_old = ::inbuf + ::inlen;
    uint8_t *b_new = b_old;
    size_t copy_len_max = (size_t)::inbuf_max - (size_t)b_new;
    if((size_t)s_strl <= copy_len_max){
      copy_len_max = (size_t)s_strl;
    } else {
      LOG("[SENSORS] ERROR: only %d bytes to inbuf, had %d bytes for sensor %d", copy_len_max, s_strl, i);
    }
    D("[SENSORS] copying %d bytes to inbuf for sensor %s", copy_len_max, s->key);
    memcpy(b_new, (uint8_t *)ou_buf, copy_len_max);
    ::inlen += copy_len_max;
  }

  // loop through sensors and call post function
  for(int i = 0; i < NR_OF_SENSORS; i++){
    sensor_r_t *s = &SENSORS::all_sensors[i];
    if(s->cfg->enabled == 0)
      continue;
    doYIELD;
    if(s->post_function == NULL)
      continue;
    s->post_function(s);
  }
}

NOINLINE
const char* at_cmd_handler_sensor(const char *at_cmd, unsigned short at_len){
    const char *p = NULL;
    for (int i = 0; i < NR_OF_SENSORS; i++) {
        sensor_r_t *s = &SENSORS::all_sensors[i];
        if (p = at_cmd_check("AT+ENABLE_", at_cmd, at_len)) {
            // move pointer past "AT+ENABLE_"
            p += strlen("AT+ENABLE_");
            // AT+ENABLE_<sensor>=<0|1> or AT+ENABLE_<sensor>?
            // match sensor?
            LOG("[SENSORS] at_cmd_handler_sensor: checking sensor '%s' for key '%s' to '%s'", s->name, s->key, p);
            if(strncasecmp(s->key, p, strlen(s->key)) != 0)
                continue; // not matching sensor key
            if(s->value_function == NULL)
                return AT_R("+ERROR: Sensor not supported in this build");
            // move pointer to the = or ? part
            p += strlen(s->key);
            if(*p == '?') {
                // query enable status
                return AT_R_INT(s->cfg->enabled);
            }
            if(*p != '=') {
                // error handle
                return AT_R("+ERROR: Enable command must end with =<0|1> or ?");
            }
            p++; // move past '='

            int val = atoi(p);
            if (val != 0 && val != 1) {
                return AT_R("+ERROR: Enable must be 0 or 1");
            }
            s->cfg->enabled = val;
            CFG_SAVE();
            return AT_R_OK;
        } else if (p = at_cmd_check("AT+LOG_INTERVAL_", at_cmd, at_len)){
            // move pointer past "AT+LOG_INTERVAL_"
            p += strlen("AT+LOG_INTERVAL_");
            // AT+LOG_INTERVAL_<sensor>=<interval>
            // match sensor?
            if(strncasecmp(s->key, p, strlen(s->key)) != 0)
                continue; // not matching sensor key
            if(s->value_function == NULL)
                return AT_R("+ERROR: Sensor not supported in this build");
            // move pointer to the = part
            p += strlen(s->key);
            if(*p == '?') {
                // query enable status
                return AT_R_INT(s->cfg->v_intv);
            }
            if(*p != '=') {
                // error handle
                return AT_R("+ERROR: Log interval command must end with =<interval>");
            }
            p++; // move past '='

            unsigned long new_interval = strtoul(p, NULL, 10);
            if(new_interval < 100){
              return AT_R("+ERROR: Log interval must be at least 100ms");
            }
            s->cfg->v_intv = new_interval;
            CFG_SAVE();
            return AT_R_OK;
        } else if (p = at_cmd_check("AT+VALUE_", at_cmd, at_len)){
            // move pointer past "AT+VALUE_"
            p += strlen("AT+VALUE_");
            // AT+VALUE_<sensor>?
            // match sensor?
            if(strncasecmp(s->key, p, strlen(s->key)) != 0)
                continue; // not matching sensor key
            if(s->value_function == NULL)
                return AT_R("+ERROR: Sensor not supported in this build");
            // move pointer to the ? part
            p += strlen(s->key);
            if(*p != '?') {
                // error handle
                return AT_R("+ERROR: Value command must end with ?");
            }
            // Call pre function if available
            if(s->pre_function != NULL)
                s->pre_function(s);
            // fetch current sensor value
            float current_v = 0.0f;
            int8_t ok = s->value_function(s, &current_v);
            if(ok < 0)
              return AT_R("+ERROR: failed to fetch sensor value");
            // Validate the value - reject NaN and infinity
            if(isnan(current_v) || isinf(current_v))
              return AT_R("+ERROR: invalid sensor value (NaN or infinity)");
            // Call post function if available
            if(s->post_function != NULL)
                s->post_function(s);
            return AT_R_DOUBLE(current_v);
        } else {
            continue; // continue to next sensor
        }
    }
    return AT_R("+ERROR: unknown sensor command");
}

NOINLINE
const char* at_cmd_handler_sensors(const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  if(p = at_cmd_check("AT+SENSORS_KVMKEY=", atcmdline, cmd_len)){
    size_t sz = strlen(p);
    if(sz+1 > 16)
      return AT_R("+ERROR: Location max 15 chars");
    if(sz == 0)
      SENSORS::cfg.kvmkey[0] = '\0';
    else
      strncpy((char *)&SENSORS::cfg.kvmkey, p, sz);
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+SENSORS_KVMKEY?", atcmdline, cmd_len)){
    return AT_R(SENSORS::cfg.kvmkey);
  } else if(p = at_cmd_check("AT+SENSORS_TIMESTAMP_ADD=", atcmdline, cmd_len)){
    char *r = NULL;
    unsigned long val = strtoul(p, &r, 10);
    if(errno != 0 || (val != 0 && val != 1) || (r == p))
      return AT_R("+ERROR: LOG_TIME must be 0 or 1");
    SENSORS::cfg.log_time = val;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+SENSORS_TIMESTAMP_ADD?", atcmdline, cmd_len)){
    return AT_R_INT(SENSORS::cfg.log_time);
  } else if(p = at_cmd_check("AT+SENSORS_LOG_UART=", atcmdline, cmd_len)){
    char *r = NULL;
    unsigned long val = strtoul(p, &r, 10);
    if(errno != 0 || (val != 0 && val != 1) || (r == p))
      return AT_R("+ERROR: LOG_UART must be 0 or 1");
    SENSORS::cfg.log_uart = val;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+SENSORS_TIMESTAMP_FMT=", atcmdline, cmd_len)){
    size_t sz = strlen(p);
    if(sz+1 > 32)
      return AT_R("+ERROR: Timestamp format max 31 chars");
    if(sz == 0)
      SENSORS::cfg.time_fmt[0] = '\0';
    else
      strncpy((char *)&SENSORS::cfg.time_fmt, p, sz);
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+SENSORS_TIMESTAMP_FMT?", atcmdline, cmd_len)){
    return AT_R(SENSORS::cfg.time_fmt);
  } else if(p = at_cmd_check("AT+SENSORS_LOG_UART?", atcmdline, cmd_len)){
    return AT_R_INT(SENSORS::cfg.log_uart);
  #ifdef SUPPORT_MQ135
  } else if(p = at_cmd_check("AT+MQ135_",atcmdline, cmd_len)){
    return atcmd_sensors_mq135(atcmdline, cmd_len);
  #endif // SUPPORT_MQ135
  #ifdef SUPPORT_LDR
  } else if(p = at_cmd_check("AT+LDR_",atcmdline, cmd_len)){
    return atcmd_sensors_ldr(atcmdline, cmd_len);
  #endif // SUPPORT_LDR
  #ifdef SUPPORT_NTC
  } else if(p = at_cmd_check("AT+NTC_",atcmdline, cmd_len)){
    return atcmd_sensors_ntc(atcmdline, cmd_len);
  #endif // SUPPORT_NTC
  #ifdef SUPPORT_S8
  } else if(p = at_cmd_check("AT+S8_",atcmdline, cmd_len)){
    return atcmd_sensors_s8(atcmdline, cmd_len);
  #endif // SUPPORT_S8
  } else if(p = at_cmd_check("AT+LOG_INTERVAL_", atcmdline, cmd_len)){
    return at_cmd_handler_sensor(atcmdline, cmd_len);
  } else if(p = at_cmd_check("AT+ENABLE_", atcmdline, cmd_len)){
    return at_cmd_handler_sensor(atcmdline, cmd_len);
  } else if(p = at_cmd_check("AT+VALUE_", atcmdline, cmd_len)){
    return at_cmd_handler_sensor(atcmdline, cmd_len);
  }
  return AT_R("+ERROR: unknown command");
}

NOINLINE
long max_sleep_time(){
  // using the current time in millis() together with the sensor intervals,
  // and the last run time, calculate the maxium sleep time
  long cur_time = millis();
  long min_time = LONG_MAX;
  for(int i = 0; i < NR_OF_SENSORS; i++){
    sensor_r_t *s = &SENSORS::all_sensors[i];
    D("[SENSORS] Sensor %s enabled: %d, interval: %lu ms, last run: %lu ms", s->key, s->cfg->enabled, s->cfg->v_intv, l_intv_counters[i]);
    if(s->cfg->enabled == 0)
      continue;
    if(s->cfg->v_intv == 0)
      continue;
    if(s->value_function == NULL)
      continue;
    if(l_intv_counters[i] == 0){
      if(s->cfg->v_intv < min_time)
        min_time = s->cfg->v_intv;
    } else {
      unsigned long time_since_last = cur_time - l_intv_counters[i];
      if(time_since_last >= s->cfg->v_intv)
        // sensor is due now, return 0
        return 0;
      unsigned long time_to_next = s->cfg->v_intv - time_since_last;
      D("[SENSORS] Sensor %s time to next: %lu ms", s->key, time_to_next);
      // check if this is the minimum time
      if(time_to_next < min_time)
        min_time = time_to_next;
    }
  }
  return min_time;
}

} // namespace SENSORS

namespace PLUGINS {
    NOINLINE
    void clear_config(){
        SENSORS::CFG_CLEAR(NULL);
    }
    NOINLINE
    void setup(){
        SENSORS::setup();
    }
    NOINLINE
    void initialize(){
        SENSORS::initialize();
    }
    NOINLINE
    void loop_pre(){
        // nothing to do here
    }
    NOINLINE
    void loop_post(){
        SENSORS::sensors_loop();
    }
    NOINLINE
    long max_sleep_time(){
        return SENSORS::max_sleep_time();
    }
    NOINLINE
    const char * at_cmd_handler(const char* atcmdline){
        return SENSORS::at_cmd_handler_sensors(atcmdline);
    }
    NOINLINE
    const char * at_get_help_string(){
        return R"EOF(
Sensor Commands:
  AT+SENSORS_KVMKEY=<location>  - Set KVM key/location identifier (max 15 chars)
  AT+SENSORS_KVMKEY?            - Get KVM key/location identifier
  AT+SENSORS_TIMESTAMP_ADD=<0|1> - Enable/disable timestamp logging (1=enable, 0=disable)
  AT+SENSORS_TIMESTAMP_ADD?     - Get timestamp logging status
  AT+SENSORS_TIMESTAMP_FMT=<format> - Set timestamp format (strftime format)
  AT+SENSORS_TIMESTAMP_FMT?     - Get timestamp format
  AT+SENSORS_LOG_UART=<0|1>     - Enable/disable UART logging (1=enable, 0=disable)
  AT+SENSORS_LOG_UART?          - Get UART logging status
)EOF"

#ifdef SUPPORT_MQ135
        R"EOF(
  AT+MQ135_R0=<value>           - Set MQ-135 R0 resistance value (1-1000 kOhms)
  AT+MQ135_R0?                  - Get MQ-135 R0 resistance value
  AT+MQ135_RL=<value>           - Set MQ-135 RL load resistance value (1-1000 kOhms)
  AT+MQ135_RL?                  - Get MQ-135 RL load resistance value
  AT+MQ135_CALIBRATE            - Calibrate MQ-135 R0 for current PPM
  AT+MQ135_REFERENCE_PPM=<value> - Set PPM for calibration (0-100000 ppm, default 428.54 for CO2)
  AT+MQ135_REFERENCE_PPM?       - Get PPM value
  AT+MQ135_A=<value>            - Set gas curve coefficient A (0.001-10000, default 110.47 for CO2)
  AT+MQ135_A?                   - Get gas curve coefficient A
  AT+MQ135_B=<value>            - Set gas curve coefficient B (-10 to 10, default -2.862 for CO2)
  AT+MQ135_B?                   - Get gas curve coefficient B
  AT+MQ135_VCC=<value>          - Set MQ-135 VCC voltage (3.0-5.5V, default 5.0)
  AT+MQ135_VCC?                 - Get MQ-135 VCC voltage
)EOF"
#endif // SUPPORT_MQ135

#ifdef SUPPORT_LDR
R"EOF(
  AT+LDR_VCC=<value>            - Set LDR VCC voltage (0-5V)
  AT+LDR_VCC?                   - Get LDR VCC voltage
  AT+LDR_DIVIDER_R=<value>      - Set LDR divider resistance in Ohm
  AT+LDR_DIVIDER_R?             - Get LDR divider resistance
  AT+LDR_EMA_ALPHA=<value>      - Set LDR EMA alpha for smoothing (0-1)
  AT+LDR_EMA_ALPHA?             - Get LDR EMA alpha
  AT+LDR_CALIBRATE_LUX=<value>  - Calibrate LDR with known lux value
  AT+LDR_R10=<value>            - Set LDR R10 reference resistance in Ohm
  AT+LDR_R10?                   - Get LDR R10 reference resistance
  AT+LDR_GAMMA=<value>          - Set LDR gamma value
  AT+LDR_GAMMA?                 - Get LDR gamma value
)EOF"
#endif // SUPPORT_LDR

#ifdef SUPPORT_NTC
R"EOF(
  AT+NTC_VCC=<value>            - Set NTC VCC voltage (0-5V)
  AT+NTC_VCC?                   - Get NTC VCC voltage
  AT+NTC_DIVIDER_R=<value>      - Set NTC divider resistance in Ohm
  AT+NTC_DIVIDER_R?             - Get NTC divider resistance
  AT+NTC_BETA=<value>           - Set NTC beta coefficient
  AT+NTC_BETA?                  - Get NTC beta coefficient
  AT+NTC_R_NOMINAL=<value>      - Set NTC nominal resistance in Ohm
  AT+NTC_R_NOMINAL?             - Get NTC nominal resistance
  AT+NTC_T_NOMINAL=<value>      - Set NTC nominal temperature in Celsius
  AT+NTC_T_NOMINAL?             - Get NTC nominal temperature
  AT+NTC_EMA_ALPHA=<value>      - Set NTC EMA alpha for smoothing (0-1)
  AT+NTC_EMA_ALPHA?             - Get NTC EMA alpha
)EOF"
#endif // SUPPORT_NTC

#ifdef SUPPORT_S8
R"EOF(
  AT+S8_CALIBRATE_ZERO          - Calibrate SenseAir S8 zero point
)EOF"
#endif // SUPPORT_S8

        R"EOF(
  AT+ENABLE_<sensor>=<0|1>      - Enable/disable sensor (1=enable, 0=disable)
  AT+ENABLE_<sensor>?           - Get sensor enable status
  AT+LOG_INTERVAL_<sensor>=<ms> - Set sensor logging interval (minimum 100ms)
  AT+LOG_INTERVAL_<sensor>?     - Get sensor logging interval
  AT+VALUE_<sensor>?            - Get current sensor value

Available sensors:
)EOF"

#ifdef SUPPORT_DHT11
        R"EOF(
  - HUMIDITY                    - DHT11 humidity sensor
  - TEMPERATURE                 - DHT11 temperature sensor
)EOF"
#endif // SUPPORT_DHT11

#ifdef SUPPORT_LDR
        R"EOF(
  - LDR_ILLUMINANCE             - LDR light sensor
)EOF"
#endif // SUPPORT_LDR

#ifdef SUPPORT_NTC
        R"EOF(
  - NTC_TEMPERATURE             - NTC temperature sensor
)EOF"
#endif // SUPPORT_NTC

#ifdef SUPPORT_MQ135
        R"EOF(
  - AIR_QUALITY                 - MQ-135 air quality sensor
)EOF"
#endif // SUPPORT_MQ135

#ifdef SUPPORT_APDS9930
        R"EOF(
  - APDS_ALS                    - APDS-9930/APDS-9960 Ambient Light Sensor
  - APDS_PROXIMITY              - APDS-9930/APDS-9960 Proximity sensor
)EOF"
#endif // SUPPORT_APDS9930

#ifdef SUPPORT_SE95
        R"EOF(
  - SE95_TEMPERATURE            - SE95 I2C temperature sensor
)EOF"
#endif // SUPPORT_SE95

#ifdef SUPPORT_BH1750
        R"EOF(
  - BH1750_ILLUMINANCE          - BH1750 Digital Light Sensor
)EOF"
#endif // SUPPORT_BH1750

#ifdef SUPPORT_S8
        R"EOF(
  - S8_CO2                      - SenseAir S8 CO2 sensor
)EOF"
#endif // SUPPORT_S8

#ifdef SUPPORT_DS18B20
        R"EOF(
  - DS18B20_TEMPERATURE         - DS18B20 temperature sensor
)EOF"
#endif // SUPPORT_DS18B20

        R"EOF(
Examples:
  AT+ENABLE_HUMIDITY=1          - Enable DHT11 humidity sensor
  AT+LOG_INTERVAL_TEMPERATURE=5000 - Set temperature logging to 5 seconds
  AT+VALUE_TEMPERATURE?         - Get current temperature reading
  AT+SENSORS_KVMKEY=livingroom  - Set location to "livingroom"
  AT+SENSORS_TIMESTAMP_ADD=1    - Enable timestamp logging
  AT+SENSORS_TIMESTAMP_FMT=%Y-%m-%d %H:%M:%S - Set timestamp format
  AT+SENSORS_LOG_UART=1         - Enable UART logging
)EOF";
    }
}

