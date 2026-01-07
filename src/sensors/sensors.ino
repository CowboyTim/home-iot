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

#if defined(SUPPORT_SE95) || defined(SUPPORT_BME280) || defined(SUPPORT_BMP280) || defined(SUPPORT_APDS9930)
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

#ifdef SUPPORT_S8
#include "s8_uart.h"
#endif

namespace SENSORS {

RTC_DATA_ATTR long l_intv_counters[NR_OF_SENSORS] = {0};

#if defined(SUPPORT_SE95) || defined(SUPPORT_BME280) || defined(SUPPORT_BMP280) || defined(SUPPORT_APDS9930)
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
  uint8_t value;
  if(Wire.available())
    value = Wire.read();
  else
    return 0xFF;
  if(Wire.endTransmission() != ESP_OK)
    return 0xFF;
  return value;
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
  if(Wire.endTransmission() != ESP_OK)
    return 0xFFFF;
  if(big_endian)
    return ((b1 << 8) | b2);
  else
    return ((b2 << 8) | b1);
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

NOINLINE
void CFG_SAVE() {
    CFG::SAVE("esp-at", "sensors", "sensors", (void *)&SENSORS::cfg, sizeof(SENSORS::cfg));
}

NOINLINE
void CFG_INIT() {
  if(!CFG::INIT("esp-at", "sensors")){
    LOGE("[CFG] Failed to initialize NVS storage for sensors");
    ESP.restart();
  } else {
    LOG("[CFG] NVS storage initialized for sensors");
  }
  CFG::LOAD("esp-at", "sensors", "sensors", (void *)&SENSORS::cfg, sizeof(SENSORS::cfg));
}


// DHT11 Sensor
#define SENSOR_DHT11_HUMIDITY    {.name = "DHT11 Humidity", .key = "humidity",}
#define SENSOR_DHT11_TEMPERATURE {.name = "DHT11 Temperature", .key = "temperature",}
#ifdef SUPPORT_DHT11
#define SENSOR_DHT11_HUMIDITY \
    {\
      .name = "DHT11 Humidity",\
      .key  = "humidity",\
      .unit_fmt = "%s:%s*%%,%.0f\r\n",\
      .init_function = init_dht11,\
      .pre_function = pre_dht11,\
      .value_function = dht11_fetch_humidity,\
      .post_function = post_dht11,\
    }
#define SENSOR_DHT11_TEMPERATURE \
    {\
      .name = "DHT11 Temperature",\
      .key  = "temperature",\
      .unit_fmt = "%s:%s*°C,%.2f\r\n",\
      .init_function = init_dht11,\
      .pre_function = pre_dht11,\
      .value_function = dht11_fetch_temperature,\
      .post_function = post_dht11,\
    }

#define DODHT(s) ((DHT*)(s->userdata))

#define DHTPIN  A0     // GPIO_NUM_0/A0 pin for DHT11

uint8_t did_dht11 = 0; // DHT11 read flag, to avoid multiple reads
RTC_DATA_ATTR double last_dht_humidity = 0.0;
RTC_DATA_ATTR double last_dht_temperature = 0.0;
RTC_DATA_ATTR unsigned long last_dht_read_time = 0;

int8_t dht11_fetch_humidity(sensor_r_t *s, double *humidity){
  if(humidity == NULL)
    return -1;
  // fetch humidity from DHT11
  if(!did_dht11){
    DODHT(s)->read();
    did_dht11 = 1;
  }
  double h = (double)DODHT(s)->readHumidity();
  LOG("[DHT11] humidity: %f %%", h);
  if(h < 0.0 || h > 100.0){
    LOG("[DHT11] humidity invalid or out of range, returning 0: %.2f", h);
    *humidity = 0.0;
    return -1;
  }
  *humidity = h;
  last_dht_humidity = h;
  last_dht_read_time = millis();
  return 1;
}

int8_t dht11_fetch_temperature(sensor_r_t *s, double *temperature){
  if(temperature == NULL)
    return -1;
  // fetch temperature from DHT11
  if(!did_dht11){
    DODHT(s)->read();
    did_dht11 = 1;
  }
  double t = (double)DODHT(s)->readTemperature();
  LOG("[DHT11] temperature: %f °C", t);
  if(t < 0.0 || t > 50.0){
    LOG("[DHT11] temperature invalid or out of range, returning 0: %.2f", t);
    *temperature = 0.0;
    return -1;
  }
  *temperature = t;
  last_dht_temperature = t;
  last_dht_read_time = millis();
  return 1;
}

void pre_dht11(sensor_r_t *s){
  did_dht11 = 0;
}

void post_dht11(sensor_r_t *s){
  did_dht11 = 0;
}

void init_dht11(sensor_r_t *s){
  // initialize DHT11 sensor
  s->userdata = new DHT(DHTPIN, DHT11);
  if(did_dht11 == 0){
    DODHT(s)->begin();
    LOG("[DHT11] initialized on pin %d", DHTPIN);
    did_dht11 = 1;
  }
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
      .unit_fmt = "%s:%s*%%,%.0f\r\n",\
      .init_function = init_bme280,\
      .pre_function = pre_bme280,\
      .value_function = bme280_fetch_humidity,\
      .post_function = post_bme280,\
    }
#define SENSOR_BME280_TEMPERATURE \
    {\
      .name = "BME280 Temperature",\
      .key  = "bme280_temperature",\
      .unit_fmt = "%s:%s*°C,%.2f\r\n",\
      .init_function = init_bme280,\
      .pre_function = pre_bme280,\
      .value_function = bme280_fetch_temperature,\
      .post_function = post_bme280,\
    }
#define SENSOR_BME280_PRESSURE \
    {\
      .name = "BME280 Pressure",\
      .key  = "bme280_pressure",\
      .unit_fmt = "%s:%s*hPa,%.2f\r\n",\
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

uint8_t did_bme280 = 0; // BME280 read flag
RTC_DATA_ATTR double last_bme280_humidity = 0.0;
RTC_DATA_ATTR double last_bme280_temperature = 0.0;
RTC_DATA_ATTR double last_bme280_pressure = 0.0;
RTC_DATA_ATTR unsigned long last_bme280_read_time = 0;

int8_t bme280_fetch_humidity(sensor_r_t *s, double *humidity){
  if(humidity == NULL)
    return -1;
  if(!did_bme280){
    // TODO
    last_bme280_read_time = millis();
    did_bme280 = 1;
  }
  LOG("[BME280] humidity: %f %%", last_bme280_humidity);
  if(last_bme280_humidity < 0.0 || last_bme280_humidity > 100.0){
    LOG("[BME280] humidity invalid or out of range: %.2f", last_bme280_humidity);
    *humidity = 0.0;
    return -1;
  }
  *humidity = last_bme280_humidity;
  return 1;
}

int8_t bme280_fetch_temperature(sensor_r_t *s, double *temperature){
  if(temperature == NULL)
    return -1;
  if(!did_bme280){
    // TODO
    last_bme280_read_time = millis();
    did_bme280 = 1;
  }
  LOG("[BME280] temperature: %f °C", last_bme280_temperature);
  if(last_bme280_temperature < -40.0 || last_bme280_temperature > 85.0){
    LOG("[BME280] temperature invalid or out of range: %.2f", last_bme280_temperature);
    *temperature = 0.0;
    return -1;
  }
  *temperature = last_bme280_temperature;
  return 1;
}

int8_t bme280_fetch_pressure(sensor_r_t *s, double *pressure){
  if(pressure == NULL)
    return -1;
  if(!did_bme280){
    // TODO
    last_bme280_read_time = millis();
    did_bme280 = 1;
  }
  LOG("[BME280] pressure: %f hPa", last_bme280_pressure);
  if(last_bme280_pressure < 300.0 || last_bme280_pressure > 1100.0){
    LOG("[BME280] pressure invalid or out of range: %.2f", last_bme280_pressure);
    *pressure = 0.0;
    return -1;
  }
  *pressure = last_bme280_pressure;
  return 1;
}

void pre_bme280(sensor_r_t *s){
  did_bme280 = 0;
}

void post_bme280(sensor_r_t *s){
  did_bme280 = 0;
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

// LDR Sensor
#define SENSOR_LDR {.name = "LDR Illuminance", .key = "ldr_illuminance",}
#ifdef SUPPORT_LDR
#define SENSOR_LDR \
    {\
      .name = "LDR Illuminance",\
      .key  = "ldr_illuminance",\
      .unit_fmt = "%s:%s*lx,%.0f\r\n",\
      .init_function = init_ldr_adc,\
      .value_function = fetch_ldr_adc,\
    }

#define LDRPIN    A1 // GPIO_NUM_1/A1 pin for LDR
int8_t fetch_ldr_adc(sensor_r_t *s, double *ldr_value){
  if(ldr_value == NULL)
    return -1;
  // fetch LDR ADC value
  int ldr_adc = analogReadMilliVolts(LDRPIN); // assuming LDR is connected to LDRPIN
  LOG("[LDR/ADC] value: %d mV", ldr_adc);
  *ldr_value = (double)ldr_adc; // convert to double for consistency
  return 1;
}

void init_ldr_adc(sensor_r_t *s){
  // initialize LDR ADC pin
  pinMode(LDRPIN, INPUT); // assuming LDR is connected to LDRPIN
  LOG("[LDR/ADC] initialized on pin %d", LDRPIN);
}
#endif // SUPPORT_LDR

// MQ-135 Air Quality Sensor
#define SENSOR_MQ135 {.name = "MQ-135 Air Quality", .key = "air_quality",}
#ifdef SUPPORT_MQ135
#define SENSOR_MQ135 \
    {\
      .name = "MQ-135 Air Quality",\
      .key  = "air_quality",\
      .unit_fmt = "%s:%s*ppm,%.0f\r\n",\
      .init_function = init_mq135_adc,\
      .value_function = fetch_mq135_adc,\
      .destroy_function = destroy_mq135_adc,\
    }

#include "soc/adc_channel.h"
#define MQ135PIN           A2 // GPIO_NUM_2/A2 pin for MQ-135
#define MQ135_ADC_CHANNEL  ADC1_GPIO2_CHANNEL
#define MQ135_RL         22.0 // kOhm load resistor
#define MQ135_R0        76.63 // kOhm clean air resistance
#define MQ135_VCC         5.0 // Sensor powered by 5V (from USB)
#define MQ135_ADC_REF_VOLTAGE_IN_MV 3300 // ESP32 ADC reference voltage in mV
#define MQ135_AVG_NR       50 // number of samples to average from ADC, higher = more stable, don't go too high or uint32_t overflow
#define MQ135_ADC_MAX    4095 // 12-bit ADC max value
#define MQ135_ADC_BITS     12 // 12-bit ADC

// For CO2: a = 110.47, b = -2.862 (from datasheet)
#define MQ135_CO2_A    110.47 // MQ-135 CO2 curve a
#define MQ135_CO2_B    -2.862 // MQ-135 CO2 curve b

#define MQ135_WARMUP_TIME    30000  // MQ-135 warm-up time in ms
#define ATMOSPHERIC_CO2_PPM  428.54 // atmospheric CO2 ppm for calibration

RTC_DATA_ATTR unsigned long mq135_startup_time = 0;

double mq135_adc_to_ppm(double mq135_r0, double mq135_rl, double adc_value) {
  double RS = ((MQ135_VCC - adc_value) / adc_value ) * mq135_rl; // in kOhm

  #ifdef SUPPORT_DHT11
  // apply a correction based on temperature and humidity if available
  if(last_dht_read_time != 0 && millis() - last_dht_read_time < 60000){ // valid DHT11 reading within last 60s
    double cf = 0.00035 * pow(last_dht_temperature, 2) - 0.019 * last_dht_temperature + 1.224;
    cf += (last_dht_humidity - 33.0) * -0.0018;
    RS /= cf;
  }
  #endif // SUPPORT_DHT11

  double RATIO = RS / mq135_r0;
  double ppm = MQ135_CO2_A * pow(RATIO, MQ135_CO2_B);
  LOG("[MQ-135] ADC Value: %f V, R0: %f kOhm, RL: %f kOhm, RS: %f kOhm, R: %f, PPM: %f", adc_value, mq135_r0, mq135_rl, RS, RATIO, ppm);
  return ppm;
}

double calibrate_mq135_r0(double mq135_rl, double adc_value) {
  double RS = ((MQ135_VCC - adc_value) / adc_value ) * mq135_rl; // in kOhm
  #ifdef SUPPORT_DHT11
  // apply a correction based on temperature and humidity if available
  if(last_dht_read_time != 0 && millis() - last_dht_read_time < 60000){ // valid DHT11 reading within last 60s
    double cf = 0.00035 * pow(last_dht_temperature, 2) - 0.019 * last_dht_temperature + 1.224;
    cf += (last_dht_humidity - 33.0) * -0.0018;
    RS /= cf;
  }
  #endif // SUPPORT_DHT11
  double R0 = RS / pow((ATMOSPHERIC_CO2_PPM / MQ135_CO2_A), (1.0 / MQ135_CO2_B));
  LOG("[MQ-135] Calibration ADC value: Voltage: %f V, RL: %f kOhm, RS: %f kOhm, R0: %f kOhm", adc_value, mq135_rl, RS, R0);
  return R0;
}

double get_adc_average(uint8_t samples) {
  uint32_t avg_adc = 0.0;
  for(uint8_t i = 0; i < samples; i++) {
    avg_adc += analogRead(MQ135PIN);
    delayMicroseconds(10);
    doYIELD;
  }
  avg_adc *= MQ135_ADC_REF_VOLTAGE_IN_MV;
  double v_adc = (double)avg_adc / samples;
  v_adc  /= MQ135_ADC_MAX;
  v_adc  /= 1000.0; // convert mV to V
  return v_adc;
}

int8_t fetch_mq135_adc(sensor_r_t *s, double *ppm){
  if(ppm == NULL)
    return -1;
  if(millis() - mq135_startup_time < MQ135_WARMUP_TIME){
    LOG("[MQ-135] sensor warming up, not ready yet, ttl: %d ms", MQ135_WARMUP_TIME - (millis() - mq135_startup_time));
    return -1; // sensor warming up
  }
  double R0 = SENSORS::cfg.mq135_r0;
  double RL = SENSORS::cfg.mq135_rl;

  // fetch average ADC value
  double avg_adc = get_adc_average(MQ135_AVG_NR);
  LOG("[MQ-135] ADC AVG(nr:%d) value: %f V", MQ135_AVG_NR, avg_adc);

  // convert ADC to PPM using MQ-135 formula R0/RL and curve
  *ppm = mq135_adc_to_ppm(R0, RL, avg_adc);
  LOG("[MQ-135] CO2 PPM: %f", *ppm);
  return 1;
}

void init_mq135_adc(sensor_r_t *s){
  // initialize MQ-135 ADC pin
  pinMode(MQ135PIN, INPUT_PULLDOWN); // assuming MQ-135 is connected to MQ135PIN
  pinMode(MQ135PIN, ANALOG);
  analogRead(MQ135PIN);
  analogSetPinAttenuation(MQ135PIN, ADC_11db);
  analogReadResolution(MQ135_ADC_BITS);
  double R0 = SENSORS::cfg.mq135_r0;
  double RL = SENSORS::cfg.mq135_rl;
  LOG("[MQ-135] ADC initialized on pin %d, ADC channel: %d, resolution: 12, attenuation 11db, R0: %0.f Ohm, RL: %0.f", MQ135PIN, MQ135_ADC_CHANNEL, R0, RL);

  // wait for MQ-135 to stabilize
  if(mq135_startup_time == 0)
    mq135_startup_time = millis();
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
      .unit_fmt = "%s:%s*lx,%.0f\r\n",\
      .init_function = init_apds9930,\
      .value_function = fetch_apds_als,\
    }
#define SENSOR_APDS9930_PROXIMITY \
    {\
      .name = "APDS Proximity",\
      .key  = "apds_proximity",\
      .unit_fmt = "%s:%s*C,%.0f\r\n",\
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

#define APDS99xx_ENABLE_PON  0x01 // Bit to power on
#define APDS99xx_ENABLE_AEN  0x02 // Bit to enable ALS
#define APDS99xx_ENABLE_PEN  0x04 // Bit to enable Proximity
#define APDS99xx_ENABLE_WEN  0x08 // Bit to enable wait timer

double lpc = nan("0x12345");

int8_t fetch_apds_als(sensor_r_t *s, double *illuminance){
  if(illuminance == NULL)
    return -1;
  
  // Read both channels (16-bit)
  uint16_t ch0 = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_CH0DATAL); // Visible + IR
  uint16_t ch1 = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_CH1DATAL); // IR only

  // Check for read errors
  if (ch0 == 0xFFFF || ch1 == 0xFFFF)
    return -1;

  // Calculate Lux using the datasheet coefficients
  double lux1 = (ch0 - 1.862 * ch1);
  double lux2 = (0.746 * ch0 - 1.291 * ch1);
  
  double final_lux = (lux1 > lux2) ? lux1 : lux2;
  final_lux *= lpc;

  // Clean up negative values (happens in very dark/pure IR environments)
  if (final_lux < 0)
    final_lux = 0;

  LOG("[APDS] CH0: %u, CH1: %u, Lux: %.2f", ch0, ch1, final_lux);
  *illuminance = final_lux;
  return 1;
}

int8_t fetch_apds_proximity(sensor_r_t *s, double *proximity_value){
  if(proximity_value == NULL)
    return -1;

  // On APDS-9930, Proximity data is 16-bit (PDATAH:PDATAL)
  // Register 0x12 is PDATAL, 0x13 is PDATAH
  uint16_t prox = i2c_read16le(APDS99xx_I2C_ADDRESS, APDS9930_PDATAL);
  if(prox == 0xFFFF) {
    LOG("[APDS] Proximity read failed");
    return -1;
  }

  // If prox is very low, the object is close
  // Simplified inverse relationship: Distance is roughly proportional to 1/sqrt(prox)
  if (prox < 1) {
    *proximity_value = 50.0; // Assume max distance if no reflection
  } else {
    *proximity_value = 50.0 / sqrt(prox);
  }

  LOG("[APDS] prox: %u, estimated distance: %.2f cm", prox, *proximity_value);
  return 1;
}

void init_apds9930(sensor_r_t *s){
  // If already initialized, skip
  if(!isnan(lpc) && lpc != nan("0x12345"))
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

  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ATIME,  0xDB);  // 101ms, 0x00=699ms, 0xC0=175ms, 0xF6=27.3ms, 0xFF=2.73ms
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_PTIME,  0xFF);  // 2.73ms recommended
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_WTIME,  0xFF);  // 2.73ms, 0x00=699, 0xb6=202, 0xff=2.73ms
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_PPULSE, 0x08);  // 8 pulses

  // note that 100mA and 8 pulses is recommended
  uint8_t PDRIVE, PDIODE, PGAIN, AGAIN;
  PDRIVE = 0x0 << 6; // 100mA of LED Power
  PDIODE = 0x2 << 4; // Proximity diode selection: CH1
  PGAIN  = 0x3 << 2; // Proximity gain 8x
  AGAIN  = 0x1 << 0; // 8x ALS gain
  i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_CONTROL, PDRIVE | PDIODE | PGAIN | AGAIN);

  // Gain and Integration Time scaling
  // Based on your init: ALS Gain = 16x, ATIME = 100ms
  double als_it = 101;  // ALS Integration time in ms -> 0x00=699, 0xC0=175, 0xDB=101, 0xF6=27.3, 0xFF=2.73
  double a_gain =   8;  // ALS Gain
  double DF = 52.0; // Device Factor from datasheet for APDS-9930
  double GA = 0.49; // Glass Attenuation Factor, Open Air = 0.49
  lpc = GA * DF / (als_it * a_gain);

  // power on the device
  if(i2c_write(APDS99xx_I2C_ADDRESS, APDS9930_ENABLE, APDS99xx_ENABLE_PON|APDS99xx_ENABLE_AEN|APDS99xx_ENABLE_PEN|APDS99xx_ENABLE_WEN) == -1){
    s->cfg->enabled = 0 ; // Disable in config
    LOG("[APDS] failed to power on the device");
    return;
  }
  LOG("[APDS] initialized on I2C address 0x%02X, id: 0x%02X", APDS99xx_I2C_ADDRESS, id);

  // wait for the sensor to be ready
  delay(12);

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
      .unit_fmt = "%s:%s*ppm,%.0f\r\n",\
      .init_function = init_s8,\
      .value_function = fetch_s8_co2,\
    }

S8_UART *sensor_S8 = NULL;
S8_sensor sensor;
void init_s8(sensor_r_t *s) {
  LOG("[S8] Initializing SenseAir S8 NDIR CO2 sensor");
  Serial1.begin(S8_BAUDRATE, SERIAL_8N1, 1, 0);
  sensor_S8 = new S8_UART(Serial1);

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

int8_t fetch_s8_co2(sensor_r_t *s, double *co2){
  if(co2 == NULL)
    return -1;
  // Fetch CO2 value from S8 sensor
  if(sensor_S8 == NULL) {
    LOG("[S8] sensor not initialized");
    return -1;
  }

  sensor.co2 = sensor_S8->get_co2();
  LOG("[S8] CO2 ppm: %d", sensor.co2);
  *co2 = (double)sensor.co2;
  return 1;
}
#endif // SUPPORT_S8

// SE95 Temperature Sensor
#define SENSOR_SE95_TEMPERATURE {.name = "SE95 Temperature", .key = "se95_temperature",}
#ifdef SUPPORT_SE95
#define SENSOR_SE95_TEMPERATURE \
    {\
      .name = "SE95 Temperature",\
      .key  = "se95_temperature",\
      .unit_fmt = "%s:%s*°C,%.5f\r\n",\
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

int8_t fetch_se95_temperature(sensor_r_t *s, double *temperature){
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
  double temp = (double)raw_temp * 0.03125;
  LOG("[SE95] temperature: %.2f °C", temp);
  *temperature = temp;
  return 1;
}
#endif // SUPPORT_SE95

sensor_r_t all_sensors[NR_OF_SENSORS] = {
    SENSOR_DHT11_HUMIDITY,
    SENSOR_DHT11_TEMPERATURE,
    SENSOR_BME280_HUMIDITY,
    SENSOR_BME280_TEMPERATURE,
    SENSOR_BME280_PRESSURE,
    SENSOR_LDR,
    SENSOR_MQ135,
    SENSOR_APDS9930_ALS,
    SENSOR_APDS9930_PROXIMITY,
    SENSOR_S8,
    SENSOR_SE95_TEMPERATURE,
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

  // load config
  CFG_INIT();

  // sensors config check for interval, when 0, assume 1000ms
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
    if(millis() - l_intv_counters[i] > s->cfg->v_intv){
      // fetch current sensor value
      double current_v = 0.0;
      int8_t ok = s->value_function(s, &current_v);
      if(ok < 0){
        LOG("[SENSORS] ERROR: failed to fetch value for sensor %s, skipping", s->key);
        l_intv_counters[i] = millis();
        continue;
      }

      // Validate the value - reject NaN and infinity
      if(isnan(current_v) || isinf(current_v)){
        LOG("[SENSORS] ERROR: invalid value (NaN or infinity) for sensor %s, skipping", s->key);
        l_intv_counters[i] = millis();
        continue;
      }
      memset((char*)s->out_buf, 0, sizeof(s->out_buf));
      int h_strl = snprintf((char *)s->out_buf, sizeof(s->out_buf), s->unit_fmt, SENSORS::cfg.kvmkey, s->key, current_v);
      if(h_strl > 0){
        // output over UART?
        if(SENSORS::cfg.log_uart){
            for(size_t i = 0; i < h_strl; i++)
              Serial.write((uint8_t)s->out_buf[i]);
            Serial.flush();
        }
        // copy over to "inbuf" from esp-at.ino
        uint8_t *b_old = ::inbuf + ::inlen;
        uint8_t *b_new = b_old;
        size_t copy_len_max = (size_t)::inbuf_max - (size_t)b_new;
        if((size_t)h_strl <= copy_len_max){
          copy_len_max = (size_t)h_strl;
        } else {
          LOG("[SENSORS] ERROR: only %d bytes to inbuf, had %d bytes for sensor %d", copy_len_max, h_strl, i);
        }
        D("[SENSORS] copying %d bytes to inbuf for sensor %s", copy_len_max, s->key);
        memcpy(b_new, (uint8_t *)s->out_buf, copy_len_max);
        ::inlen += copy_len_max;
      } else {
        LOG("[SENSORS] ERROR: snprintf failed for sensor %s: %s", s->key, strerror(errno));
      }
      l_intv_counters[i] = millis();
    }
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
            double current_v = 0.0;
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
  if(p = at_cmd_check("AT+KVMKEY=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 16)
      return AT_R("+ERROR: Location max 16 chars");
    strncpy((char *)&SENSORS::cfg.kvmkey, p, sz);
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+KVMKEY?", atcmdline, cmd_len)){
    return AT_R(SENSORS::cfg.kvmkey);
  #ifdef SUPPORT_MQ135
  } else if(p = at_cmd_check("AT+MQ135_R0?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(SENSORS::cfg.mq135_r0);
  } else if(p = at_cmd_check("AT+MQ135_CALIBRATE_CO2", atcmdline, cmd_len)){
    if(SENSORS::cfg.mq135_rl <= 0.0)
      return AT_R("+ERROR: invalid MQ135 RL value, set RL first");
    if(millis() - mq135_startup_time < MQ135_WARMUP_TIME){
      char ttl_msg[100] = {0};
      unsigned long ttl_ms = MQ135_WARMUP_TIME - (millis() - mq135_startup_time);
      snprintf(ttl_msg, sizeof(ttl_msg), "+ERROR: MQ135 sensor warming up, not ready yet, ttl: %lu ms", ttl_ms);
      return AT_R_S(String(ttl_msg));
    }
    // fetch average ADC value
    double avg_adc = get_adc_average(MQ135_AVG_NR);
    LOG("[MQ-135] Calibration ADC AVG(nr:%d) value: %f V", MQ135_AVG_NR, avg_adc);
    SENSORS::cfg.mq135_r0 = calibrate_mq135_r0(SENSORS::cfg.mq135_rl, avg_adc);
    CFG_SAVE();
    return AT_R_DOUBLE(SENSORS::cfg.mq135_r0);
  } else if(p = at_cmd_check("AT+MQ135_R0=", atcmdline, cmd_len)){
    double new_r0 = atof(p);
    if(new_r0 < 1.0 || new_r0 > 1000.0)
      return AT_R("+ERROR: invalid R0 value 1-1000 kOhm");
    SENSORS::cfg.mq135_r0 = new_r0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MQ135_RL?", atcmdline, cmd_len)){
    return AT_R_DOUBLE(SENSORS::cfg.mq135_rl);
  } else if(p = at_cmd_check("AT+MQ135_RL=", atcmdline, cmd_len)){
    double new_r0 = atof(p);
    if(new_r0 < 1.0 || new_r0 > 1000.0)
      return AT_R("+ERROR: invalid RL value 1-1000 kOhm");
    SENSORS::cfg.mq135_rl = new_r0;
    CFG_SAVE();
    return AT_R_OK;
  #endif // SUPPORT_MQ135
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

NOINLINE
void erase_config(){
    // erase NVS config for sensors
    CFG::CLEAR("esp-at", "sensors", "sensors");
}


} // namespace SENSORS

namespace PLUGINS {
    NOINLINE
    void clear_config(){
        SENSORS::erase_config();
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
  AT+KVMKEY=<location>          - Set KVM key/location identifier (max 15 chars)
  AT+KVMKEY?                    - Get KVM key/location identifier
)EOF"

#ifdef SUPPORT_MQ135
        R"EOF(
  AT+MQ135_R0=<value>           - Set MQ-135 R0 resistance value (1-1000 kOhms)
  AT+MQ135_R0?                  - Get MQ-135 R0 resistance value
  AT+MQ135_RL=<value>           - Set MQ-135 RL load resistance value (1-1000 kOhms)
  AT+MQ135_RL?                  - Get MQ-135 RL load resistance value
)EOF"
#endif // SUPPORT_MQ135

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

#ifdef SUPPORT_MQ135
        R"EOF(
  - AIR_QUALITY                 - MQ-135 air quality/CO2 sensor
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

#ifdef SUPPORT_S8
        R"EOF(
  - S8_CO2                      - SenseAir S8 CO2 sensor
)EOF"
#endif // SUPPORT_S8

        R"EOF(
Examples:
  AT+ENABLE_HUMIDITY=1          - Enable DHT11 humidity sensor
  AT+LOG_INTERVAL_TEMPERATURE=5000 - Set temperature logging to 5 seconds
  AT+VALUE_TEMPERATURE?         - Get current temperature reading
  AT+KVMKEY=livingroom          - Set location to "livingroom"
)EOF";
    }
}

