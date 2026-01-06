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

#ifdef SUPPORT_S8
#include "s8_uart.h"
#endif // SUPPORT_S8

#ifdef SUPPORT_SE95
#include <Wire.h>
#endif // SUPPORT_SE95

#ifdef SUPPORT_APDS9930
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#endif // SUPPORT_APDS9930

namespace SENSORS {

RTC_DATA_ATTR long l_intv_counters[NR_OF_SENSORS] = {0};

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

// APDS-9930 Sensor
#define SENSOR_APDS9930_ILLUMINANCE {.name = "APDS-9930 Illuminance", .key = "apds_illuminance",}
#define SENSOR_APDS9930_COLOR       {.name = "APDS-9930 Color", .key = "apds_color",}
#ifdef SUPPORT_APDS9930
#define SENSOR_APDS9930_ILLUMINANCE \
    {\
      .name = "APDS-9930 Illuminance",\
      .key  = "apds_illuminance",\
      .unit_fmt = "%s:%s*lx,%.0f\r\n",\
      .init_function = init_apds9930,\
      .value_function = fetch_apds_illuminance,\
    }
#define SENSOR_APDS9930_COLOR \
    {\
      .name = "APDS-9930 Color",\
      .key  = "apds_color",\
      .unit_fmt = "%s:%s*C,%.0f\r\n",\
      .init_function = init_apds9930,\
      .value_function = fetch_apds_color,\
    }

// re-export Wire for sensors.cpp in this SENSORS namespace, note that "Wire"
// is a global extern object
TwoWire Wire = Wire;
#define APDS9930_I2C_ADDRESS 0x39 // default I2C address for APDS-9930

Adafruit_APDS9930 apds(APDS9930_I2C_ADDRESS);

int8_t fetch_apds_illuminance(sensor_r_t *s, double *illuminance){
  if(illuminance == NULL)
    return -1;
  // fetch APDS-9930 illuminance (lux)
  float lux = 0;
  apds.getLux(&lux);
  LOG("[APDS-9930] lux: %.2f", lux);
  *illuminance = (double)lux;
  return 1
}

int8_t fetch_apds_color(sensor_r_t *s, double *color_value){
  if(color_value == NULL)
    return -1;
  // fetch APDS-9930 color (returns C, but logs R,G,B,C)
  uint16_t apds_r = 0, apds_g = 0, apds_b = 0, apds_c = 0;
  apds.getRGB(&apds_r, &apds_g, &apds_b, &apds_c);
  LOG("[APDS-9930] RGB: %d,%d,%d,%d", apds_r, apds_g, apds_b, apds_c);
  // For the main value, return clear channel (C)
  *color_value = (double)apds_c;
  return 1;
}

void init_apds9930(sensor_r_t *s){
  if(!apds.begin()) {
    LOG("[APDS-9930] not found");
    return;
  }
  apds.enableColor(true);
  apds.enableLightSensor(true);
  LOG("[APDS-9930] initialized");
  return;
}
#endif // APDS-9930

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

// re-export Wire for sensors.cpp in this SENSORS namespace, note that "Wire"
// is a global extern object
TwoWire Wire = Wire;
#define I2C_DAT   6
#define I2C_CLK   7

#define SE95_I2C_ADDRESS      0x48 // default I2C address for SE95
#define SE95_TEMPERATURE      0x00 // Command to read temperature from SE95 sensor
#define SE95_CONFIGURATION    0x01 // Command to configure SE95 sensor
#define SE95_THYST            0x02 // Command to store the hysteresis threshold
#define SE95_TOS              0x03 // Command to store the overtemperature shutdown threshold
#define SE95_ID               0x05 // Command to read the ID of the SE95 sensor

void init_se95(sensor_r_t *s) {
  // Initialize SE95 temperature sensor
  Wire.begin();
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.setPins(I2C_DAT, I2C_CLK);
  #endif
  // Fetch temperature from SE95 sensor
  Wire.beginTransmission(SE95_I2C_ADDRESS);
  if(Wire.endTransmission() != ESP_OK){
      s->cfg->enabled = 0 ; // Disable in config
    LOG("[SE95] sensor not found");
    return;
  }
  LOG("[SE95] temperature sensor initialized");
  return;
}

int8_t fetch_se95_temperature(sensor_r_t *s, double *temperature){
  Wire.beginTransmission(SE95_I2C_ADDRESS);
  Wire.write(SE95_TEMPERATURE);
  Wire.endTransmission();
  Wire.requestFrom(SE95_I2C_ADDRESS, 2);
  uint8_t msb, lsb;
  if(Wire.available()){
    msb = Wire.read();
  } else {
    LOG("[SE95] temperature read error, no data available");
    return -1; // No data available, return 0.0
  }
  if(Wire.available()){
    lsb = Wire.read();
  } else {
    LOG("[SE95] temperature read error, no data available");
    return -1; // No data available, return 0.0
  }
  if(Wire.endTransmission() != ESP_OK){
    LOG("[SE95] temperature read error, end transmission failed");
    return -1; // Transmission error, return 0.0
  }

  // Convert the raw temperature data
  // MSB=0, +(TEMP * 0.03125)
  // MSB=1, -(TEMP two complement) * 0.03125
  // get 13 bits, shift right 3
  uint16_t raw_temp = ((msb << 8) | lsb) >> 3;
  double temp;
  temp = (double)raw_temp * 0.03125;

  // Fetch temperature from SE95 sensor
  LOG("[SE95] temperature: %.2f °C", temp);
  *temperature = (double)temp;
  return 1;
}
#endif // SUPPORT_SE95


sensor_r_t all_sensors[NR_OF_SENSORS] = {
    SENSOR_DHT11_HUMIDITY,
    SENSOR_DHT11_TEMPERATURE,
    SENSOR_LDR,
    SENSOR_MQ135,
    SENSOR_APDS9930_ILLUMINANCE,
    SENSOR_APDS9930_COLOR,
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
    if(millis() - mq135_startup_time < MQ135_WARMUP_TIME)
      return AT_R("+ERROR: MQ135 sensor warming up, not ready yet");
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
  - APDS_ILLUMINANCE            - APDS-9930 light sensor
  - APDS_COLOR                  - APDS-9930 RGB color sensor
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

