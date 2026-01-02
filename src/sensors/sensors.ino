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

// Logging setup for esp32c3

#ifndef VERBOSE
#define VERBOSE
#endif // VERBOSE

#include <sensors.h>
#include <common.h>

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


#ifdef SUPPORT_DHT11
#define DHTPIN  A0     // GPIO_NUM_0/A0 pin for DHT11
DHT dht = DHT(DHTPIN, DHT11);
uint8_t did_dht11 = 0; // DHT11 read flag, to avoid multiple reads
double dht11_fetch_humidity(sensor_r_t *s){
  // fetch humidity from DHT11
  if(!did_dht11){
    dht.read();
    did_dht11 = 1;
  }
  double h = (double)dht.readHumidity();
  LOG("[DHT11] humidity: %f %%", h);
  if(h < 0.0 || h > 100.0){
    LOG("[DHT11] humidity out of range, returning 0: %.2f", h);
    h = 0.0;
  }
  return h;
}

double dht11_fetch_temperature(sensor_r_t *s){
  // fetch temperature from DHT11
  if(!did_dht11){
    dht.read();
    did_dht11 = 1;
  }
  double t = (double)dht.readTemperature();
  LOG("[DHT11] temperature: %f °C", t);
  if(t < -40.0 || t > 80.0){
    LOG("[DHT11] temperature out of range, returning 0: %.2f", t);
    t = 0.0;
  }
  return t;
}

void pre_dht11(sensor_r_t *s){
  did_dht11 = 0;
}

void post_dht11(sensor_r_t *s){
  did_dht11 = 0;
}

void init_dht11(sensor_r_t *s){
  // initialize DHT11 sensor
  if(did_dht11 == 0){
    dht.begin();
    LOG("[DHT11] initialized on pin %d", DHTPIN);
    did_dht11 = 1;
  }
}

#endif // SUPPORT_DHT11

#ifdef SUPPORT_LDR
#define LDRPIN    A1 // GPIO_NUM_1/A1 pin for LDR
double fetch_ldr_adc(sensor_r_t *s){
  // fetch LDR ADC value
  int ldr_adc = analogReadMilliVolts(LDRPIN); // assuming LDR is connected to LDRPIN
  LOG("[LDR/ADC] value: %d mV", ldr_adc);
  double ldr_value = (double)ldr_adc; // convert to double for consistency
  return ldr_value;
}

void init_ldr_adc(sensor_r_t *s){
  // initialize LDR ADC pin
  pinMode(LDRPIN, INPUT); // assuming LDR is connected to LDRPIN
  LOG("[LDR/ADC] initialized on pin %d", LDRPIN);
}
#endif // SUPPORT_LDR

// MQ-135 Air Quality Sensor
#ifdef SUPPORT_MQ135
#define MQ135PIN           A2 // GPIO_NUM_2/A2 pin for MQ-135
#define MQ135_RL      10000.0 // 10k Ohm load resistor
#define MQ135_VCC         5.0 // Sensor powered by 5V
#define MQ135_ADC_REF     3.3 // ESP32 ADC reference voltage

double mq135_adc_to_ppm(double mq135_r0, int adc_value) {
  float voltage = (float)adc_value * MQ135_ADC_REF / 4095.0;
  float RS = (MQ135_VCC - voltage) * MQ135_RL / voltage;
  float ratio = RS / mq135_r0;
  // For CO2: a = 110.47, b = -2.862 (from datasheet)
  float ppm = pow(10, (log10(ratio) - log10(110.47)) / -2.862);
  return ppm;
}

double fetch_mq135_adc(sensor_r_t *s){
  double mq135_r0 = 10000.0; // default R0 value
  if(s->userdata != NULL)
    mq135_r0 = *((double*)s->userdata);
  // fetch MQ-135 ADC value
  int mq135_adc = analogRead(MQ135PIN); // raw ADC value (0-4095)
  LOG("[MQ-135] ADC value: %d", mq135_adc);
  double ppm = mq135_adc_to_ppm(mq135_r0, mq135_adc);
  LOG("[MQ-135] CO2 ppm: %.2f", ppm);
  return ppm;
}

void init_mq135_adc(sensor_r_t *s){
  // initialize MQ-135 ADC pin
  pinMode(MQ135PIN, INPUT);
  analogSetPinAttenuation(MQ135PIN, ADC_11db);
  analogReadResolution(12);
  s->userdata = malloc(sizeof(double));
  if(s->userdata == NULL)
    LOG("[MQ-135] ERROR: unable to allocate memory for userdata, using default R0 of 10k Ohm");
  memcpy(s->userdata, (void *)&SENSORS::cfg.mq135_r0, sizeof(double));
  LOG("[MQ-135] ADC initialized on %d", MQ135PIN);
}

void destroy_mq135_adc(sensor_r_t *s){
  // free userdata
  if(s->userdata != NULL){
    free(s->userdata);
    s->userdata = NULL;
  }
}

#endif // SUPPORT_MQ135

#ifdef APDS9930
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#define APDS9930_I2C_ADDRESS 0x39 // default I2C address for APDS-9930
Adafruit_APDS9930 apds(APDS9930_I2C_ADDRESS);
uint16_t apds_r = 0, apds_g = 0, apds_b = 0, apds_c = 0;
double fetch_apds_illuminance(sensor_r_t *s){
  // fetch APDS-9930 illuminance (lux)
  float lux = 0;
  apds.getLux(&lux);
  LOG("[APDS-9930] lux: %.2f", lux);
  return (double)lux;
}

double fetch_apds_color(sensor_r_t *s){
  // fetch APDS-9930 color (returns C, but logs R,G,B,C)
  apds.getRGB(&apds_r, &apds_g, &apds_b, &apds_c);
  LOG("[APDS-9930] RGB: %d,%d,%d,%d", apds_r, apds_g, apds_b, apds_c);
  // For the main value, return clear channel (C)
  return (double)apds_c;
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

#ifdef SUPPORT_S8
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

double fetch_s8_co2(sensor_r_t *s){
  // Fetch CO2 value from S8 sensor
  if(sensor_S8 == NULL) {
    LOG("[S8] sensor not initialized");
    return 0.0;
  }

  sensor.co2 = sensor_S8->get_co2();
  LOG("[S8] CO2 ppm: %d", sensor.co2);
  return (double)sensor.co2;
}
#endif // SUPPORT_S8

#ifdef SUPPORT_SE95
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

double fetch_se95_temperature(sensor_r_t *s) {
  Wire.beginTransmission(SE95_I2C_ADDRESS);
  Wire.write(SE95_TEMPERATURE);
  Wire.endTransmission();
  Wire.requestFrom(SE95_I2C_ADDRESS, 2);
  uint8_t msb, lsb;
  if(Wire.available()){
    msb = Wire.read();
  } else {
    LOG("[SE95] temperature read error, no data available");
    return 0.0; // No data available, return 0.0
  }
  if(Wire.available()){
    lsb = Wire.read();
  } else {
    LOG("[SE95] temperature read error, no data available");
    return 0.0; // No data available, return 0.0
  }
  if(Wire.endTransmission() != ESP_OK){
    LOG("[SE95] temperature read error, end transmission failed");
    return 0.0; // Transmission error, return 0.0
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
  return (double)temp;
}
#endif // SUPPORT_SE95

sensor_r_t all_sensors[NR_OF_SENSORS] = {
    #ifdef SUPPORT_DHT11
    {
      .name = "DHT11 Humidity",
      .unit_fmt = "%s:%s*%%,%.0f\r\n",
      .key  = "humidity",
      .init_function = init_dht11,
      .pre_function = pre_dht11,
      .value_function = dht11_fetch_humidity,
      .post_function = post_dht11,
    },
    {
      .name = "DHT11 Temperature",
      .unit_fmt = "%s:%s*°C,%.2f\r\n",
      .key  = "temperature",
      .init_function = init_dht11,
      .pre_function = pre_dht11,
      .value_function = dht11_fetch_temperature,
      .post_function = post_dht11,
    },
    #else
    {},
    {},
    #endif // SUPPORT_DHT11
    #ifdef SUPPORT_LDR
    {
      .name = "LDR Illuminance",
      .unit_fmt = "%s:%s*lx,%.0f\r\n",
      .key  = "ldr_illuminance",
      .init_function = init_ldr_adc,
      .value_function = fetch_ldr_adc,
    },
    #else
    {},
    #endif // SUPPORT_LDR
    #ifdef SUPPORT_MQ135
    {
      .name = "MQ-135 Air Quality",
      .unit_fmt = "%s:%s*ppm,%.0f\r\n",
      .key  = "air_quality",
      .init_function = init_mq135_adc,
      .value_function = fetch_mq135_adc,
      .destroy_function = destroy_mq135_adc,
    },
    #else
    {},
    #endif // SUPPORT_MQ135
    #ifdef APDS9930
    {
      .name = "APDS-9930 Illuminance",
      .unit_fmt = "%s:%s*lx,%.0f\r\n",
      .key  = "apds_illuminance",
      .init_function = init_apds9930,
      .value_function = fetch_apds_illuminance,
    },
    {
      .name = "APDS-9930 Color",
      .unit_fmt = "%s:%s*C,%.0f\r\n",
      .key  = "apds_color",
      .init_function = init_apds9930,
      .value_function = fetch_apds_color,
    },
    #else
    {},
    {},
    #endif // APDS
    #ifdef SUPPORT_S8
    {
      .name = "S8 CO2",
      .unit_fmt = "%s:%s*ppm,%.0f\r\n",
      .key  = "s8_co2",
      .init_function = init_s8,
      .value_function = fetch_s8_co2,
    },
    #else
    {},
    #endif // SUPPORT_S8
    #ifdef SUPPORT_SE95
    {
      .name = "SE95 Temperature",
      .unit_fmt = "%s:%s*°C,%.5f\r\n",
      .key  = "se95_temperature",
      .init_function = init_se95,
      .value_function = fetch_se95_temperature,
    },
    #else
    {},
    #endif // SUPPORT_SE95
};

NOINLINE
void initialize(){

  // sensors setup
  setup();

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
    if(s->init_function != NULL){
      // call function
      s->init_function(s);
    } else {
      LOG("[SENSORS] Sensor index %d Sensor name %s not configured, skipping setup", i, s->name);
    }
  }

  // config log on UART when VERBOSE=1
  DO_VERBOSE(
    for(int i = 0; i < NR_OF_SENSORS; i++){
      sensor_r_t *s = &SENSORS::all_sensors[i];
      LOG("[SENSORS] Sensor %s log interval (ms): %lu", s->name, s->cfg->v_intv);
      if(s->value_function == NULL)
        LOG("[SENSORS] Sensor index %d Sensor name %s not configured, skipping", i, s->name);
    }
  )

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
      double current_v = s->value_function(s);
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
            LOG("[SENSORS] at_cmd_handler_sensor: checking sensor key '%s' to '%s'", s->key, p);
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
            double current_value = s->value_function(s);
            // Call post function if available
            if(s->post_function != NULL)
                s->post_function(s);
            return AT_R_DOUBLE(current_value);
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
  } else if(p = at_cmd_check("AT+MQ135_R0=", atcmdline, cmd_len)){
    double new_r0 = atof(p);
    if(new_r0 < 1000.0 || new_r0 > 100000.0)
      return AT_R("+ERROR: invalid R0 value (1000-100000)");
    SENSORS::cfg.mq135_r0 = new_r0;
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


} // namespace SENSORS

namespace PLUGINS {
    NOINLINE
    void clear_config(){
        memset((void *)&SENSORS::cfg, 0, sizeof(SENSORS::cfg));
        CFG_SAVE();
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
  AT+MQ135_R0=<value>           - Set MQ-135 R0 resistance value (1000-100000 Ohms)
  AT+MQ135_R0?                  - Get MQ-135 R0 resistance value
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

#ifdef APDS9930
        R"EOF(
  - APDS_ILLUMINANCE            - APDS-9930 light sensor
  - APDS_COLOR                  - APDS-9930 RGB color sensor
)EOF"
#endif // APDS9930

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

