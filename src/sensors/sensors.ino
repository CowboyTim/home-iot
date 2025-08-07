#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include "SerialCommands.h"
#include "EEPROM.h"
#include "sntp.h"

#ifndef VERBOSE
#define VERBOSE
#endif

/* NTP server to use, can be configured later on via AT commands */
#ifndef DEFAULT_NTP_SERVER
#define DEFAULT_NTP_SERVER "at.pool.ntp.org"
#endif

/* ESP yield */
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
 #define doYIELD yield();
#else
 #define doYIELD
#endif

#ifndef S8
#define S8
#endif
#ifndef SE95
#define SE95
#endif
#ifndef DHT11
#define DHT11
#endif
#ifndef LDR
#define LDR
#endif
#ifndef APDS9930
#undef APDS9930   // TODO: implement software + hardware
#endif
#ifndef MQ135
#define MQ135
#endif

#define NR_OF_SENSORS 9
#define HUMIDITY          0 // DHT11 humidity sensor
#define TEMPERATURE       1 // DHT11 temperature sensor
#define PRESSURE          2 // BMP280 pressure sensor
#define LDR_ILLUMINANCE   3 // LDR illuminance sensor
#define AIR_QUALITY       4 // MQ-135 air quality sensor
#define APDS_ILLUMINANCE  5 // APDS-9930 illuminance sensor
#define APDS_COLOR        6 // APDS-9930 color sensor
#define SE95_TEMPERATURE  7 // SE95 temperature sensor
#define S8_CO2            8 // S8 CO2 sensor

const char *v_key[NR_OF_SENSORS] = {
  "humidity",
  "temperature",
  "pressure",
  "ldr_illuminance",
  "air_quality",
  "apds_illuminance",
  "apds_color",
  "se95_temperature",
  "s8_co2"
};

const char *v_unit[NR_OF_SENSORS] = {
  "%s:%s*%%,%.0f\r\n",             // HUMIDITY
  "%s:%s*°C,%.2f\r\n",             // TEMPERATURE
  "%s:%s*hPa,%.0f\r\n",            // PRESSURE
  "%s:%s*lx,%.0f\r\n",             // LDR ILLUMINANCE
  "%s:%s*ppm,%.0f\r\n",            // AIR_QUALITY
  "%s:%s*lx,%.0f\r\n",             // APDS ILLUMINANCE
  "%s:%s*rgbc,%lu,%lu,%lu,%lu\r\n",// APDS COLOR (R,G,B,C)
  "%s:%s*°C,%.5f\r\n",             // SE95_TEMPERATURE
  "%s:%s*ppm,%.0f\r\n"             // S8_CO2
};

/* our AT commands over UART to config WiFi */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");

#define CFGVERSION 0x01 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00 

#define UDP_HOST_IP_MAXLEN  40 // enough for IPv6 string

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  uint8_t do_verbose   = 0;
  uint8_t do_debug     = 0;
  uint8_t do_log       = 0;
  uint16_t udp_port    = 0;
  char udp_host_ip[UDP_HOST_IP_MAXLEN] = {0}; // IPv4 or IPv6 string
  uint16_t main_loop_delay = 100;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
  char kvmkey[16]      = "unknown"; // location, max 15 + 1
  double mq135_r0      = 10000.0; // Default R0, configurable via AT command
  unsigned long v_intv[NR_OF_SENSORS]  = {0};
  uint8_t       enabled[NR_OF_SENSORS] = {1};
};
cfg_t cfg;

#define OUTBUFFER_SIZE  128
WiFiUDP udp;
IPAddress udp_tgt;
uint8_t valid_udp_host = 0;
char outbuffer[OUTBUFFER_SIZE] = {0};
int h_strl = 0;
uint8_t ntp_is_synced         = 1;
uint8_t logged_wifi_status    = 0;
unsigned long last_wifi_check = 0;
unsigned long last_v_intv[NR_OF_SENSORS] = {0};
void(* resetFunc)(void) = 0;

char* at_cmd_check(const char *cmd, const char *at_cmd, unsigned short at_len){
  unsigned short l = strlen(cmd); /* AT+<cmd>=, or AT, or AT+<cmd>? */
  if(at_len >= l && strncmp(cmd, at_cmd, l) == 0){
    if(*(cmd+l-1) == '='){
      return (char *)at_cmd+l;
    } else {
      return (char *)at_cmd;
    }
  }
  return NULL;
}

void at_cmd_handler(SerialCommands* s, const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  #ifdef AT_DEBUG
  Serial.print(F("AT: ["));
  Serial.print(atcmdline);
  Serial.print(F("], size: "));
  Serial.println(cmd_len);
  #endif
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31){
      s->GetSerial()->println(F("+ERROR: WiFI SSID max 31 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.wifi_ssid);
    return;
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      s->GetSerial()->println(F("+ERROR: WiFi PASS max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  #ifdef DEBUG
  } else if(p = at_cmd_check("AT+WIFI_PASS?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.wifi_pass);
  #endif
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
    uint8_t wifi_stat = WiFi.status();
    switch(wifi_stat) {
        case WL_CONNECTED:
          s->GetSerial()->println(F("connected"));
          break;
        case WL_CONNECT_FAILED:
          s->GetSerial()->println(F("failed"));
          break;
        case WL_CONNECTION_LOST:
          s->GetSerial()->println(F("connection lost"));
          break;
        case WL_DISCONNECTED:
          s->GetSerial()->println(F("disconnected"));
          break;
        case WL_IDLE_STATUS:
          s->GetSerial()->println(F("idle"));
          break;
        case WL_NO_SSID_AVAIL:
          s->GetSerial()->println(F("no SSID configured"));
          break;
        default:
          s->GetSerial()->println(wifi_stat);
    }
    return;
  } else if(p = at_cmd_check("AT+KVMKEY=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 15){
      s->GetSerial()->println(F("+ERROR: Location max 15 chars"));
      return;
    }
    strncpy((char *)&cfg.kvmkey, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+KVMKEY?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.kvmkey);
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.do_verbose);
  #endif
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.do_log);
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      s->GetSerial()->println(F("+ERROR: NTP hostname max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.ntp_host);
    return;
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    if(ntp_is_synced)
      s->GetSerial()->println(F("ntp synced"));
    else
      s->GetSerial()->println(F("not ntp synced"));
    return;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.udp_port);
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0){
      s->GetSerial()->println(F("+ERROR: invalid UDP port"));
      return;
    }
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
      setup_udp();
    }
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.udp_host_ip);
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= UDP_HOST_IP_MAXLEN){
      s->GetSerial()->println(F("+ERROR: invalid udp host ip (too long)"));
      return;
    }
    IPAddress tst;
    if(!tst.fromString(p)){
      s->GetSerial()->println(F("+ERROR: invalid udp host ip"));
      return;
    }
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_host_ip, p, UDP_HOST_IP_MAXLEN-1);
    cfg.udp_host_ip[UDP_HOST_IP_MAXLEN-1] = '\0';
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_udp();
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0){
      s->GetSerial()->println(F("+ERROR: invalid number"));
      return;
    }
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.main_loop_delay);
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    s->GetSerial()->println(F("OK"));
    resetFunc();
    return;
  } else if(p = at_cmd_check("AT+MQ135_R0?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.mq135_r0, 2);
    return;
  } else if(p = at_cmd_check("AT+MQ135_R0?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.mq135_r0, 2);
    return;
  } else if(p = at_cmd_check("AT+MQ135_R0=", atcmdline, cmd_len)){
    double new_r0 = atof(p);
    if(new_r0 < 1000.0 || new_r0 > 100000.0){
      s->GetSerial()->println(F("+ERROR: invalid R0 value (1000-100000)"));
      return;
    }
    cfg.mq135_r0 = new_r0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+LOG_INTERVAL_", atcmdline, cmd_len)){
    at_cmd_handler_sensor(s, atcmdline, cmd_len);
    return;
  } else if(p = at_cmd_check("AT+ENABLE_", atcmdline, cmd_len)){
    at_cmd_handler_sensor(s, atcmdline, cmd_len);
    return;
  } else {
    s->GetSerial()->println(F("+ERROR: unknown command"));
    return;
  }
  s->GetSerial()->println(F("OK"));
  return;
}

void set_v(unsigned long *v, const char *p){
  errno = 0;
  unsigned int l_int = (double)strtoul(p, NULL, 10);
  if(errno != 0){
    ATSc.GetSerial()->println(F("+ERROR: invalid number"));
    return;
  }
  if(l_int < 100){
    ATSc.GetSerial()->println(F("+ERROR: invalid interval, must be at least 100ms"));
    return;
  }
  if(l_int != *(unsigned long *)v){
    *(unsigned long *)v = l_int;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}

#ifdef DHT11
#include <DFRobot_DHT11.h>
#define DHTPIN  A0     // GPIO2/A0 pin for DHT11
DFRobot_DHT11 DHT;
uint8_t did_dht11 = 0; // DHT11 read flag, to avoid multiple reads
double dht11_fetch_humidity(){
  // fetch humidity from DHT11
  if(!did_dht11){
    DHT.read(DHTPIN);
    did_dht11 = 1;
  }
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("DHT11 humidity: "));
    Serial.print(DHT.humidity);
    Serial.println(F(" %"));
  }
  #endif
  double h = (double)DHT.humidity;
  if(h < 0.0 || h > 100.0){
    #ifdef VERBOSE
    if(cfg.do_verbose){
      Serial.print(F("DHT11 humidity out of range, returning 0"));
      Serial.println(h);
    }
    #endif
    h = 0.0;
  }
  return h;
}

double dht11_fetch_temperature(){
  // fetch temperature from DHT11
  if(!did_dht11){
    DHT.read(DHTPIN);
    did_dht11 = 1;
  }
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("DHT11 temperature: "));
    Serial.print(DHT.temperature);
    Serial.println(F(" C"));
  }
  #endif
  double t = (double)DHT.temperature;
  if(t < -40.0 || t > 80.0){
    #ifdef VERBOSE
    if(cfg.do_verbose){
      Serial.print(F("DHT11 temperature out of range, returning 0"));
      Serial.println(t);
    }
    #endif
    t = 0.0;
  }
  return t;
}

void pre_dht11(){
  did_dht11 = 0;
}

void post_dht11(){
  did_dht11 = 0;
}
#endif // DHT11

#ifdef LDR
#define LDRPIN    A1 // GPIO3/A1 pin for LDR
double fetch_ldr_adc(){
  // fetch LDR ADC value
  int ldr_adc = analogReadMilliVolts(A1); // assuming LDR is connected to A0
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("LDR ADC value: "));
    Serial.println(ldr_adc);
  }
  #endif
  double ldr_value = (double)ldr_adc; // convert to double for consistency
  return ldr_value;
}

void init_ldr_adc(){
  // initialize LDR ADC pin
  pinMode(A1, INPUT); // assuming LDR is connected to A1
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.println(F("LDR ADC initialized on A1"));
  #endif
}
#endif // LDR

// MQ-135 Air Quality Sensor
#ifdef MQ135
#define MQ135PIN  A2 // GPIO4/A2 pin for MQ-135
#define MQ135_RL 10000.0 // 10k Ohm load resistor
#define MQ135_VCC 5.0    // Sensor powered by 5V
#define MQ135_ADC_REF 3.3 // ESP32 ADC reference voltage

double mq135_adc_to_ppm(int adc_value) {
  float voltage = (float)adc_value * MQ135_ADC_REF / 4095.0;
  float RS = (MQ135_VCC - voltage) * MQ135_RL / voltage;
  float ratio = RS / cfg.mq135_r0;
  // For CO2: a = 110.47, b = -2.862 (from datasheet)
  float ppm = pow(10, (log10(ratio) - log10(110.47)) / -2.862);
  return ppm;
}

double fetch_mq135_adc(){
  // fetch MQ-135 ADC value
  int mq135_adc = analogRead(MQ135PIN); // raw ADC value (0-4095)
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("MQ-135 ADC value: "));
    Serial.println(mq135_adc);
  }
  #endif
  double ppm = mq135_adc_to_ppm(mq135_adc);
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("MQ-135 CO2 ppm: "));
    Serial.println(ppm);
  }
  #endif
  return ppm;
}

void init_mq135_adc(){
  // initialize MQ-135 ADC pin
  pinMode(MQ135PIN, INPUT);
  analogSetPinAttenuation(MQ135PIN, ADC_11db);
  analogReadResolution(12);
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.println(F("MQ-135 ADC initialized on A2"));
  #endif
}
#endif // MQ135

#ifdef APDS9930
#include <Wire.h>
#include <Adafruit_APDS9930.h>
#define APDS9930_I2C_ADDRESS 0x39 // default I2C address for APDS-9930
Adafruit_APDS9930 apds(APDS9930_I2C_ADDRESS);
uint16_t apds_r = 0, apds_g = 0, apds_b = 0, apds_c = 0;
double fetch_apds_illuminance() {
  // fetch APDS-9930 illuminance (lux)
  float lux = 0;
  apds.getLux(&lux);
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.print(F("APDS-9930 lux: "));
    Serial.println(lux);
  }
  #endif
  return (double)lux;
}

double fetch_apds_color() {
  // fetch APDS-9930 color (returns C, but logs R,G,B,C)
  apds.getRGB(&apds_r, &apds_g, &apds_b, &apds_c);
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.print(F("APDS-9930 RGB: "));
    Serial.print(apds_r); Serial.print(",");
    Serial.print(apds_g); Serial.print(",");
    Serial.print(apds_b); Serial.print(",");
    Serial.println(apds_c);
  }
  #endif
  // For the main value, return clear channel (C)
  return (double)apds_c;
}

void init_apds9930() {
  if(!apds.begin()) {
    #ifdef VERBOSE
    if(cfg.do_verbose)
      Serial.println(F("APDS-9930 not found!"));
    #endif
    return;
  }
  apds.enableColor(true);
  apds.enableLightSensor(true);
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.println(F("APDS-9930 initialized"));
  #endif
  return;
}
#endif // APDS-9930

#ifdef S8
/* Sensair S8 LP sensor for CO2 */
#include "s8_uart.h"
S8_UART *sensor_S8;
S8_sensor sensor;
void init_s8() {
  Serial1.begin(S8_BAUDRATE, SERIAL_8N1, 1, 0);
  sensor_S8 = new S8_UART(Serial1);

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  if(len == 0){
    Serial.println("SenseAir S8 CO2 sensor not found!");
    delete sensor_S8;
    sensor_S8 = NULL;
    cfg.enabled[S8_CO2] = 0; // Disable in config
    return;
  }

  // Show basic S8 sensor info
  Serial.println("| >>> SenseAir S8 NDIR CO2 sensor <<<");
  printf("| Firmware version: %s\n", sensor.firm_version);
  sensor.sensor_id = sensor_S8->get_sensor_ID();
  Serial.print("| Sensor ID: 0x"); printIntToHex(sensor.sensor_id, 4);
  Serial.println("");
  sensor.sensor_type_id = sensor_S8->get_sensor_type_ID();
  Serial.print("| Sensor type: 0x"); printIntToHex(sensor.sensor_type_id, 3);
  Serial.println("");
  sensor.map_version = sensor_S8->get_memory_map_version();
  Serial.print("| Memory map version: ");
  Serial.println(sensor.map_version);
  sensor.abc_period = sensor_S8->get_ABC_period();
  if(sensor.abc_period > 0){
    Serial.print("| ABC (automatic background calibration) period: ");
    Serial.print(sensor.abc_period); Serial.println(" hours");
  } else {
    Serial.println("| ABC (automatic calibration) is disabled");
  }
  Serial.println("| Setting ABC to 0 and wait 1s");
  sensor.abc_period = sensor_S8->set_ABC_period(0);
  delay(1000);
  sensor.abc_period = sensor_S8->get_ABC_period();
  if(sensor.abc_period > 0){
    Serial.print("| ABC (automatic background calibration) period: ");
    Serial.print(sensor.abc_period);
    Serial.println(" hours");
  } else {
    Serial.println("| ABC (automatic calibration) is disabled (0s)");
  }

  // Check the health of the sensor
  Serial.println("| Checking the health of the sensor...");
  sensor.meter_status = sensor_S8->get_meter_status();
  if(sensor.meter_status & S8_MASK_METER_ANY_ERROR) {
    Serial.println("| One or more errors detected!");
    if(sensor.meter_status & S8_MASK_METER_FATAL_ERROR)
      Serial.println("| Fatal error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OFFSET_REGULATION_ERROR)
      Serial.println("| Offset regulation error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_ALGORITHM_ERROR)
      Serial.println("| Algorithm error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OUTPUT_ERROR)
      Serial.println("| Output error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_SELF_DIAG_ERROR)
      Serial.println("| Self diagnostics error in sensor!");
    if(sensor.meter_status & S8_MASK_METER_OUT_OF_RANGE)
      Serial.println("| Out of range in sensor!");
    if(sensor.meter_status & S8_MASK_METER_MEMORY_ERROR)
      Serial.println("| Memory error in sensor!");
  } else {
    Serial.println("| The sensor is OK.");
  }
  return;
}

double fetch_s8_co2() {
  // Fetch CO2 value from S8 sensor
  if(sensor_S8 == NULL) {
    #ifdef VERBOSE
    if(cfg.do_verbose)
        Serial.println(F("S8 sensor not initialized!"));
    #endif
    return 0.0;
  }

  sensor.co2 = sensor_S8->get_co2();
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("S8 CO2 ppm: "));
    Serial.println(sensor.co2);
  }
  #endif
  return (double)sensor.co2;
}
#endif // S8

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
void init_se95() {
  // Initialize SE95 temperature sensor
  Wire.begin();
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.setPins(I2C_DAT, I2C_CLK);
  #endif
  // Fetch temperature from SE95 sensor
  Wire.beginTransmission(SE95_I2C_ADDRESS);
  if(Wire.endTransmission() != ESP_OK){
    cfg.enabled[SE95_TEMPERATURE] = 0; // Disable in config
    #ifdef VERBOSE
    if(cfg.do_verbose)
      Serial.println(F("SE95 sensor not found!"));
    #endif
    return;
  }
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.println(F("SE95 temperature sensor initialized"));
  #endif
  return;
}

double fetch_se95_temperature() {
  Wire.beginTransmission(SE95_I2C_ADDRESS);
  Wire.write(SE95_TEMPERATURE);
  Wire.endTransmission();
  Wire.requestFrom(SE95_I2C_ADDRESS, 2);
  uint8_t msb, lsb;
  if(Wire.available()){
    msb = Wire.read();
  } else {
    #ifdef VERBOSE
    if(cfg.do_verbose)
      Serial.println(F("SE95 temperature read error, no data available"));
    #endif
    return 0.0; // No data available, return 0.0
  }
  if(Wire.available()){
    lsb = Wire.read();
  } else {
    #ifdef VERBOSE
    if(cfg.do_verbose)
      Serial.println(F("SE95 temperature read error, no data available"));
    #endif
    return 0.0; // No data available, return 0.0
  }
  if(Wire.endTransmission() != ESP_OK){
    #ifdef VERBOSE
    if(cfg.do_verbose)
      Serial.println(F("SE95 temperature read error, end transmission failed"));
    #endif
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
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("SE95 temperature: "));
    Serial.print(temp);
    Serial.println(F(" C"));
  }
  #endif
  return (double)temp;
}
#endif // SE95

double (*v_value_function[NR_OF_SENSORS])() = {
#ifdef DHT11
    &dht11_fetch_humidity,    // HUMIDITY
    &dht11_fetch_temperature, // TEMPERATURE
#else
    NULL,                     // HUMIDITY
    NULL,                     // TEMPERATURE
#endif
    NULL,                     // PRESSURE
#ifdef LDR
    &fetch_ldr_adc,           // LDR ILLUMINANCE
#else
    NULL,                     // LDR ILLUMINANCE
#endif
#ifdef MQ135
    &fetch_mq135_adc,         // AIR_QUALITY
#else
    NULL,                     // AIR_QUALITY
#endif
#ifdef APDS9930
    &fetch_apds_illuminance,  // APDS ILLUMINANCE
    &fetch_apds_color,        // APDS COLOR
#else
    NULL,                     // APDS ILLUMINANCE
    NULL,                     // APDS COLOR
#endif
#ifdef SE95
    &fetch_se95_temperature,  // SE95_TEMPERATURE
#else
    NULL,                     // SE95_TEMPERATURE
#endif
#ifdef S8
    &fetch_s8_co2             // S8 CO2
#else
    NULL                      // S8 CO2
#endif
};

void (*v_init_function[NR_OF_SENSORS])() = {
    NULL,               // HUMIDITY
    NULL,               // TEMPERATURE
    NULL,               // PRESSURE
#ifdef LDR
    &init_ldr_adc,      // LDR ILLUMINANCE
#else
    NULL,               // LDR ILLUMINANCE
#endif
#ifdef MQ135
    &init_mq135_adc,    // AIR_QUALITY
#else
    NULL,               // AIR_QUALITY
#endif
#ifdef APDS9930
    &init_apds9930,     // APDS ILLUMINANCE
    &init_apds9930,     // APDS COLOR
#else
    NULL,               // APDS ILLUMINANCE
    NULL,               // APDS COLOR
#endif
#ifdef SE95
    &init_se95,         // SE95_TEMPERATURE
#else
    NULL,               // SE95_TEMPERATURE
#endif
#ifdef S8
    &init_s8            // S8 CO2
#else
    NULL                // S8 CO2
#endif
};

void (*v_pre_function[NR_OF_SENSORS])() = {
#ifdef DHT11
    &pre_dht11,        // HUMIDITY
    &pre_dht11,        // TEMPERATURE
#else
    NULL,              // HUMIDITY
    NULL,              // TEMPERATURE
#endif
    NULL,              // PRESSURE
    NULL,              // LDR ILLUMINANCE
    NULL,              // AIR_QUALITY
    NULL,              // APDS ILLUMINANCE
    NULL,              // APDS COLOR
    NULL,              // SE95_TEMPERATURE
    NULL               // S8 CO2
};

void (*v_post_function[NR_OF_SENSORS])() = {
#ifdef DHT11
    &post_dht11,        // HUMIDITY
    &post_dht11,        // TEMPERATURE
#else
    NULL,               // HUMIDITY
    NULL,               // TEMPERATURE
#endif
    NULL,               // PRESSURE
    NULL,               // LDR ILLUMINANCE
    NULL,               // AIR_QUALITY
    NULL,               // APDS ILLUMINANCE
    NULL,               // APDS COLOR
    NULL,               // SE95_TEMPERATURE
    NULL                // S8 CO2
};

void setup(){
  // Serial setup, init at 115200 8N1
  Serial.begin(115200);

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  ATSc.SetDefaultHandler(&at_cmd_handler);

  // see http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  setenv("TZ", "UTC", 1);
  tzset();

  // setup WiFi with ssid/pass from EEPROM if set
  setup_wifi();

  // setup UDP if host IP is set
  setup_udp();
  #ifdef VERBOSE
  if(cfg.do_verbose){
    if(strlen(cfg.ntp_host) && strlen(cfg.wifi_ssid) && strlen(cfg.wifi_pass)){
      Serial.print(F("will sync with ntp, wifi ssid/pass ok: "));
      Serial.println(cfg.ntp_host);
    }
  }
  #endif

  // setup NTP sync to RTC
  configTime(0, 0, (char *)&cfg.ntp_host);

  // sensors check
  for(int i = 0; i < NR_OF_SENSORS; i++)
    last_v_intv[i] = millis();

  // sensors config check for interval, when 0, assume 1000ms
  for(int i = 0; i < NR_OF_SENSORS; i++){
    if(cfg.v_intv[i] == 0)
      cfg.v_intv[i] = 1000;
    if(cfg.v_intv[i] < 100)
      cfg.v_intv[i] = 100;
  }

  // setup sensors
  for(int i = 0; i < NR_OF_SENSORS; i++){
    if(v_init_function[i] != NULL){
      v_init_function[i]();
    } else {
      #ifdef VERBOSE
      if(cfg.do_verbose){
        Serial.print(F("Sensor index "));
        Serial.print(i);
        Serial.print(F(" Sensor name "));
        Serial.print(v_key[i]);
        Serial.println(F(" not configured, skipping setup"));
      }
      #endif
    }
  }

  // config log on UART when VERBOSE=1
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.println(F("DHT11/BME280 ESP32/ESP8266 logger started"));
    Serial.print(F("NTP server: "));
    Serial.println(cfg.ntp_host);
    Serial.print(F("WiFi SSID: "));
    Serial.println(cfg.wifi_ssid);
    Serial.print(F("VERBOSE: "));
    Serial.println(cfg.do_verbose);
    Serial.print(F("LOG UART: "));
    Serial.println(cfg.do_log);
    Serial.print(F("UDP host: "));
    Serial.print(cfg.udp_host_ip);
    Serial.print(F(":"));
    Serial.println(cfg.udp_port);
    Serial.print(F("KVM key: "));
    Serial.println(cfg.kvmkey);
    Serial.print(F("Main loop delay: "));
    Serial.println(cfg.main_loop_delay);
    for(int i = 0; i < NR_OF_SENSORS; i++){
      Serial.print(F("Sensor "));
      Serial.print(v_key[i]);
      Serial.print(F(" log interval (ms): "));
      Serial.println((unsigned long)cfg.v_intv[i]);
      if(v_value_function[i] == NULL){
        Serial.print(F("Sensor index "));
        Serial.print(i);
        Serial.print(F(" Sensor name "));
        Serial.print(v_key[i]);
        Serial.println(F(" not configured, skipping"));
      }
    }
  }
  #endif
}

void loop(){
  if(ATSc.GetSerial()->available()){
    ATSc.ReadSerial();
  }

  delay(cfg.main_loop_delay);

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(!logged_wifi_status){
      #ifdef VERBOSE
      if(cfg.do_verbose){
        Serial.println(F("WiFi connected: "));
        Serial.print(F("ipv4:"));
        Serial.println(WiFi.localIP());
        Serial.println(WiFi.gatewayIP());
        Serial.print(F("WiFi MAC: "));
        Serial.println(WiFi.macAddress());
        Serial.print(F("WiFi RSSI: "));
        Serial.println(WiFi.RSSI());
        Serial.print(F("WiFi SSID: "));
        Serial.println(WiFi.SSID());
      }
      #endif
      logged_wifi_status = 1;
    }
    last_wifi_check = millis();
  }

  // loop through sensors and call pre function
  for(int i = 0; i < NR_OF_SENSORS; i++){
    if(cfg.enabled[i] == 0) continue;
    doYIELD;
    if(v_pre_function[i] == NULL)
      continue;
    v_pre_function[i]();
  }

  // loop through sensors and check if we need to fetch & log
  for(int i = 0; i < NR_OF_SENSORS; i++){
    if(cfg.enabled[i] == 0) continue;
    doYIELD;
    if(v_value_function[i] == NULL)
        continue;
    if(millis() - last_v_intv[i] > cfg.v_intv[i]){
      double current_v = v_value_function[i]();
      memset((char*)&outbuffer, 0, OUTBUFFER_SIZE);
      h_strl = snprintf((char *)&outbuffer, OUTBUFFER_SIZE, v_unit[i], cfg.kvmkey, v_key[i], current_v);
      if(h_strl > 0){
          // output over UART?
          if(cfg.do_log)
            Serial.print(outbuffer);
          // log to UDP sink?
          if(valid_udp_host && WiFi.status() == WL_CONNECTED){
            udp.beginPacket(udp_tgt, cfg.udp_port);
            udp.write((uint8_t*)&outbuffer, h_strl);
            udp.endPacket();
          }
      } else {
          #ifdef VERBOSE
          if(cfg.do_verbose){
            Serial.print(F("snprintf failed: "));
            Serial.println(strerror(errno));
          }
          #endif
      }
      last_v_intv[i] = millis();
    }
  }

  // loop through sensors and call post function
  for(int i = 0; i < NR_OF_SENSORS; i++){
    if(cfg.enabled[i] == 0) continue;
    doYIELD;
    if(v_post_function[i] == NULL)
      continue;
    v_post_function[i]();
  }
}

void setup_cfg(){
  // EEPROM read
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(CFG_EEPROM, cfg);
  // was (or needs) initialized?
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    // clear
    memset(&cfg, 0, sizeof(cfg));
    // reinit
    cfg.initialized       = CFGINIT;
    cfg.version           = CFGVERSION;
    cfg.do_verbose        = 1;
    cfg.do_log            = 1;
    cfg.main_loop_delay   = 100;
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    cfg.mq135_r0 = 10000.0; // Default R0
    for(int i = 0; i < NR_OF_SENSORS; i++)
      cfg.v_intv[i] = 1000;
    for(int i = 0; i < NR_OF_SENSORS; i++)
      cfg.enabled[i] = 1; // enable all sensors by default
    // write to EEPROM
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}


void WiFiEvent(WiFiEvent_t event){
  #ifdef VERBOSE
  if(cfg.do_verbose){
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println(F("WiFi STA started"));
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.print(F("WiFi STA connected to "));
            Serial.println(WiFi.SSID());
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            Serial.println("STA IPv6");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print(F("WiFi STA got IP: "));
            Serial.println(WiFi.localIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println(F("WiFi STA disconnected"));
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Serial.println(F("WiFi STA stopped"));
            break;
        default:
            break;
    }
  }
  #endif
}

void setup_wifi(){
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;
  logged_wifi_status = 0; // reset logged status

  WiFi.disconnect(); // disconnect from any previous connection
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.println(F("Setting up WiFi..."));
  WiFi.onEvent(WiFiEvent);
  #endif
  WiFi.mode(WIFI_STA); // set WiFi mode to Station
  WiFi.setAutoReconnect(true); // enable auto-reconnect
  WiFi.setSleep(false); // disable WiFi sleep mode
  WiFi.setHostname("dht11-bme280-logger"); // set hostname for the device
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // set WiFi transmit power (optional, adjust as needed)
  WiFi.enableSTA(true); // enable Station mode
  WiFi.enableIPv6(true); // enable IPv6 support

  // connect to Wi-Fi
  #ifdef VERBOSE
  if(cfg.do_verbose){
    Serial.print(F("Connecting to "));
    Serial.println(cfg.wifi_ssid);
  }
  #endif
  WiFi.persistent(false);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
}

void setup_udp(){
  if(udp_tgt.fromString(cfg.udp_host_ip) && cfg.udp_port > 0){
    valid_udp_host = 1;
    #ifdef VERBOSE
    if(cfg.do_verbose){
      Serial.print(F("send counters to "));
      Serial.print(cfg.udp_host_ip);
      Serial.print(F(", port:"));
      Serial.println(cfg.udp_port);
    }
    #endif
  } else {
    valid_udp_host = 0;
    #ifdef VERBOSE
    if(cfg.do_verbose){
      Serial.print(F("udp target host/port is not valid"));
      Serial.println(cfg.udp_host_ip);
    }
    #endif
  }
}

void at_cmd_handler_sensor(SerialCommands *s, const char *at_cmd, unsigned short at_len){
    const char *p = NULL;
    for (int i = 0; i < NR_OF_SENSORS; i++) {
        if (p = at_cmd_check("AT+ENABLE_", at_cmd, at_len)) {
            // move pointer past "AT+ENABLE_"
            p += strlen("AT+ENABLE_");
            // AT+ENABLE_<sensor>=<0|1> or AT+ENABLE_<sensor>?
            #ifdef AT_DEBUG
            Serial.print(F("AT+ENABLE_ command for sensor: "));
            Serial.println(v_key[i]);
            #endif
            // match sensor?
            if(strncasecmp(v_key[i], p, strlen(v_key[i])) != 0) {
                #ifdef AT_DEBUG
                Serial.print(F("Sensor key does not match: "));
                Serial.println(v_key[i]);
                Serial.println(p);
                #endif
                continue; // not matching sensor key
            }
            // move pointer to the = or ? part
            p += strlen(v_key[i]);
            if(*p == '?') {
                // query enable status
                s->GetSerial()->println(cfg.enabled[i]);
                return;
            }
            if(*p != '=') {
                // error handle
                s->GetSerial()->println(F("+ERROR: Enable command must end with =<0|1> or ?"));
                return;
            }
            p++; // move past '='

            int val = atoi(p);
            if (val != 0 && val != 1) {
                s->GetSerial()->println(F("+ERROR: Enable must be 0 or 1"));
                return;
            }
            cfg.enabled[i] = val;
            EEPROM.put(CFG_EEPROM, cfg);
            EEPROM.commit();
            s->GetSerial()->println(F("OK"));
            return;
        } else if (p = at_cmd_check("AT+LOG_INTERVAL_", at_cmd, at_len)){
            // move pointer past "AT+LOG_INTERVAL_"
            p += strlen("AT+LOG_INTERVAL_");
            // AT+LOG_INTERVAL_<sensor>=<interval>
            #ifdef AT_DEBUG
            Serial.print(F("AT+LOG_INTERVAL_ command for sensor: "));
            Serial.println(v_key[i]);
            #endif
            // match sensor?
            if(strncasecmp(v_key[i], p, strlen(v_key[i])) != 0) {
                #ifdef AT_DEBUG
                Serial.print(F("Sensor key does not match: "));
                Serial.println(v_key[i]);
                Serial.println(p);
                #endif
                continue; // not matching sensor key
            }
            // move pointer to the = part
            p += strlen(v_key[i]);
            if(*p != '=') {
                // error handle
                s->GetSerial()->println(F("+ERROR: Log interval command must end with =<interval>"));
                return;
            }
            p++; // move past '='

            unsigned long new_interval = strtoul(p, NULL, 10);
            if(new_interval < 100){
              s->GetSerial()->println(F("+ERROR: Log interval must be at least 100ms"));
              return;
            }
            cfg.v_intv[i] = new_interval;
            EEPROM.put(CFG_EEPROM, cfg);
            EEPROM.commit();
            s->GetSerial()->println(F("OK"));
            return;
        } else {
            continue; // continue to next sensor
        }
    }
    // if no sensor matched, print error
    s->GetSerial()->println(F("+ERROR: unknown command"));
}
