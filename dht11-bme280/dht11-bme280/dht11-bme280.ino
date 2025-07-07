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

#ifdef DHT11
#include <DFRobot_DHT11.h>
#define DHTPIN  A0     // GPIO2/A0 pin for DHT11
DFRobot_DHT11 DHT;
uint8_t did_dht11 = 0; // DHT11 read flag, to avoid multiple reads
#endif

#ifdef APDS9930
#include <Wire.h>
#include <Adafruit_APDS9930.h>
#define APDS9930_I2C_ADDRESS 0x39 // default I2C address for APDS-9930
#endif // APDS9930

#ifdef LDR
#define LDRPIN    A1 // GPIO3/A1 pin for LDR
#endif

#ifdef MQ135
#define MQ135PIN  A2 // GPIO4/A2 pin for MQ-135
#endif

#define NR_OF_SENSORS 7 // increased from 5 to 7
#define HUMIDITY          0
#define TEMPERATURE       1
#define PRESSURE          2
#define LDR_ILLUMINANCE   3
#define AIR_QUALITY       4
#define APDS_ILLUMINANCE  5
#define APDS_COLOR        6

const char *v_key[NR_OF_SENSORS] = {
  "humidity",
  "temperature",
  "pressure",
  "ldr_illuminance",
  "air_quality",
  "apds_illuminance",
  "apds_color"
};

const char *v_unit[NR_OF_SENSORS] = {
  "%s:%s*%%,%.0f\r\n",             // HUMIDITY
  "%s:%s*°C,%.2f\r\n",             // TEMPERATURE
  "%s:%s*hPa,%.0f\r\n",            // PRESSURE
  "%s:%s*lx,%.0f\r\n",             // LDR ILLUMINANCE
  "%s:%s*ppm,%.0f\r\n",            // AIR_QUALITY
  "%s:%s*lx,%.0f\r\n",             // APDS ILLUMINANCE
  "%s:%s*rgbc,%lu,%lu,%lu,%lu\r\n" // APDS COLOR (R,G,B,C)
};

#ifdef APDS9930
Adafruit_APDS9930 apds(APDS9930_I2C_ADDRESS);
uint16_t apds_r = 0, apds_g = 0, apds_b = 0, apds_c = 0;
#endif // APDS9930

/* our AT commands over UART to config WiFi */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");

#define CFGVERSION 0x01 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00 

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  uint8_t do_verbose   = 0;
  uint8_t do_debug     = 0;
  uint8_t do_log       = 0;
  uint16_t udp_port    = 0;
  char udp_host_ip[16] = {0};
  uint16_t main_loop_delay = 100;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
  char kvmkey[16]      = "unknown"; // location, max 15 + 1
  unsigned long v_intv[NR_OF_SENSORS] = {0};
};
cfg_t cfg;

#define OUTBUFFER_SIZE  128
WiFiUDP udp;
IPAddress udp_tgt;
uint8_t valid_udp_host = 0;
char outbuffer[OUTBUFFER_SIZE] = {0};
int h_strl = 0;
uint8_t ntp_is_synced          = 1;
uint8_t logged_wifi_status     = 0;
unsigned long last_wifi_check  = 0;
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
      s->GetSerial()->println(F("WiFI SSID max 31 chars"));
      s->GetSerial()->println(F("ERROR"));
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
      s->GetSerial()->println(F("WiFI password max 63 chars"));
      s->GetSerial()->println(F("ERROR"));
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
  } else if(p = at_cmd_check("AT+HUMIDITY_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[HUMIDITY]);
  } else if(p = at_cmd_check("AT+HUMIDITY_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[HUMIDITY], p);
  } else if(p = at_cmd_check("AT+TEMPERATURE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[TEMPERATURE]);
  } else if(p = at_cmd_check("AT+TEMPERATURE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[TEMPERATURE], p);
  } else if(p = at_cmd_check("AT+PRESSURE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[PRESSURE]);
  } else if(p = at_cmd_check("AT+PRESSURE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[PRESSURE], p);
  } else if(p = at_cmd_check("AT+LDR_ILLUMINANCE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[LDR_ILLUMINANCE]);
  } else if(p = at_cmd_check("AT+LDR_ILLUMINANCE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[LDR_ILLUMINANCE], p);
  } else if(p = at_cmd_check("AT+KVMKEY=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 15){
      s->GetSerial()->println(F("Location max 15 chars"));
      s->GetSerial()->println(F("ERROR"));
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
      s->GetSerial()->println(F("NTP hostname max 63 chars"));
      s->GetSerial()->println(F("ERROR"));
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
      s->GetSerial()->println(F("invalid udp port"));
      s->GetSerial()->println(F("ERROR"));
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
    IPAddress tst;
    if(!tst.fromString(p)){
      s->GetSerial()->println(F("invalid udp host ip"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    strcpy(cfg.udp_host_ip, p);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_udp();
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0){
      s->GetSerial()->println(F("invalid integer"));
      s->GetSerial()->println(F("ERROR"));
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
  } else if(p = at_cmd_check("AT+AIR_QUALITY_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[AIR_QUALITY]);
  } else if(p = at_cmd_check("AT+AIR_QUALITY_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[AIR_QUALITY], p);
  } else if(p = at_cmd_check("AT+APDS_ILLUMINANCE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[APDS_ILLUMINANCE]);
  } else if(p = at_cmd_check("AT+APDS_ILLUMINANCE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[APDS_ILLUMINANCE], p);
  } else if(p = at_cmd_check("AT+APDS_COLOR_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[APDS_COLOR]);
  } else if(p = at_cmd_check("AT+APDS_COLOR_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[APDS_COLOR], p);
  } else {
    s->GetSerial()->println(F("ERROR"));
    return;
  }
  s->GetSerial()->println(F("OK"));
  return;
}

void set_v(unsigned long *v, const char *p){
  errno = 0;
  unsigned int l_int = (double)strtoul(p, NULL, 10);
  if(errno != 0){
    ATSc.GetSerial()->println(F("invalid integer"));
    ATSc.GetSerial()->println(F("ERROR"));
    return;
  }
  if(l_int < 100){
    ATSc.GetSerial()->println(F("interval must be at least 100ms"));
    ATSc.GetSerial()->println(F("ERROR"));
    return;
  }
  if(l_int != *(unsigned long *)v){
    *(unsigned long *)v = l_int;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}

#ifdef DHT11
double dht11_fetch_humidity(){
  // fetch humidity from DHT11
  if(!did_dht11){
    DHT.read(DHTPIN);
    did_dht11 = 1;
  }
  if(cfg.do_log){
    Serial.print(F("DHT11 humidity: "));
    Serial.print(DHT.humidity);
    Serial.println(F(" %"));
  }
  double h = (double)DHT.humidity;
  if(h < 0.0 || h > 100.0){
    if(cfg.do_log){
      Serial.print(F("DHT11 humidity out of range, returning 0"));
      Serial.println(h);
    }
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
  if(cfg.do_log){
    Serial.print(F("DHT11 temperature: "));
    Serial.print(DHT.temperature);
    Serial.println(F(" C"));
  }
  double t = (double)DHT.temperature;
  if(t < -40.0 || t > 80.0){
    if(cfg.do_log){
      Serial.print(F("DHT11 temperature out of range, returning 0"));
      Serial.println(t);
    }
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
double fetch_ldr_adc(){
  // fetch LDR ADC value
  int ldr_adc = analogReadMilliVolts(A1); // assuming LDR is connected to A0
  if(cfg.do_log){
    Serial.print(F("LDR ADC value: "));
    Serial.println(ldr_adc);
  }
  double ldr_value = (double)ldr_adc; // convert to double for consistency
  return ldr_value;
}

void init_ldr_adc(){
  // initialize LDR ADC pin
  pinMode(A1, INPUT); // assuming LDR is connected to A1
  if(cfg.do_log)
    Serial.println(F("LDR ADC initialized on A1"));
}
#endif // LDR

// MQ-135 Air Quality Sensor
#ifdef MQ135
#define MQ135_RL 10000.0 // 10k Ohm load resistor
#define MQ135_R0 10000.0 // Default R0, calibrate for your sensor
#define MQ135_VCC 5.0    // Sensor powered by 5V
#define MQ135_ADC_REF 3.3 // ESP32 ADC reference voltage

double mq135_adc_to_ppm(int adc_value) {
  float voltage = (float)adc_value * MQ135_ADC_REF / 4095.0;
  float RS = (MQ135_VCC - voltage) * MQ135_RL / voltage;
  float ratio = RS / MQ135_R0;
  // For CO2: a = 110.47, b = -2.862 (from datasheet)
  float ppm = pow(10, (log10(ratio) - log10(110.47)) / -2.862);
  return ppm;
}

double fetch_mq135_adc(){
  // fetch MQ-135 ADC value
  int mq135_adc = analogRead(MQ135PIN); // raw ADC value (0-4095)
  if(cfg.do_log){
    Serial.print(F("MQ-135 ADC value: "));
    Serial.println(mq135_adc);
  }
  double ppm = mq135_adc_to_ppm(mq135_adc);
  if(cfg.do_log){
    Serial.print(F("MQ-135 CO2 ppm: "));
    Serial.println(ppm);
  }
  return ppm;
}

void init_mq135_adc(){
  // initialize MQ-135 ADC pin
  pinMode(MQ135PIN, INPUT);
  analogSetPinAttenuation(MQ135PIN, ADC_11db);
  analogReadResolution(12);
  if(cfg.do_log)
    Serial.println(F("MQ-135 ADC initialized on A2"));
}
#endif // MQ135

#ifdef APDS9930
double fetch_apds_illuminance() {
  // fetch APDS-9930 illuminance (lux)
  float lux = 0;
  apds.getLux(&lux);
  if(cfg.do_log){
    Serial.print(F("APDS-9930 lux: "));
    Serial.println(lux);
  }
  return (double)lux;
}

double fetch_apds_color() {
  // fetch APDS-9930 color (returns C, but logs R,G,B,C)
  apds.getRGB(&apds_r, &apds_g, &apds_b, &apds_c);
  if(cfg.do_log){
    Serial.print(F("APDS-9930 RGB: "));
    Serial.print(apds_r); Serial.print(",");
    Serial.print(apds_g); Serial.print(",");
    Serial.print(apds_b); Serial.print(",");
    Serial.println(apds_c);
  }
  // For the main value, return clear channel (C)
  return (double)apds_c;
}

void init_apds9930() {
  if(!apds.begin()) {
    if(cfg.do_log) Serial.println(F("APDS-9930 not found!"));
    return;
  }
  apds.enableColor(true);
  apds.enableLightSensor(true);
  if(cfg.do_log) Serial.println(F("APDS-9930 initialized"));
}
#endif // APDS-9930

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
    &fetch_apds_color         // APDS COLOR
#else
    NULL,                     // APDS ILLUMINANCE
    NULL                      // APDS COLOR
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
    &init_apds9930      // APDS COLOR
#else
    NULL,               // APDS ILLUMINANCE
    NULL                // APDS COLOR
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
    NULL               // APDS COLOR
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
    NULL                // APDS COLOR
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
  // any new AT command? on USB uart
  ATSc.ReadSerial();

  delay(cfg.main_loop_delay);

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(WiFi.status() == WL_CONNECTED){
      if(!logged_wifi_status){
        #ifdef VERBOSE
        if(cfg.do_verbose){
          Serial.print(F("WiFi connected: "));
          Serial.println(WiFi.localIP());
        }
        #endif
        logged_wifi_status = 1;
      }
      if(!valid_udp_host)
        setup_udp();
    } else {
      valid_udp_host = 0;
    }
    last_wifi_check = millis();
  }

  // loop through sensors and call pre function
  for(int i = 0; i < NR_OF_SENSORS; i++){
    doYIELD;
    if(v_pre_function[i] == NULL)
      continue;
    v_pre_function[i]();
  }

  // loop through sensors and check if we need to fetch & log
  for(int i = 0; i < NR_OF_SENSORS; i++){
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
          if(valid_udp_host){
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
    for(int i = 0; i < NR_OF_SENSORS; i++)
      cfg.v_intv[i] = 1000;
    // write
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}

void setup_wifi(){
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;

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
      Serial.print(F(":"));
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

