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
#include <Wire.h>

#ifndef VERBOSE
#define VERBOSE
#endif
#ifndef DEBUG
#define DEBUG
#endif

#define I2C_DAT   6
#define I2C_CLK   7

/* DUCO CO2 Sensor contains a SE95 temperature on i2c, The
   Temperature_LM75_Derived class can read that */
#define TEMP_SENSOR
#ifdef TEMP_SENSOR
#include <Temperature_LM75_Derived.h>
NXP_SE95 se95_temp_sensor;
#endif

/* DUCO CO2 Sensor uses an Sensair S8 LP sensor for CO2 */
#include "s8_uart.h"
S8_UART *sensor_S8;
S8_sensor sensor;

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

/* our AT commands over UART to config WiFi */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");

#define CFGVERSION 0x02 // switch between 0x01/0x02 to reinit the config struct change
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
  unsigned long co2_log_interval  = 0;
  unsigned long temp_log_interval = 0;
  uint16_t main_loop_delay = 0;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
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
unsigned long last_co2        = 0;
#ifdef TEMP_SENSOR
unsigned long last_temp       = 0;
#endif
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
  } else if(p = at_cmd_check("AT+CO2_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.co2_log_interval);
  } else if(p = at_cmd_check("AT+CO2_LOG_INTERVAL=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int l_int = (double)strtoul(p, NULL, 10);
    if(errno != 0){
      s->GetSerial()->println(F("invalid integer"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    if(l_int != cfg.co2_log_interval){
      cfg.co2_log_interval = l_int;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
  } else if(p = at_cmd_check("AT+TEMP_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.temp_log_interval);
  } else if(p = at_cmd_check("AT+TEMP_LOG_INTERVAL=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int l_int = (double)strtoul(p, NULL, 10);
    if(errno != 0){
      s->GetSerial()->println(F("invalid integer"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    if(l_int != cfg.temp_log_interval){
      cfg.temp_log_interval = l_int;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
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
  } else {
    s->GetSerial()->println(F("ERROR"));
    return;
  }
  s->GetSerial()->println(F("OK"));
  return;
}

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

  Serial1.begin(S8_BAUDRATE, SERIAL_8N1, 1, 0);
  sensor_S8 = new S8_UART(Serial1);

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  if(len == 0){
    Serial.println("SenseAir S8 CO2 sensor not found!");
    while(1){
        doYIELD;
        delay(1);
    }
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

  #ifdef TEMP_SENSOR
  Wire.begin();
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.setPins(I2C_DAT, I2C_CLK);
  #endif
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
  }

  // CO2
  if(millis() - last_co2 > cfg.co2_log_interval){
    sensor.co2 = sensor_S8->get_co2();
    memset((char*)&outbuffer, 0, OUTBUFFER_SIZE);
    h_strl = snprintf((char *)&outbuffer, OUTBUFFER_SIZE, "duco:S8_CO2*ppm,%d\r\n", sensor.co2);
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
    }
    last_co2 = millis();
  }

  // Temperature
  #ifdef TEMP_SENSOR
  if(millis() - last_temp > cfg.temp_log_interval){
    double temp_c = se95_temp_sensor.readTemperatureC();
    memset((char*)&outbuffer, 0, OUTBUFFER_SIZE);
    h_strl = snprintf((char *)&outbuffer, OUTBUFFER_SIZE, "duco:SE95_temperature*C,%f\r\n", temp_c);
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
    }
    last_temp = millis();
  }
  #endif
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
    cfg.co2_log_interval  = 1000;
    cfg.temp_log_interval = 1000;
    cfg.main_loop_delay   = 100;
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
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

