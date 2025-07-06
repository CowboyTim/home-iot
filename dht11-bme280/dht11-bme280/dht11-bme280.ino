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
#ifndef DEBUG
#define DEBUG
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

#define NR_OF_SENSORS 4
#define HUMIDITY      0
#define TEMPERATURE   1
#define PRESSURE      2
#define ILLUMINANCE   3
const char *v_key[NR_OF_SENSORS] = {
  "humidity",
  "temperature",
  "pressure",
  "illuminance"
};

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
  } else if(p = at_cmd_check("AT+TEMP_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[TEMPERATURE]);
  } else if(p = at_cmd_check("AT+TEMP_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[TEMPERATURE], p);
  } else if(p = at_cmd_check("AT+PRESSURE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[PRESSURE]);
  } else if(p = at_cmd_check("AT+PRESSURE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[PRESSURE], p);
  } else if(p = at_cmd_check("AT+ILLUMINANCE_LOG_INTERVAL?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.v_intv[ILLUMINANCE]);
  } else if(p = at_cmd_check("AT+ILLUMINANCE_LOG_INTERVAL=", atcmdline, cmd_len)){
    set_v(&cfg.v_intv[ILLUMINANCE], p);
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
  } else if(p = at_cmd_check("AT+VERSION?", atcmdline, cmd_len)){
    s->GetSerial()->println(F("0.2"
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

void set_v(unsigned long *v, const char *p){
  errno = 0;
  unsigned int l_int = (double)strtoul(p, NULL, 10);
  if(errno != 0){
    ATSc.GetSerial()->println(F("invalid integer"));
    ATSc.GetSerial()->println(F("ERROR"));
    return;
  }
  if(l_int != *(unsigned long *)v){
    *(unsigned long *)v = l_int;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
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

  // sensors check
  for(int i = 0; i < NR_OF_SENSORS; i++)
    last_v_intv[i] = millis();
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

  // HUMIDITY
  if(millis() - last_v_intv[HUMIDITY] > cfg.v_intv[HUMIDITY]){
    double humidity = 0; // TODO!
    memset((char*)&outbuffer, 0, OUTBUFFER_SIZE);
    h_strl = snprintf((char *)&outbuffer, OUTBUFFER_SIZE, "%s:%s*%,%d\r\n", cfg.kvmkey, v_key[HUMIDITY], (int)humidity);
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
    last_v_intv[HUMIDITY] = millis();
  }

  // TEMPERATURE
  if(millis() - last_v_intv[TEMPERATURE] > cfg.v_intv[TEMPERATURE]){
    double temp_c = 0; // TODO!
    memset((char*)&outbuffer, 0, OUTBUFFER_SIZE);
    h_strl = snprintf((char *)&outbuffer, OUTBUFFER_SIZE, "%s:%s*°C,%f\r\n", cfg.kvmkey, v_key[TEMPERATURE], temp_c);
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
    last_v_intv[TEMPERATURE] = millis();
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
    for(int i = 0; i < NR_OF_SENSORS; i++)
      cfg.v_intv[i] = 0;
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
