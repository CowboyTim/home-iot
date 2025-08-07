#include <Arduino.h>
#include <sys/time.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include "time.h"
#include "SerialCommands.h"
#include "EEPROM.h"
#include "esp_sntp.h"

#ifndef VERBOSE
#define VERBOSE
#endif

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "uart"
#endif

#if defined(DEBUG) || defined(VERBOSE)
void print_time_to_serial(const char *tformat = "[%H:%M:%S]: "){
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  char d_outstr[20];
  strftime(d_outstr, 20, tformat, &gm_new_tm);
  Serial.print(d_outstr);
}
#endif

#ifdef VERBOSE
 #define LOG_TIME_FORMAT "[\%H:\%M:\%S]: "
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define DOLOG(L)    if(cfg.do_verbose){Serial.print(L);}
  #define DOLOGLN(L)  if(cfg.do_verbose){Serial.println(L);}
  #define DOLOGT()    print_time_to_serial(LOG_TIME_FORMAT);
 #else
  #define DOLOG(L)    if(cfg.do_verbose){Serial.print(L);}
  #define DOLOGLN(L)  if(cfg.do_verbose){Serial.println(L);}
  #define DOLOGT()    print_time_to_serial(LOG_TIME_FORMAT);
 #endif
#else
 #define DOLOG(L)
 #define DOLOGLN(L)
 #define DOLOGT()
#endif

#ifdef DEBUG
 #define DEBUG_TIME_FORMAT "[\%H:\%M:\%S]: "
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define DODEBUG(L)    Serial.print(L);
  #define DODEBUGLN(L)  Serial.println(L);
  #define DODEBUGT()    print_time_to_serial(DEBUG_TIME_FORMAT);
 #else
  #define DODEBUG(L)    Serial.print(L);
  #define DODEBUGLN(L)  Serial.println(L);
  #define DODEBUGT()    print_time_to_serial(DEBUG_TIME_FORMAT);
 #endif
#else
 #define DODEBUG(L)
 #define DODEBUGLN(L)
 #define DODEBUGT()
 #define T()
#endif

#define BLUETOOTH_UART_AT

#ifdef BLUETOOTH_UART_AT
#ifndef BLUETOOTH_UART_DEVICE_NAME
#define BLUETOOTH_UART_DEVICE_NAME DEFAULT_HOSTNAME
#endif

#ifdef BT_CLASSIC
#ifndef BLUETOOTH_UART_DEFAULT_PIN
#define BLUETOOTH_UART_DEFAULT_PIN "1234"
#endif
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#warning Bluetooth is not enabled or possible.
#undef BT_CLASSIC
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#warning Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#undef BT_CLASSIC
#endif
#endif

#define BT_BLE
#ifdef BT_BLE
#include <BLEUUID.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#endif
#endif // BLUETOOTH_UART_AT

#if !defined(BT_BLE) && !defined(BT_CLASSIC)
#undef BLUETOOTH_UART_AT
#endif

#ifdef BT_CLASSIC
/* AT commands over Classic Serial Bluetooth */
BluetoothSerial SerialBT;
char atscbt[128] = {""};
SerialCommands ATScBT(&SerialBT, atscbt, sizeof(atscbt), "\r\n", "\r\n");
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

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_DHCP    4

#define UDP_HOST_IP_MAXLEN  40 // enough for IPv6 string

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  uint8_t do_verbose   = 0;
  uint8_t do_debug     = 0;
  uint8_t do_log       = 0;
  uint16_t udp_port    = 0;
  uint16_t main_loop_delay = 100;
  char wifi_ssid[32]   = {0}; // max 31 + 1
  char wifi_pass[64]   = {0}; // nax 63 + 1
  char ntp_host[64]    = {0}; // max hostname + 1
  uint8_t ip_mode      = IPV4_DHCP | IPV6_DHCP;
  char hostname[64]    = {0}; // max hostname + 1
  uint8_t ipv4_addr[4] = {0}; // static IP address
  uint8_t ipv4_gw[4]   = {0}; // static gateway
  uint8_t ipv4_mask[4] = {0}; // static netmask
  uint8_t ipv4_dns[4]  = {0}; // static DNS server
  char udp_host_ip[UDP_HOST_IP_MAXLEN] = {0}; // IPv4 or IPv6 string
  char kvmkey[16]      = "unknown"; // location, max 15 + 1
  double mq135_r0      = 10000.0; // Default R0, configurable via AT command
  unsigned long v_intv[NR_OF_SENSORS]  = {0};
  uint8_t       enabled[NR_OF_SENSORS] = {1};
};
cfg_t cfg;

#define OUTBUFFER_SIZE  128
WiFiUDP udp;
IPAddress udp_tgt;
unsigned long last_v_intv[NR_OF_SENSORS] = {0};
uint8_t valid_udp_host = 0;
char outbuffer[OUTBUFFER_SIZE] = {0};
int h_strl = 0;
uint8_t ntp_is_synced         = 1;
uint8_t logged_wifi_status    = 0;
unsigned long last_wifi_check = 0;

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
  DODEBUGT();
  DODEBUG(F("AT: ["));
  DODEBUG(atcmdline);
  DODEBUG(F("], size: "));
  DODEBUGLN(cmd_len);
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31){
      at_send_response(s, F("+ERROR: WiFI SSID max 31 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    if(strlen(cfg.wifi_ssid) == 0){
      at_send_response(s, F("+ERROR: WiFi SSID not set"));
    } else {
      at_send_response(s, String(cfg.wifi_ssid));
    }
    return;
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: WiFi PASS max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
    uint8_t wifi_stat = WiFi.status();
    String response;
    switch(wifi_stat) {
        case WL_CONNECTED:
          response = "connected";
          break;
        case WL_CONNECT_FAILED:
          response = "failed";
          break;
        case WL_CONNECTION_LOST:
          response = "connection lost";
          break;
        case WL_DISCONNECTED:
          response = "disconnected";
          break;
        case WL_IDLE_STATUS:
          response = "idle";
          break;
        case WL_NO_SSID_AVAIL:
          response = "no SSID configured";
          break;
        default:
          response = String(wifi_stat);
    }
    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+KVMKEY=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 15){
      at_send_response(s, F("+ERROR: Location max 15 chars"));
      return;
    }
    strncpy((char *)&cfg.kvmkey, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+KVMKEY?", atcmdline, cmd_len)){
    at_send_response(s, cfg.kvmkey);
    return;
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.do_verbose));
    return;
  #endif
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.do_log));
    return;
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: NTP hostname max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    if(strlen(cfg.ntp_host) == 0){
      at_send_response(s, F("+ERROR: NTP hostname not set"));
      return;
    }
    at_send_response(s, String(cfg.ntp_host));
    return;
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    String response;
    if(ntp_is_synced)
      response = "ntp synced";
    else
      response = "not ntp synced";
    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.udp_port));
    return;
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0){
      at_send_response(s, F("+ERROR: invalid UDP port"));
      return;
    }
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
      setup_udp();
    }
    return;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)){
    at_send_response(s, cfg.udp_host_ip);
    return;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= UDP_HOST_IP_MAXLEN){
      at_send_response(s, F("+ERROR: invalid udp host ip (too long)"));
      return;
    }
    IPAddress tst;
    if(!tst.fromString(p)){
      at_send_response(s, F("+ERROR: invalid udp host ip"));
      return;
    }
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_host_ip, p, UDP_HOST_IP_MAXLEN-1);
    cfg.udp_host_ip[UDP_HOST_IP_MAXLEN-1] = '\0';
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_udp();
    return;
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0){
      at_send_response(s, F("+ERROR: invalid number"));
      return;
    }
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.main_loop_delay));
    return;
  } else if(p = at_cmd_check("AT+HOSTNAME=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: hostname max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.hostname, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    // Apply hostname immediately if WiFi is connected
    if(WiFi.status() == WL_CONNECTED){
      WiFi.setHostname(cfg.hostname);
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+HOSTNAME?", atcmdline, cmd_len)){
    if(strlen(cfg.hostname) == 0){
      at_send_response(s, F(DEFAULT_HOSTNAME)); // default hostname
    } else {
      at_send_response(s, String(cfg.hostname));
    }
    return;
  } else if(p = at_cmd_check("AT+IPV4=", atcmdline, cmd_len)){
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")){
      // Enable IPv4 DHCP
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_STATIC) | IPV4_DHCP;
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else if(params.equalsIgnoreCase("DISABLE")){
      // Disable IPv4
      cfg.ip_mode &= ~(IPV4_DHCP | IPV4_STATIC);
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else {
      // Parse static IPv4: ip,netmask,gateway[,dns]
      int commaPos1 = params.indexOf(',');
      int commaPos2 = params.indexOf(',', commaPos1 + 1);
      int commaPos3 = params.indexOf(',', commaPos2 + 1);

      if(commaPos1 == -1 || commaPos2 == -1){
        at_send_response(s, F("+ERROR: IPv4 options: DHCP, DISABLE, or ip,netmask,gateway[,dns]"));
        return;
      }

      String ip = params.substring(0, commaPos1);
      String netmask = params.substring(commaPos1 + 1, commaPos2);
      String gateway = params.substring(commaPos2 + 1, commaPos3 == -1 ? params.length() : commaPos3);
      String dns = commaPos3 == -1 ? "8.8.8.8" : params.substring(commaPos3 + 1);

      // Parse IP addresses
      if(!ip.length() || !netmask.length() || !gateway.length()){
        at_send_response(s, F("+ERROR: IPv4 format: ip,netmask,gateway[,dns]"));
        return;
      }

      // Parse and validate IP address
      int ip_parts[4], mask_parts[4], gw_parts[4], dns_parts[4];
      if(sscanf(ip.c_str(), "%d.%d.%d.%d", &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts[3]) != 4 ||
         sscanf(netmask.c_str(), "%d.%d.%d.%d", &mask_parts[0], &mask_parts[1], &mask_parts[2], &mask_parts[3]) != 4 ||
         sscanf(gateway.c_str(), "%d.%d.%d.%d", &gw_parts[0], &gw_parts[1], &gw_parts[2], &gw_parts[3]) != 4 ||
         sscanf(dns.c_str(), "%d.%d.%d.%d", &dns_parts[0], &dns_parts[1], &dns_parts[2], &dns_parts[3]) != 4){
        at_send_response(s, F("+ERROR: invalid IP address format"));
        return;
      }

      // Validate IP ranges (0-255)
      for(int i = 0; i < 4; i++){
        if(ip_parts[i] < 0 || ip_parts[i] > 255 || mask_parts[i] < 0 || mask_parts[i] > 255 ||
           gw_parts[i] < 0 || gw_parts[i] > 255 || dns_parts[i] < 0 || dns_parts[i] > 255){
          at_send_response(s, F("+ERROR: IP address parts must be 0-255"));
          return;
        }
        cfg.ipv4_addr[i] = ip_parts[i];
        cfg.ipv4_mask[i] = mask_parts[i];
        cfg.ipv4_gw[i] = gw_parts[i];
        cfg.ipv4_dns[i] = dns_parts[i];
      }

      // Enable static IPv4
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_DHCP) | IPV4_STATIC;
    }

    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+IPV4?", atcmdline, cmd_len)){
    String response;
    if(cfg.ip_mode & IPV4_DHCP){
      response = "DHCP";
    } else if(cfg.ip_mode & IPV4_STATIC){
      response = String(cfg.ipv4_addr[0]) + "." + String(cfg.ipv4_addr[1]) + "." + 
                 String(cfg.ipv4_addr[2]) + "." + String(cfg.ipv4_addr[3]) + "," +
                 String(cfg.ipv4_mask[0]) + "." + String(cfg.ipv4_mask[1]) + "." +
                 String(cfg.ipv4_mask[2]) + "." + String(cfg.ipv4_mask[3]) + "," +
                 String(cfg.ipv4_gw[0]) + "." + String(cfg.ipv4_gw[1]) + "." +
                 String(cfg.ipv4_gw[2]) + "." + String(cfg.ipv4_gw[3]) + "," +
                 String(cfg.ipv4_dns[0]) + "." + String(cfg.ipv4_dns[1]) + "." +
                 String(cfg.ipv4_dns[2]) + "." + String(cfg.ipv4_dns[3]);
    } else {
      response = "DISABLED";
    }
    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+IPV6=", atcmdline, cmd_len)){
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")){
      // Enable IPv6 DHCP
      cfg.ip_mode |= IPV6_DHCP;
    } else if(params.equalsIgnoreCase("DISABLE")){
      // Disable IPv6
      cfg.ip_mode &= ~IPV6_DHCP;
    } else {
      at_send_response(s, F("+ERROR: IPv6 options: DHCP, DISABLE"));
      return;
    }

    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+IPV6?", atcmdline, cmd_len)){
    String response;
    if(cfg.ip_mode & IPV6_DHCP){
      response = "DHCP";
    } else {
      response = "DISABLED";
    }
    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    at_send_response(s, F("OK"));
    resetFunc();
    return;
  } else if(p = at_cmd_check("AT+MQ135_R0?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.mq135_r0, 2));
    return;
  } else if(p = at_cmd_check("AT+MQ135_R0=", atcmdline, cmd_len)){
    double new_r0 = atof(p);
    if(new_r0 < 1000.0 || new_r0 > 100000.0){
      at_send_response(s, F("+ERROR: invalid R0 value (1000-100000)"));
      return;
    }
    cfg.mq135_r0 = new_r0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    return;
  } else if(p = at_cmd_check("AT+LOG_INTERVAL_", atcmdline, cmd_len)){
    at_cmd_handler_sensor(s, atcmdline, cmd_len);
    return;
  } else if(p = at_cmd_check("AT+ENABLE_", atcmdline, cmd_len)){
    at_cmd_handler_sensor(s, atcmdline, cmd_len);
    return;
  } else {
    at_send_response(s, F("+ERROR: unknown command"));
    return;
  }
}

// BLE UART Service - Nordic UART Service UUID
#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE UART buffer
String bleCommandBuffer = "";
bool bleCommandReady = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DODEBUGT();
      DODEBUGLN(F("BLE client connected"));
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DODEBUGT();
      DODEBUGLN(F("BLE client disconnected"));
    }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      DODEBUGT();
      DODEBUGLN(F("BLE UART Write Callback"));
      DODEBUGT();
      DODEBUG(F("Characteristic Value: >>"));
      DODEBUG(pCharacteristic->getValue().c_str());
      DODEBUGLN(F("<<"));
      DODEBUGT();
      bleCommandBuffer = "";
      DODEBUG(F("BLE Command Buffer START: "));
      DODEBUGLN(bleCommandBuffer);
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        // Process each byte individually to handle command terminators properly
        for (size_t i = 0; i < rxValue.length(); i++) {
          if (rxValue[i] == '\n' || rxValue[i] == '\r') {
            // Command terminator found, mark command as ready if buffer is not empty
            if (bleCommandBuffer.length() > 0)
              bleCommandReady = true;
          } else {
            // Add character to command buffer
            bleCommandBuffer += (char)rxValue[i];
          }

          // Check if command buffer is too long
          if (bleCommandBuffer.length() > 120) {
            // Reset buffer if it's too long without terminator
            bleCommandBuffer = "";
            bleCommandReady = false;
          }
        }
      }
      DODEBUGT();
      DODEBUG(F("BLE Command Buffer: "));
      DODEBUGLN(bleCommandBuffer);
    }
};

void setup_ble() {
  DOLOGLN(F("Setting up BLE"));

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_UART_DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for TX (notifications to client)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE Characteristic for RX (writes from client)
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  DOLOGLN(F("BLE Advertising started, waiting for client connection"));
}

void handle_ble_command() {
  if (bleCommandReady && bleCommandBuffer.length() > 0) {
    // Process the BLE command using the same AT command handler

    if (bleCommandBuffer.startsWith("AT")) {
      // Handle AT command
      at_cmd_handler(NULL, bleCommandBuffer.c_str());
    } else {
      ble_send_response("+ERROR: invalid command");
    }

    bleCommandBuffer = "";
    bleCommandReady = false;
  }
}

void ble_send_response(const String& response) {
  DOLOG(F("Sending BLE response: "));
  DOLOGLN(response);
  if (deviceConnected && pTxCharacteristic) {
    // Send response with line terminator
    String fullResponse = response + "\r\n";

    // Split response into 20-byte chunks (BLE characteristic limit)
    int responseLength = fullResponse.length();
    int offset = 0;

    while (offset < responseLength) {
      int chunkSize = min(20, responseLength - offset);
      String chunk = fullResponse.substring(offset, offset + chunkSize);
      pTxCharacteristic->setValue(chunk.c_str());
      pTxCharacteristic->notify();
      offset += chunkSize;
      delay(10); // Small delay between chunks
    }
  }
}
#endif // BT_BLE

void at_send_response(SerialCommands* s, const String& response) {
  if (s != NULL) {
    s->GetSerial()->println(response);
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  } else {
    ble_send_response(response);
  #endif
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

  // BlueTooth SPP setup possible?
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  setup_ble();
  #endif

  #if defined(BLUETOOTH_UART_AT) && defined(BT_CLASSIC)
  DOLOG(F("Setting up Bluetooth Classic"));
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&at_cmd_handler);
  #endif

  // setup WiFi with ssid/pass from EEPROM if set
  setup_wifi();

  // setup NTP sync if needed
  setup_ntp();

  // setup UDP if host IP is set
  setup_udp();

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
  // DOLOG(F("."));

  // Handle Serial AT commands
  if(ATSc.GetSerial()->available()){
    ATSc.ReadSerial();
  }

  // Handle BLE AT commands
  #ifdef BLUETOOTH_UART_AT
  #ifdef BT_BLE
  handle_ble_command();

  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    // restart advertising
    pServer->startAdvertising();
    DOLOG(F("BLE Restart advertising"));
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  #endif
  #endif

  delay(cfg.main_loop_delay);

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(!logged_wifi_status){
      #ifdef VERBOSE
      if(cfg.do_verbose){
        if(WiFi.status() == WL_CONNECTED){
          DOLOGLN(F("WiFi connected: "));
          DOLOG(F("WiFi ipv4: "));
          DOLOGLN(WiFi.localIP());
          DOLOG(F("WiFi ipv4 gateway: "));
          DOLOGLN(WiFi.gatewayIP());
          DOLOG(F("WiFi ipv4 netmask: "));
          DOLOGLN(WiFi.subnetMask());
          DOLOG(F("WiFi ipv4 DNS: "));
          DOLOGLN(WiFi.dnsIP());
          DOLOG(F("WiFi MAC: "));
          DOLOGLN(WiFi.macAddress());
          DOLOG(F("WiFi RSSI: "));
          DOLOGLN(WiFi.RSSI());
          DOLOG(F("WiFi SSID: "));
          DOLOGLN(WiFi.SSID());
        } else {
          DOLOGLN(F("WiFi not connected"));
        }
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
    cfg.ip_mode = IPV4_DHCP | IPV6_DHCP;
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
        case ARDUINO_EVENT_WIFI_READY:
            DOLOGLN(F("WiFi ready"));
            break;
        case ARDUINO_EVENT_WIFI_STA_START:
            DOLOGLN(F("WiFi STA started"));
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            DOLOGLN(F("WiFi STA stopped"));
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            DOLOG(F("WiFi STA connected to "));
            DOLOGLN(WiFi.SSID());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            DOLOGLN(F("WiFi STA disconnected"));
            break;
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            DOLOGLN(F("WiFi STA auth mode changed"));
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            DOLOGLN("WiFi STA got IPv6: ");
            {
                IPAddress g_ip6 = WiFi.globalIPv6();
                DOLOG(F("Global IPv6: "));
                DOLOGLN(g_ip6.toString());
                IPAddress l_ip6 = WiFi.linkLocalIPv6();
                DOLOG(F("LinkLocal IPv6: "));
                DOLOGLN(l_ip6.toString());
            }
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            DOLOG(F("WiFi STA got IP: "));
            DOLOGLN(WiFi.localIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            DOLOGLN(F("WiFi STA lost IP"));
            break;
        case ARDUINO_EVENT_WPS_ER_SUCCESS:
            DOLOGLN(F("WPS succeeded"));
            break;
        case ARDUINO_EVENT_WPS_ER_FAILED:
            DOLOGLN(F("WPS failed"));
            break;
        case ARDUINO_EVENT_WPS_ER_TIMEOUT:
            DOLOGLN(F("WPS timed out"));
            break;
        case ARDUINO_EVENT_WPS_ER_PIN:
            DOLOGLN(F("WPS PIN received"));
            break;
        default:
            break;
    }
  }
  #endif
}

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_START_EVT){
    DOLOGLN(F("BlueTooth UART Initialized SPP"));
  } else if(event == ESP_SPP_SRV_OPEN_EVT){
    DOLOGLN(F("BlueTooth UART Client connected"));
  } else if(event == ESP_SPP_CLOSE_EVT){
    DOLOGLN(F("BlueTooth UART Client disconnected"));
  } else if(event == ESP_SPP_DATA_IND_EVT){
    DOLOGLN(F("BlueTooth UART Data received"));
    // any new AT command?
    ATScBT.ReadSerial();
  }
}
#endif

void cb_ntp_synced(struct timeval *tv){
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  DOLOG(F("NTP synced, new time: "));
  DOLOG(t);
  char d_outstr[100];
  strftime(d_outstr, 100, ", sync: %a, %b %d %Y %H:%M:%S%z %Z (%s)", &gm_new_tm);
  DOLOGLN(d_outstr);
  ntp_is_synced = 1;
}

void setup_ntp(){
  // if we have a NTP host configured, sync
  if(strlen(cfg.ntp_host)){
    DOLOG(F("will sync with ntp: "));
    DOLOG(cfg.ntp_host);
    DOLOG(F(", interval: "));
    DOLOG(4 * 3600);
    DOLOG(F(", timezone: "));
    DOLOGLN("UTC");
    sntp_set_sync_interval(4 * 3600 * 1000UL);
    sntp_set_time_sync_notification_cb(cb_ntp_synced);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)&cfg.ntp_host);
    sntp_init();
    setenv("TZ", "UTC", 1);
    tzset();
  }
}

void setup_wifi(){
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;
  logged_wifi_status = 0; // reset logged status

  WiFi.disconnect(); // disconnect from any previous connection
  DOLOGLN(F("WiFi setup"));
  WiFi.onEvent(WiFiEvent);

  // IPv4 configuration
  if(cfg.ip_mode & IPV4_DHCP){
    DOLOGLN(F("WiFi Using DHCP for IPv4"));
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  } else if(cfg.ip_mode & IPV4_STATIC){
    DOLOGLN(F("WiFi Using static IPv4 configuration"));
    WiFi.config(IPAddress(cfg.ipv4_addr[0], cfg.ipv4_addr[1], cfg.ipv4_addr[2], cfg.ipv4_addr[3]),
                IPAddress(cfg.ipv4_gw[0], cfg.ipv4_gw[1], cfg.ipv4_gw[2], cfg.ipv4_gw[3]),
                IPAddress(cfg.ipv4_mask[0], cfg.ipv4_mask[1], cfg.ipv4_mask[2], cfg.ipv4_mask[3]),
                IPAddress(cfg.ipv4_dns[0], cfg.ipv4_dns[1], cfg.ipv4_dns[2], cfg.ipv4_dns[3]));
  } else {
    DOLOGLN(F("WiFi Using no IPv4 configuration, assume loopback address"));
    WiFi.config(
      IPAddress(127,0,0,1),
      IPAddress(255,255,255,0),
      IPAddress(127,0,0,1),
      IPAddress(127,0,0,1));
  }

  WiFi.mode(WIFI_STA);
  WiFi.enableSTA(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  if(cfg.hostname){
    WiFi.setHostname(cfg.hostname);
  } else {
    WiFi.setHostname(DEFAULT_HOSTNAME);
  }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // IPv6 configuration
  if(cfg.ip_mode & IPV6_DHCP){
    DOLOGLN(F("WiFi Using DHCP for IPv6"));
    WiFi.enableIPv6(true);
  }

  // connect to Wi-Fi
  DOLOG(F("WiFi Connecting to "));
  DOLOGLN(cfg.wifi_ssid);
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
                at_send_response(s, String(cfg.enabled[i]));
                return;
            }
            if(*p != '=') {
                // error handle
                at_send_response(s, F("+ERROR: Enable command must end with =<0|1> or ?"));
                return;
            }
            p++; // move past '='

            int val = atoi(p);
            if (val != 0 && val != 1) {
                at_send_response(s, F("+ERROR: Enable must be 0 or 1"));
                return;
            }
            cfg.enabled[i] = val;
            EEPROM.put(CFG_EEPROM, cfg);
            EEPROM.commit();
            at_send_response(s, F("OK"));
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
                at_send_response(s, F("+ERROR: Log interval command must end with =<interval>"));
                return;
            }
            p++; // move past '='

            unsigned long new_interval = strtoul(p, NULL, 10);
            if(new_interval < 100){
              at_send_response(s, F("+ERROR: Log interval must be at least 100ms"));
              return;
            }
            cfg.v_intv[i] = new_interval;
            EEPROM.put(CFG_EEPROM, cfg);
            EEPROM.commit();
            at_send_response(s, F("OK"));
            return;
        } else {
            continue; // continue to next sensor
        }
    }
    // if no sensor matched, print error
    at_send_response(s, F("+ERROR: unknown sensor command"));
}
