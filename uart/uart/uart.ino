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
#include <algorithm>
#include "SerialCommands.h"
#include "EEPROM.h"
#include "sntp.h"

#define BLUETOOTH_UART_AT

#ifdef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_DEFAULT_PIN "1234"
#define BLUETOOTH_UART_DEVICE_NAME "UART"

#ifdef BT_CLASSIC
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

#endif

#if !defined(BT_BLE) && !defined(BT_CLASSIC)
#undef BLUETOOTH_UART_AT
#endif

#ifdef BT_CLASSIC
/* AT commands over Classic Serial Bluetooth */
BluetoothSerial SerialBT;
char atscbt[128] = {""};
SerialCommands ATScBT(&SerialBT, atscbt, sizeof(atscbt), "\r\n", "\r\n");
#endif

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
  uint16_t main_loop_delay = 100;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
};
cfg_t cfg;

WiFiUDP udp;
IPAddress udp_tgt;
uint8_t valid_udp_host = 0;

/* state flags */
uint8_t ntp_is_synced      = 1;
uint8_t logged_wifi_status = 0;
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
  #ifdef AT_DEBUG
  if(s != NULL) {
    s->GetSerial()->print(F("AT: ["));
    s->GetSerial()->print(atcmdline);
    s->GetSerial()->print(F("], size: "));
    s->GetSerial()->println(cmd_len);
  }
  #endif
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: WiFI SSID max 31 chars"));
      } else {
        ble_send_response("+ERROR: WiFI SSID max 31 chars");
      }
      return;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.wifi_ssid);
    } else {
      ble_send_response(String(cfg.wifi_ssid));
    }
    return;
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: WiFi PASS max 63 chars"));
      } else {
        ble_send_response("+ERROR: WiFi PASS max 63 chars");
      }
      return;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  #ifdef DEBUG
  } else if(p = at_cmd_check("AT+WIFI_PASS?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.wifi_pass);
    } else {
      ble_send_response(String(cfg.wifi_pass));
    }
  #endif
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
    if(s != NULL) {
      s->GetSerial()->println(response);
    } else {
      ble_send_response(response);
    }
    return;
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.do_verbose);
    } else {
      ble_send_response(String(cfg.do_verbose));
    }
  #endif
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.do_log);
    } else {
      ble_send_response(String(cfg.do_log));
    }
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: NTP hostname max 63 chars"));
      } else {
        ble_send_response("+ERROR: NTP hostname max 63 chars");
      }
      return;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.ntp_host);
    } else {
      ble_send_response(String(cfg.ntp_host));
    }
    return;
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    String response;
    if(ntp_is_synced)
      response = "ntp synced";
    else
      response = "not ntp synced";
    if(s != NULL) {
      s->GetSerial()->println(response);
    } else {
      ble_send_response(response);
    }
    return;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.udp_port);
    } else {
      ble_send_response(String(cfg.udp_port));
    }
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: invalid UDP port"));
      } else {
        ble_send_response("+ERROR: invalid UDP port");
      }
      return;
    }
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
      setup_udp();
    }
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.udp_host_ip);
    } else {
      ble_send_response(String(cfg.udp_host_ip));
    }
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= UDP_HOST_IP_MAXLEN){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: invalid udp host ip (too long)"));
      } else {
        ble_send_response("+ERROR: invalid udp host ip (too long)");
      }
      return;
    }
    IPAddress tst;
    if(!tst.fromString(p)){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: invalid udp host ip"));
      } else {
        ble_send_response("+ERROR: invalid udp host ip");
      }
      return;
    }
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_host_ip, p, UDP_HOST_IP_MAXLEN-1);
    cfg.udp_host_ip[UDP_HOST_IP_MAXLEN-1] = '\0';
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_udp();
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0){
      if(s != NULL) {
        s->GetSerial()->println(F("+ERROR: invalid number"));
      } else {
        ble_send_response("+ERROR: invalid number");
      }
      return;
    }
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(cfg.main_loop_delay);
    } else {
      ble_send_response(String(cfg.main_loop_delay));
    }
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    if(s != NULL) {
      s->GetSerial()->println(F("OK"));
    } else {
      ble_send_response("OK");
    }
    resetFunc();
    return;
  } else {
    if(s != NULL) {
      s->GetSerial()->println(F("+ERROR: unknown command"));
    } else {
      ble_send_response("+ERROR: unknown command");
    }
    return;
  }
  if(s != NULL) {
    s->GetSerial()->println(F("OK"));
  } else {
    ble_send_response("OK");
  }
}

// BLE UART Service - Nordic UART Service UUID
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
      #ifdef VERBOSE
      if(cfg.do_verbose) {
        Serial.println("BLE client connected");
      }
      #endif
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      #ifdef VERBOSE
      if(cfg.do_verbose) {
        Serial.println("BLE client disconnected");
      }
      #endif
    }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        // Process each byte individually to handle command terminators properly
        for (size_t i = 0; i < rxValue.length(); i++) {
          if (rxValue[i] == '\n' || rxValue[i] == '\r') {
            // Command terminator found, mark command as ready if buffer is not empty
            if (bleCommandBuffer.length() > 0) {
              bleCommandReady = true;
            }
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
    }
};

void setup_ble() {
  // Create the BLE Device
  BLEDevice::init("UART-BLE");

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

  #ifdef VERBOSE
  if(cfg.do_verbose) {
    Serial.println("BLE UART service started");
    Serial.println("Waiting for a client connection to notify...");
  }
  #endif
}

void handle_ble_command() {
  if (bleCommandReady && bleCommandBuffer.length() > 0) {
    // Process the BLE command using the same AT command handler
    
    if (bleCommandBuffer.startsWith("AT")) {
      // Handle AT command
      at_cmd_handler(NULL, bleCommandBuffer.c_str());
    } else {
      ble_send_response("+ERROR: Invalid command");
    }
    
    bleCommandBuffer = "";
    bleCommandReady = false;
  }
}

void ble_send_response(const String& response) {
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

  // BlueTooth SPP setup possible?
  #ifdef BLUETOOTH_UART_AT
  #ifdef BT_BLE
  setup_ble();
  #endif
  #ifdef BT_CLASSIC
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&at_cmd_handler);
  #endif
  #endif

  // setup WiFi with ssid/pass from EEPROM if set
  setup_wifi();

  // setup UDP if host IP is set
  setup_udp();

  // setup NTP sync to RTC
  #ifdef VERBOSE
  if(cfg.do_verbose){
    if(strlen(cfg.ntp_host) && strlen(cfg.wifi_ssid) && strlen(cfg.wifi_pass)){
      Serial.print(F("will sync with ntp, wifi ssid/pass ok: "));
      Serial.println(cfg.ntp_host);
    }
  }
  #endif
  configTime(0, 0, (char *)&cfg.ntp_host);
}

void loop(){
  #ifdef VERBOSE
  if(cfg.do_verbose)
    Serial.print(F("."));
  #endif

  // Handle Serial AT commands
  if(ATSc.GetSerial()->available()){
    ATSc.ReadSerial();
  } else {
    // no AT command, just continue
    doYIELD;
  }

  // Handle BLE AT commands
  #ifdef BLUETOOTH_UART_AT
  #ifdef BT_BLE
  handle_ble_command();
  
  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    #ifdef VERBOSE
    if(cfg.do_verbose) {
      Serial.println("Start advertising");
    }
    #endif
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  #else
  // no BLE, just continue
  doYIELD;
  #endif
  #endif

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

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_START_EVT){
    #ifdef VERBOSE
    Serial.println(F("BlueTooth UART Initialized SPP"));
    #endif
  } else if(event == ESP_SPP_SRV_OPEN_EVT){
    #ifdef VERBOSE
    Serial.println(F("BlueTooth UART Client connected"));
    #endif
  } else if(event == ESP_SPP_CLOSE_EVT){
    #ifdef VERBOSE
    Serial.println(F("BlueTooth UART Client disconnected"));
    #endif
  } else if(event == ESP_SPP_DATA_IND_EVT){
    #ifdef VERBOSE
    Serial.println(F("BlueTooth UART Data received"));
    #endif
    // any new AT command?
    ATScBT.ReadSerial();
  }
}
#endif

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
  WiFi.setHostname("uart-logger"); // set hostname for the device
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
