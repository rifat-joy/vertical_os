#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <esp_partition.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <EEPROM.h>
#include <HTTPUpdate.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// #include <time.h>
#include <SoftwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/*-------->> RTOS header <<--------*/
#define _1KB 1024
#define MAX_PRIORITY 3
#define cpu_0 0
#define cpu_1 1

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)

/*---> Function macros <---*/
#define INDICATOR_PIN_on() digitalWrite(INDICATOR_PIN, LOW);
#define INDICATOR_PIN_off() digitalWrite(INDICATOR_PIN, HIGH);
#define buzzer()                           \
    digitalWrite(BUZZER_PIN, HIGH);        \
    vTaskDelay(2000 / portTICK_PERIOD_MS); \
    digitalWrite(BUZZER_PIN, LOW);
#define dispenseBlink()                \
    for (int i = 0; i < 5; i++)        \
    {                                  \
        digitalWrite(led_strip, HIGH); \
        delay(200);                    \
        digitalWrite(led_strip, LOW);  \
        delay(200);                    \
    }
#define eepromReset()            \
    for (int n = 0; n < 10; n++) \
    {                            \
        EEPROM.write(n + 1, 0);  \
    }                            \
    EEPROM.commit();

/*---> For realy switching <---*/
#define ON 0
#define OFF 1

/*---> Error Status Codes <---*/
#define OK 200                // Working as it should be
#define spring_error 101      // Spring rotating for more than define period
#define drop_sensor_error 103 // constantly getting feedback

/*---> Transection ID maximum retry count <---*/
#define tran_id_max_retry 5 // Maximum number of retry to send the tran_id back to server after product being dispensed

/*---> For webserver <---*/
#define FORM "<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>RU Setup</title><style> *,::after,::before{box-sizing:border-box}body{margin:0;font-family:'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans';font-size:1rem;font-weight:400;line-height:1.5;color:#212529;background-color:#f5f5f5}.form-control{display:block;width:100%;height:calc(1.5em + .75rem + 2px);border:1px solid #ced4da}button{cursor:pointer;border:1px solid transparent;color:#fff;background-color:#007bff;border-color:#007bff;padding:.5rem 1rem;font-size:1.25rem;line-height:1.5;border-radius:.3rem;width:100%}.form-signin{width:100%;max-width:400px;padding:15px;margin:auto}h1{text-align:center}</style></head><body><main class='form-signin'><form action='/' method='post'><h1 class=''>RU Settings</h1><br/><div class='form-floating'><label>SSID</label><input type='text' class='form-control' name='ssid'></div><div class='form-floating'><br/><label>Password</label><input type='text' class='form-control' name='password'></div><div class='form-floating'><br/><label>RU Tag</label><input type='text' class='form-control' name='ru_tag'></div><div class='form-floating'><br/><label>Ping Topic</label><input type='text' class='form-control' name='ping_topic'></div><div class='form-floating'><br/><label>Drop sensor ideal status</label><input type='number' class='form-control' name='status'></div><div class='form-floating'><br/><label>Mag and Spring Configuration</label><br/><br/><label>Spring in mag 1:</label><input type='number' class='form-control' name='mag_1'><label>Spring in mag 2:</label><input type='number' class='form-control' name='mag_2'><label>Spring in mag 3:</label><input type='number' class='form-control' name='mag_3'><label>Spring in mag 4:</label><input type='number' class='form-control' name='mag_4'><label>Spring in mag 5:</label><input type='number' class='form-control' name='mag_5'><label>Spring in mag 6:</label><input type='number' class='form-control' name='mag_6'><label>Spring in mag 7:</label><input type='number' class='form-control' name='mag_7'><label>Spring in mag 8:</label><input type='number' class='form-control' name='mag_8'></div><br/><br/><button type='submit'>Save</button><p style='text-align: right'></p></form></main></body></html>"
#define RESPONSE "<!doctype html> <html lang='en'> <head> <meta charset='utf-8'> <meta name='viewport' content='width=device-width, initial-scale=1'> <title>RU Setup</title> <style> *,::after,::before { box-sizing: border-box; } body { margin: 0; font-family: 'Segoe UI', Roboto, 'Helvetica Neue', Arial, 'Noto Sans', 'Liberation Sans'; font-size: 1rem; font-weight: 400; line-height: 1.5; color: #212529; background-color: #f5f5f5; } .form-control { display: block; width: 100%; height: calc(1.5em + .75rem + 2px); border: 1px solid #ced4da; } button { border: 1px solid transparent; color: #fff; background-color: #007bff; border-color: #007bff; padding: .5rem 1rem; font-size: 1.25rem; line-height: 1.5; border-radius: .3rem; width: 100%; } .form-signin { width: 100%; max-width: 400px; padding: 15px; margin: auto; } h1, p { text-align: center; } </style> </head> <body> <main class='form-signin'> <h1>RU Setup</h1> <br/> <p>Settings have been saved successfully...!<br />Restarting the device.</p> </main> </body> </html>"
#define ERROR "File Not Found\n\n"

/*---> On demand AP pin <---*/
#define PIN_AP 13 // need to change the pin enable pin is connected to pin 5

/*---> RFID pin definition <---*/
#define RFID_1_RX_PIN 21 // RFID ESP8266 pin number D1
#define RFID_2_RX_PIN 19 // RFID ESP8266 pin number D2
#define RFID_3_RX_PIN 18 // RFID ESP8266 pin number D5
#define RFID_4_RX_PIN 4  // RFID ESP8266 pin number D6
#define RFID_TX_PIN 14   // One common tx pin for all RDM6300 RFID ESP8266 pin number D7

/*---> Relay control pin <---*/
#define ENABLE_PIN 32 // MOTOR ESP8266 pin number D0
#define SELECTOR_A 0  // MOTOR ESP8266 pin number D1
#define SELECTOR_B 2  // MOTOR ESP8266 pin number D2
#define SELECTOR_C 12 // MOTOR ESP8266 pin number D3
#define SELECTOR_D 5

/*---> Other control pin <---*/
#define MOTOR_FEEDBACK_PIN 23 // MOTOR ESP8266 pin number D6
#define BUZZER_PIN 22         // MOTOR ESP8266 pin number D7
#define INDICATOR_PIN 26      // MOTOR ESP8266 pin number D5
// #define MODEM_PIN               33                    // MOTOR ESP8266 pin number D4
#define led_strip 27 // LED strip pin relay K11
#define AudOut 25

/*---> Acesspoint credentials <---*/
const char *ap_ssid = "Vertical_RU";
const char *ap_password = "letmeintoRU";

/*---> Wifi credentials <---*/
String ssid;
String password;
String newHostname = "_Tuki^Taki_"; // To identify the connedted devices in hotspot network

/*---> MQTT credentials <---*/
const char *mqtt_server = "prohorii.vertical-innovations.com";
const char *user = "vilmqtt";
const char *pass = "mvqitlt";
const char *RFID_TOPIC = "jyotilocal";     // For LIVE SERVER
String pingOtaTopic = "testjyotipingtran"; // OTA ping topic.
String ru_tag;                             // Unique RU tag. Source: Dash.vertical-innovations.com
String PING_TOPIC;                         // Ping topic: || 23jyotipingtran for dhaka || 19jyotipingtran for CTG || 52jyotipingtran for Narayanganj
String dispenseTopic;
String pingTranTopic;
String clientId;
String OtaVerCheckTopic;   // Subscribed to Ota version check. If given version doesn't match with current version, an update will occur
String OtaUpdateInfoTopic; // Publish Ota Update info "VIL_DEMO_00motorupdateinfo"

float mag_spring_conf[9] = {0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

/*---> Drop sensor status indicator <---*/
unsigned int drop_sensor_ideal_status;
unsigned int drop_sensor_active_status;

/*---> Motor Auto Shutdown <---*/
const TickType_t max_motor_run_period = pdMS_TO_TICKS(15000); // Unit is in milli seconds
const TickType_t min_motor_run_period = pdMS_TO_TICKS(100);   // Unit is in milli seconds

/*---> Error flags <---*/
bool spring_error_flag = false; // flags error code when spring is rotating more than pre-defined time
bool OK_flag = true;            // indicates all working as expected
bool drop_sensor_flag = false;  // automatically receiving feedback from drop sensor

/*---> Motor Auto Shutdown <---*/
int ds_count = 0;

/*---> Watch Dog Timer <---*/
volatile int tick_ping = 0;  // counter for error ping count
volatile int tick_motor = 0; // counter for long motor run
volatile int tick_modem = 0; // counter for error in modem connectivty
volatile int tick_sntp = 0;  // counter for sntp error (indicates no internet data)

/*---> Varieables to store feedback & which motor to run <---*/
int motor_feedback;
int motor_no_init;

/*---> Message buffer <---*/
char msg[300];
char msgo[300];
char dmsg[300];
char rmsg[300];
char msgRFID[300];

/*---> Varieable to store time <---*/
TickType_t xLastPingTime = 0;
TickType_t xLastMsgTime = 0;
TickType_t xLastOtaMsgTime = 0;

const TickType_t xPingInterval = pdMS_TO_TICKS(5000);
const TickType_t xMsgInterval = pdMS_TO_TICKS(1000);
const TickType_t xOtaMsgInterval = pdMS_TO_TICKS(25000);

/*---> varieable to store ping and post data. Data type example are given in commetns <---*/
int magazine;      // = doc["mag"]; // 1
const char *res;   // = doc["res"]; // 1
int qty;           // = doc["qty"]; // 1
int Info;          // = doc["info"]; // 1
String id;         // = doc["id"]; // "135"
int disabled;      // = doc["disabled"]; // 0
int capacity;      // Maximum Capacity of the magazine. Configured at server side
int remaining_qty; // Current inventory in magazine
int res_flag = 0;  // doc["flag"]; // 0,1,2

/*---> varibles for RFID data read, 5s pause between to consecutive RFID read <---*/
String Text = "";
String NoCard; // No RFID card
boolean state = false;
char c;
unsigned long lastRead = 0;
int isRead = 0;

typedef struct X_STRUCT{
  String rfid_msg;
  String mag;
} xStruct;

// Vertical innovations website certificate
const char *x509CA PROGMEM = R"EOF("
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----

")EOF";

// BearSSL::X509List x509(x509CA);

// DigiCert High Assurance EV Root CA certificate (github) for OTA update
const char trustRoot[] PROGMEM = R"EOF("
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----

")EOF";

// BearSSL::X509List trustRoot_ca(trustRoot);

WiFiClientSecure net;
PubSubClient client(net);
WebServer server(80);
Preferences preferences;
SoftwareSerial RFID_1(RFID_1_RX_PIN, RFID_TX_PIN);
SoftwareSerial RFID_2(RFID_2_RX_PIN, RFID_TX_PIN);
SoftwareSerial RFID_3(RFID_3_RX_PIN, RFID_TX_PIN);
SoftwareSerial RFID_4(RFID_4_RX_PIN, RFID_TX_PIN);

#endif