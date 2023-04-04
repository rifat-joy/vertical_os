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
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"

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
#define INDICATOR_PIN_on()  digitalWrite(INDICATOR_PIN, LOW);
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
#define ON          0
#define OFF         1

/*---> Error Status Codes <---*/
#define OK                      200         // Working as it should be
#define spring_error            101         // Spring rotating for more than define period
#define drop_sensor_error       103         // constantly getting feedback

/*---> Transection ID maximum retry count <---*/
#define tran_id_max_retry       5 // Maximum number of retry to send the tran_id back to server after product being dispensed

/*---> For webserver <---*/
#define FORM "<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>RU Setup</title><style> *,::after,::before{box-sizing:border-box}body{margin:0;font-family:'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans';font-size:1rem;font-weight:400;line-height:1.5;color:#212529;background-color:#f5f5f5}.form-control{display:block;width:100%;height:calc(1.5em + .75rem + 2px);border:1px solid #ced4da}button{cursor:pointer;border:1px solid transparent;color:#fff;background-color:#007bff;border-color:#007bff;padding:.5rem 1rem;font-size:1.25rem;line-height:1.5;border-radius:.3rem;width:100%}.form-signin{width:100%;max-width:400px;padding:15px;margin:auto}h1{text-align:center}</style></head><body><main class='form-signin'><form action='/' method='post'><h1 class=''>RU Settings</h1><br/><div class='form-floating'><label>SSID</label><input type='text' class='form-control' name='ssid'></div><div class='form-floating'><br/><label>Password</label><input type='text' class='form-control' name='password'></div><div class='form-floating'><br/><label>RU Tag</label><input type='text' class='form-control' name='ru_tag'></div><div class='form-floating'><br/><label>Ping Topic</label><input type='text' class='form-control' name='ping_topic'></div><div class='form-floating'><br/><label>Drop sensor ideal status</label><input type='number' class='form-control' name='status'></div><div class='form-floating'><br/><label>Mag and Spring Configuration</label><br/><br/><label>Spring in mag 1:</label><input type='number' class='form-control' name='mag_1'><label>Spring in mag 2:</label><input type='number' class='form-control' name='mag_2'><label>Spring in mag 3:</label><input type='number' class='form-control' name='mag_3'><label>Spring in mag 4:</label><input type='number' class='form-control' name='mag_4'><label>Spring in mag 5:</label><input type='number' class='form-control' name='mag_5'><label>Spring in mag 6:</label><input type='number' class='form-control' name='mag_6'><label>Spring in mag 7:</label><input type='number' class='form-control' name='mag_7'><label>Spring in mag 8:</label><input type='number' class='form-control' name='mag_8'></div><br/><br/><button type='submit'>Save</button><p style='text-align: right'></p></form></main></body></html>"
#define RESPONSE "<!doctype html> <html lang='en'> <head> <meta charset='utf-8'> <meta name='viewport' content='width=device-width, initial-scale=1'> <title>RU Setup</title> <style> *,::after,::before { box-sizing: border-box; } body { margin: 0; font-family: 'Segoe UI', Roboto, 'Helvetica Neue', Arial, 'Noto Sans', 'Liberation Sans'; font-size: 1rem; font-weight: 400; line-height: 1.5; color: #212529; background-color: #f5f5f5; } .form-control { display: block; width: 100%; height: calc(1.5em + .75rem + 2px); border: 1px solid #ced4da; } button { border: 1px solid transparent; color: #fff; background-color: #007bff; border-color: #007bff; padding: .5rem 1rem; font-size: 1.25rem; line-height: 1.5; border-radius: .3rem; width: 100%; } .form-signin { width: 100%; max-width: 400px; padding: 15px; margin: auto; } h1, p { text-align: center; } </style> </head> <body> <main class='form-signin'> <h1>RU Setup</h1> <br/> <p>Settings have been saved successfully...!<br />Restarting the device.</p> </main> </body> </html>"
#define ERROR "File Not Found\n\n"

/*---> On demand AP pin <---*/
#define PIN_AP                  13 // need to change the pin enable pin is connected to pin 5

/*---> RFID pin definition <---*/
#define RFID_1_RX_PIN           21      // RFID ESP8266 pin number D1
#define RFID_2_RX_PIN           19      // RFID ESP8266 pin number D2
#define RFID_3_RX_PIN           18      // RFID ESP8266 pin number D5
#define RFID_4_RX_PIN           4       // RFID ESP8266 pin number D6
#define RFID_TX_PIN             14      // One common tx pin for all RDM6300 RFID ESP8266 pin number D7

/*---> Relay control pin <---*/
#define ENABLE_PIN              32      // MOTOR ESP8266 pin number D0
#define SELECTOR_A              0      // MOTOR ESP8266 pin number D1
#define SELECTOR_B              2      // MOTOR ESP8266 pin number D2
#define SELECTOR_C              12      // MOTOR ESP8266 pin number D3
#define SELECTOR_D              5

/*---> Other control pin <---*/
#define MOTOR_FEEDBACK_PIN      23       // MOTOR ESP8266 pin number D6
#define BUZZER_PIN              22       // MOTOR ESP8266 pin number D7
#define INDICATOR_PIN           26       // MOTOR ESP8266 pin number D5
// #define MODEM_PIN               33                    // MOTOR ESP8266 pin number D4
#define led_strip               27       // LED strip pin relay K11
#define AudOut                  25

/*---> Acesspoint credentials <---*/
const char *ap_ssid = "Vertical_RU";
const char *ap_password = "letmeintoRU";

/*---> Wifi credentials <---*/
String ssid;
String password;
String newHostname = "_Tuki^Taki_";        // To identify the connedted devices in hotspot network

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
String OtaVerCheckTopic;                   // Subscribed to Ota version check. If given version doesn't match with current version, an update will occur
String OtaUpdateInfoTopic;                 // Publish Ota Update info "VIL_DEMO_00motorupdateinfo"

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

/*---> Message buffer <---*/
char msg[300];
char msgo[300];
char dmsg[300];
char rmsg[300];
char msgRFID[300];

WiFiClientSecure net;
PubSubClient client(net);
WebServer server(80);
Preferences preferences;

#endif