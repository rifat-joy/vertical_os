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
#include <time.h>
#include <SoftwareSerial.h>

/*-------->> RTOS header <<--------*/
#define _1KB 1024
#define MAX_PRIORITY 3
#define cpu_0 0
#define cpu_1 1

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

#endif