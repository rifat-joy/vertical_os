#include "main.h"

typedef enum
{
  LOAD_BIT = BIT_0,
  PORTAL_BIT = BIT_1,
  AP_BIT = BIT_2,
  CONFIG_BIT = BIT_3,
  WiFi_BIT = BIT_4,
  MQTT_BIT = BIT_5,
  MQTT_RC_BIT = BIT_6,
  DISPENSE_BIT = BIT_7,
  STATUS_BIT = BIT_8,
  OTA_BIT = BIT_9,
  C_RES_BIT = BIT_10
} EventBits;

/*---> Handelers to control task, Queues & Events <---*/
SemaphoreHandle_t dispenseSemephore, pingTranSemephore, OtaSemephore;
QueueHandle_t RfidQHandle, AudioQHandle;
EventGroupHandle_t EventGroupHandle = NULL;
TaskHandle_t ConfigurationPortal_Handle, ioControl_handle, StartAP_Handle, loadConfig_Handle;
TaskHandle_t WiFi_Handle, MQTT_Handle, MQTTReconnect_Handle, TranIdPost_Handle, ProductDispense_Handle;
TaskHandle_t OTA_Handle, Callbackresponse_Handle, PlayTrack_Handle;
TaskHandle_t rfid_Handle, rfid_publishHandle;

/*Task prototype*/
void WiFiConnectivity(void *pvParameters);
void ConfigurationPortal(void *pvParameters);
void loadConfig(void *pvParameters);
void StartAP(void *pvParameters);
void MQTTConnectivity(void *pvParameters);
void MQTTReconnect(void *pvParameters);
void TranIdPost(void *pvParameters);
void ProductDispense(void *pvParameters);
void ioControl(void *pvParameters);
// void rfid_read(void *pvParameters);
// void rfid_publish(void *pvParameters);
void FirmwareUpdate(void *pvParameters);
void Callbackresponse(void *pvParameters);

/*----->> Control functions <<-----*/
void relay_SetStatus(unsigned char status_1, unsigned char status_2, unsigned char status_3, unsigned char status_4)
{
  digitalWrite(SELECTOR_A, status_1);
  digitalWrite(SELECTOR_B, status_2);
  digitalWrite(SELECTOR_C, status_3);
  digitalWrite(ENABLE_PIN, status_4);
}

void motor_run(int mag_num) // Select which motor to run
{
  if (mag_num == 1)
    relay_SetStatus(ON, ON, ON, ON); // turn on RELAY_1
  if (mag_num == 2)
    relay_SetStatus(OFF, ON, ON, ON); // turn on RELAY_2
  if (mag_num == 3)
    relay_SetStatus(ON, OFF, ON, ON); // turn on RELAY_3
  if (mag_num == 4)
    relay_SetStatus(OFF, OFF, ON, ON); // turn on RELAY_4
  if (mag_num == 5)
    relay_SetStatus(ON, ON, OFF, ON); // turn on RELAY_5
  if (mag_num == 6)
    relay_SetStatus(OFF, ON, OFF, ON); // turn on RELAY_6
  if (mag_num == 7)
    relay_SetStatus(ON, OFF, OFF, ON); // turn on RELAY_7
  if (mag_num == 8)
    relay_SetStatus(OFF, OFF, OFF, ON); // turn on RELAY_8
}

void PlayTrack(char const *Number)
{
  int NumChars = strlen(Number); // could lose this line of put strlen in loop below, but bad form to do so
  Sequence.RemoveAllPlayItems(); // Clear out any previous playlist
  for (int i = 0; i < NumChars; i++)
    // For each number add in the sound for that number to the sequence
    switch (Number[i])
    {
    case '0':
      Sequence.AddPlayItem(&Zero);
      break;
    case '1':
      Sequence.AddPlayItem(&One);
      break;
    case '2':
      Sequence.AddPlayItem(&Two);
      break;
    case '3':
      Sequence.AddPlayItem(&Three);
      break;
    case '4':
      Sequence.AddPlayItem(&Four);
      break;
    case '5':
      Sequence.AddPlayItem(&Five);
      break;
    case '6':
      Sequence.AddPlayItem(&Six);
      break;
    case '7':
      Sequence.AddPlayItem(&Seven);
      break;
    case '8':
      Sequence.AddPlayItem(&Eight);
      break;
    case '9':
      Sequence.AddPlayItem(&Nine);
      break;
    }
  DacAudio.Play(&Sequence); // Play the sequence, will not wait here to complete, works independently of your code
  Serial.println(Number);   // Confirm number entered to the user over the serial
}

/*----->> Callback functions <<-----*/
void handlePortal()
{
  if (server.method() == HTTP_POST)
  {
    Serial.println("Copying the data into NVS");

    preferences.putString("_ssid", server.arg("ssid"));
    preferences.putString("_password", server.arg("password"));
    preferences.putFloat("_version", atof(server.arg("version").c_str()));
    preferences.putString("_ru_tag", server.arg("ru_tag"));
    preferences.putString("_ping_topic", server.arg("ping_topic"));
    preferences.putUInt("status", strtoul(server.arg("status").c_str(), NULL, 0));
    preferences.putFloat("mag_1", atof(server.arg("mag_1").c_str()));
    preferences.putFloat("mag_2", atof(server.arg("mag_2").c_str()));
    preferences.putFloat("mag_3", atof(server.arg("mag_3").c_str()));
    preferences.putFloat("mag_4", atof(server.arg("mag_4").c_str()));
    preferences.putFloat("mag_5", atof(server.arg("mag_5").c_str()));
    preferences.putFloat("mag_6", atof(server.arg("mag_6").c_str()));
    preferences.putFloat("mag_7", atof(server.arg("mag_7").c_str()));
    preferences.putFloat("mag_8", atof(server.arg("mag_8").c_str()));

    Serial.println("Device configurations Saved using Preferences");
    preferences.end();
    PlayTrack("6");
    // Sending web response html file
    server.send(200, "text/html", RESPONSE);
    Serial.println("Settings saved to NVS restarting device");
    delay(3000);
    server.stop();
    ESP.restart();
  }
  else
  {
    // Sending the html from to get user data
    server.send(200, "text/html", FORM);
  }
}

void handleNotFound()
{
  server.send(404, "text/html", ERROR);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String dispenseString = "";
  if (strcmp(topic, dispenseTopic.c_str()) == 0)
  {
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
      dispenseString += (String)(char)payload[i];
    }
    // Serial.println(dispenseString);
    if (dispenseString == "RFID deoesn't exist")
    {
      // Serial.println("play song  ONE");
      PlayTrack("3");
    }
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.println("Failed to deserialize payload");
      return;
    }
    magazine = doc["mag"];                // 1
    qty = doc["qty"];                     // 1
    Info = doc["info"];                   // 1
    id = doc["id"].as<String>();          // "135"
    disabled = doc["disabled"];           // 0
    capacity = doc["capacity"];           // 0
    remaining_qty = doc["remaining_qty"]; // 0
    if (xSemaphoreGive(dispenseSemephore) != pdTRUE)
    {
      Serial.println("Semaphore is not given");
    }
  }

  if (strcmp(topic, pingTranTopic.c_str()) == 0)
  {
    StaticJsonDocument<32> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.println("Failed to deserialize pingtran topic data");
      return;
    }
    res = doc["res"].as<String>(); //
    res_flag = doc["flag"];        // 0
    xEventGroupSetBits(EventGroupHandle, C_RES_BIT);
    /*
       res_flag = doc["flag"]; // 0 == Red INDICATOR_PIN light on
       res_flag = doc["flag"]; // 1 == Reboot
       res_flag = doc["flag"]; // 2 == Green INDICATOR_PIN light on
    */
  }

  if (strcmp(topic, OtaVerCheckTopic.c_str()) == 0)
  { // If version is checked
    char ota_version;
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    for (int n = 0; n < length; n++) // automatically adjust for number of digits
    {
      Serial.println(payload[n]);
      ota_version += (char)payload[n];
    }
    preferences.putFloat("_version", ota_version);
    Serial.println();
  }
}

void setup()
{
  Serial.begin(115200);
  if (!preferences.begin("credentials", false))
  {
    Serial.println("NVS Error");
    return;
  }
  // EEPROM.begin(512); // Initialize EEPROM
  // eepromReset();

  // Serial.print("EEPROM value: ");
  // for (int n = 0; n < 3; n++)
  // {
  //   Serial.print(EEPROM.read(n + 1));
  // }
  // Serial.println("");

  // Serial.print("Firmware version: ");
  // Serial.println(Version);
  RFID_1.begin(9600);
  RFID_2.begin(9600);
  RFID_3.begin(9600);
  RFID_4.begin(9600);
  pinMode(MOTOR_FEEDBACK_PIN, INPUT_PULLUP);
  pinMode(PIN_AP, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(INDICATOR_PIN, OUTPUT);
  pinMode(led_strip, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SELECTOR_D, OUTPUT);
  pinMode(SELECTOR_C, OUTPUT);
  pinMode(SELECTOR_B, OUTPUT);
  pinMode(SELECTOR_A, OUTPUT);
  relay_SetStatus(OFF, OFF, OFF, OFF); // turn off all the relay

  /* Functions and Environment setup*/
  

  EventGroupHandle = xEventGroupCreate();
  // RfidQHandle = xQueueCreate(2, sizeof(xStruct));
  // AudioQHandle = xQueueCreate(5, sizeof(char));
  dispenseSemephore = xSemaphoreCreateBinary();

  if (EventGroupHandle != NULL)
  {
    xTaskCreatePinnedToCore(WiFiConnectivity, "WiFi", (_1KB * 3), NULL, 2, &WiFi_Handle, cpu_0);                     // 456
    xTaskCreatePinnedToCore(ConfigurationPortal, "Portal", (_1KB * 2), NULL, 1, &ConfigurationPortal_Handle, cpu_1); // 1404 -> 1356
    xTaskCreatePinnedToCore(loadConfig, "NVS", (_1KB * 2), NULL, 1, &loadConfig_Handle, cpu_1);                      // 300
    xTaskCreatePinnedToCore(StartAP, "Webserver", (_1KB * 3), NULL, 1, &StartAP_Handle, cpu_1);                      // 2204
    xTaskCreatePinnedToCore(MQTTConnectivity, "MQTT", (_1KB * 2), NULL, 2, &MQTT_Handle, cpu_0);                     // 1404
    xTaskCreatePinnedToCore(MQTTReconnect, "MQTT_RC", (_1KB * 4), NULL, 1, &MQTTReconnect_Handle, cpu_0);            // 732
    xTaskCreatePinnedToCore(ioControl, "io and DS", (_1KB * 2), NULL, 1, &ioControl_handle, cpu_1);                  // 636
    xTaskCreatePinnedToCore(TranIdPost, "TranID", (_1KB * 2), NULL, 1, &TranIdPost_Handle, cpu_1);                   // 288
    xTaskCreatePinnedToCore(ProductDispense, "Dispense", (_1KB * 3), NULL, 1, &ProductDispense_Handle, cpu_1);       // 1200
    // xTaskCreatePinnedToCore(rfid_read, "Rfid Read", (_1KB * 4), &RfidQHandle, 1, &rfid_Handle, cpu_1);
    // xTaskCreatePinnedToCore(rfid_publish, "Rfid publish", (_1KB * 10), &RfidQHandle, 1, &rfid_publishHandle, cpu_1);
    xTaskCreatePinnedToCore(FirmwareUpdate, "OTA", (_1KB * 5), NULL, 1, &OTA_Handle, cpu_1);
    xTaskCreatePinnedToCore(Callbackresponse, "Res", (_1KB), NULL, 1, &Callbackresponse_Handle, cpu_1);
    // xTaskCreatePinnedToCore(FillBuffer, "AudioBuffer", (_1KB), NULL, 1, NULL, cpu_1);
  }
  else
  {
    Serial.println("Task couldn't created successfully");
  }
}

void loop() { DacAudio.FillBuffer(); }

/*<<<------------->>> Task <<<------------->>>*/
void ConfigurationPortal(void *pvParameters)
{
  (void)pvParameters;
  int count = 0;
  xEventGroupSetBits(EventGroupHandle, LOAD_BIT);
  xEventGroupWaitBits(EventGroupHandle, PORTAL_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
  if (ssid.length() == 0 || password.length() == 0)
  {
    Serial.println("Configuration File not found \n Starting Portal");
    xEventGroupSetBits(EventGroupHandle, AP_BIT);
  }
  else
  {
    Serial.println("got credentials from nvs");
    xEventGroupSetBits(EventGroupHandle, CONFIG_BIT);
  }
  // Returns the unused stack of total given stack to task
  Serial.println("Portal->" + String(uxTaskGetStackHighWaterMark(ConfigurationPortal_Handle)));
  for (;;)
  {
    vTaskDelay(1);
    if (!digitalRead(PIN_AP))
    {
      count++;
      Serial.println("Button count:" + String(count));
      Serial.println("Portal->" + String(uxTaskGetStackHighWaterMark(ConfigurationPortal_Handle)));
      if (count == 15)
      {
        count = 0;
        xEventGroupSetBits(EventGroupHandle, AP_BIT);
      }
      delay(200);
    }
    else
    {
      count = 0;
    }
  }
}

void loadConfig(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    // eTaskState state = eTaskGetState(WiFi_Handle);
    if (eTaskGetState(WiFi_Handle) == eSuspended)
    {
      vTaskResume(WiFi_Handle);
    }

    xEventGroupWaitBits(EventGroupHandle, LOAD_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

    ssid = preferences.getString("_ssid", "");
    password = preferences.getString("_password", "");
    Version = preferences.getFloat("_version");
    ru_tag = preferences.getString("_ru_tag", "");
    PING_TOPIC = preferences.getString("_ping_topic", "");
    drop_sensor_ideal_status = preferences.getUInt("status", 0);
    drop_sensor_active_status = (!drop_sensor_ideal_status);
    mag_spring_conf[1] = preferences.getFloat("mag_1");
    mag_spring_conf[2] = preferences.getFloat("mag_2");
    mag_spring_conf[3] = preferences.getFloat("mag_3");
    mag_spring_conf[4] = preferences.getFloat("mag_4");
    mag_spring_conf[5] = preferences.getFloat("mag_5");
    mag_spring_conf[6] = preferences.getFloat("mag_6");
    mag_spring_conf[7] = preferences.getFloat("mag_7");
    mag_spring_conf[8] = preferences.getFloat("mag_8");

    dispenseTopic = ru_tag + "res";
    pingTranTopic = ru_tag + "ptres";
    clientId = ru_tag + "motor";

    OtaVerCheckTopic = ru_tag + "motor" + "update";       // Subscribed to Ota version check. If given version doesn't match with current version, an update will occur
    OtaUpdateInfoTopic = ru_tag + "motor" + "updateinfo"; // Publish Ota Update info "VIL_DEMO_00motorupdateinfo"
    Serial.println("NVS->" + String(uxTaskGetStackHighWaterMark(loadConfig_Handle)));
    xEventGroupSetBits(EventGroupHandle, PORTAL_BIT);
  }
}

void StartAP(void *pvParameters)
{
  (void)pvParameters;

  IPAddress AP_LOCAL_IP(192, 168, 1, 160);
  IPAddress AP_GATEWAY_IP(192, 168, 1, 4);
  IPAddress AP_NETWORK_MASK(255, 255, 255, 0);

  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, AP_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    vTaskSuspend(WiFi_Handle);
    Serial.println("Disconnecting current wifi connection");
    esp_wifi_disconnect();
    // WiFi.disconnect();
    // esp_wifi_deinit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY_IP, AP_NETWORK_MASK))
    {
      Serial.println("AP Config Failed");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (!WiFi.softAP(ap_ssid, ap_password))
    {
      Serial.println("AP Failed");
    }
    Serial.println("Initializing softap");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    server.on("/", handlePortal);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Webserver->" + String(uxTaskGetStackHighWaterMark(StartAP_Handle)));
    PlayTrack("5");
    while (server.method() != HTTP_POST)
    {
      server.handleClient();
    }
  }
}

void WiFiConnectivity(void *pvParameters)
{
  (void)pvParameters;
  volatile int tick_modem = 0; // counter for error in modem connectivty
  xEventGroupWaitBits(EventGroupHandle, CONFIG_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
  // xSemaphoreTake(WiFi_Semephore, portMAX_DELAY);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  Serial.println("TukiTaki Module-> " + ru_tag);
  Serial.println("Connecting to-> " + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(newHostname.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED)
  {
    INDICATOR_PIN_off();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // wifi_status(WiFi.status());
    tick_modem = tick_modem + 1;
    Serial.printf("Tick modem count %d of 120 \n", tick_modem);
    if (tick_modem == 120)
    {
      ESP.restart();
    }
  }
  Serial.println("WiFi connected");

  Serial.println("WiFi-> " + String(uxTaskGetStackHighWaterMark(WiFi_Handle)));
  for (;;)
  {
    xEventGroupSetBits(EventGroupHandle, WiFi_BIT);
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid.c_str(), password.c_str());
      while (WiFi.status() != WL_CONNECTED)
      {
        INDICATOR_PIN_off();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // wifi_status(WiFi.status());
        tick_modem = tick_modem + 1;
        Serial.printf("Tick modem count %d of 120 \n", tick_modem);
        if (tick_modem == 120)
        {
          ESP.restart();
        }
      }
      Serial.println("WiFi connected");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void MQTTConnectivity(void *pvParameters)
{
  (void)pvParameters;
  net.setCACert(x509CA);
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);

  xEventGroupWaitBits(EventGroupHandle, WiFi_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

  xEventGroupSetBits(EventGroupHandle, MQTT_RC_BIT);
  vTaskDelay(1);
  xEventGroupWaitBits(EventGroupHandle, MQTT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

  xLastPingTime = xTaskGetTickCount();
  xLastMsgTime = xTaskGetTickCount();
  xLastOtaMsgTime = xTaskGetTickCount();

  Serial.println("MQTT-> " + String(uxTaskGetStackHighWaterMark(MQTT_Handle)));
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, WiFi_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    if (!client.connected())
    {
      xEventGroupSetBits(EventGroupHandle, MQTT_RC_BIT);
      vTaskDelay(1);
      xEventGroupWaitBits(EventGroupHandle, MQTT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    }

    TickType_t xCurrentTime = xTaskGetTickCount();

    if (xCurrentTime - xLastPingTime > xPingInterval)
    {
      INDICATOR_PIN_off();
    }
    client.loop();

    if (xCurrentTime - xLastMsgTime > xMsgInterval)
    {
      xLastMsgTime = xCurrentTime;
      StaticJsonDocument<32> root;
      root["ru"] = ru_tag;
      serializeJson(root, msg);
      if (client.publish(PING_TOPIC.c_str(), msg))
      {
        xLastPingTime = xCurrentTime;
      }
      else
      {
        Serial.println("Ping Error");
        INDICATOR_PIN_off();
      }
    }
    if (xCurrentTime - xLastOtaMsgTime > xOtaMsgInterval)
    {
      xLastOtaMsgTime = xCurrentTime;
      String msgOTA = ru_tag + "Pinging at version: " + String(Version); // Displays current OTA version

      if (client.publish(pingOtaTopic.c_str(), msgOTA.c_str()))
      {
        // PlayTrack("0");
        Serial.println("  OTA Pinging...");
        Serial.print("Current version: ");
        Serial.println(Version);
      }
      else
      {
        Serial.println("OTA Ping Error");
      }
    }
  }
}

void MQTTReconnect(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, MQTT_RC_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    while (!client.connected())
    {
      Serial.print("Attempting MQTT connection...");
      if (client.connect(clientId.c_str(), user, pass))
      {
        Serial.println("connected");
        client.subscribe(dispenseTopic.c_str());
        client.subscribe(pingTranTopic.c_str());
        client.subscribe(OtaVerCheckTopic.c_str());
        xEventGroupSetBits(EventGroupHandle, MQTT_BIT);
        Serial.println("MQTT_RC-> " + String(uxTaskGetStackHighWaterMark(MQTTReconnect_Handle)));
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        INDICATOR_PIN_off();

        tick_ping = tick_ping + 1;
        Serial.printf("Tick ping count %d of 5\n", tick_ping);

        if (tick_ping == 5) // wait for reconnect to MQTT Server
        {
          ESP.restart();
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
      }
    }
  }
}

void ioControl(void *pvParameters)
{
  (void)pvParameters;

  Serial.println("IO->" + String(uxTaskGetStackHighWaterMark(ioControl_handle)));
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, MQTT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    if (digitalRead(MOTOR_FEEDBACK_PIN) == drop_sensor_active_status)
    {
      ds_count++;
      Serial.println("Into DS Loop");
      Serial.println(ds_count);
      if (ds_count == 20)
      {
        ds_count = 0;
        StaticJsonDocument<32> dstat;
        dstat["ru_tag"] = ru_tag;
        dstat["code"] = drop_sensor_error;
        serializeJson(dstat, dmsg);
        Serial.println(dmsg);
        client.publish("jyotistatuscode", dmsg);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        relay_SetStatus(OFF, OFF, OFF, OFF); // All Relay OFF (Deactivate Demux)
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    else
    {
      ds_count = 0;
    }
  }
}

void TranIdPost(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    xSemaphoreTake(dispenseSemephore, portMAX_DELAY);
    if (magazine > 0)
    {
      Serial.println(magazine);
      // product_dispense(Response.magazine, Response.qty, Response.capacity, Response.remaining_qty);
      xEventGroupSetBits(EventGroupHandle, DISPENSE_BIT);
      vTaskDelay(1);
      xEventGroupWaitBits(EventGroupHandle, STATUS_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
      // vTaskDelay(950 / portTICK_PERIOD_MS);

      StaticJsonDocument<200> roott;
      roott["ru"] = ru_tag;
      roott["tran_id"] = id;
      roott["status_code"] = OK;

      if (spring_error_flag)
      {
        roott["status_code"] = spring_error;
        // Serial.println(spring_error);
        spring_error_flag = false;
        drop_sensor_flag = false;
        OK_flag = true;
      }
      if (drop_sensor_flag)
      {
        // Serial.println(drop_sensor_error);
        roott["status_code"] = drop_sensor_error;
        drop_sensor_flag = false;
        spring_error_flag = false;
        OK_flag = true;
      }

      serializeJson(roott, msgo);
      Serial.println(msgo);
      if (client.publish("jyotitranidpost", msgo))
      {
        Serial.println("Tran Id Posted");
        if (roott["status_code"] == OK)
        {
          PlayTrack("8");
          dispenseBlink();
        }
        else
        {
          PlayTrack("7");
          dispenseBlink();
        }
        roott["status_code"] = OK;
      }
      else
      {
        int tran_id_post_count = 0;
        Serial.println("Tran Id Post Error.");
        Serial.print("Error Count: ");
        Serial.println(tran_id_post_count);
        bool tran_id_succes_status_flag = false;
        while (!tran_id_succes_status_flag)
        {
          if (client.publish("jyotitranidpost", msgo))
          {
            Serial.println("Tran Id Posted");
            tran_id_succes_status_flag = true;
            Serial.println("Tran Id Post Error.");
          }
          else
          {
            tran_id_post_count++;
            if (tran_id_post_count >= tran_id_max_retry)
            {
              tran_id_succes_status_flag = true;
              Serial.println("Tran Id Post Error.");
            } // if (tran_id_post_count >= tran_id_max_retry)
          }
        } // while (!tran_id_succes_status_flag)
      }
    }
    Serial.println("TrabID-> " + String(uxTaskGetStackHighWaterMark(TranIdPost_Handle)));
  }
}

void ProductDispense(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, DISPENSE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    PlayTrack("2");
    for (; qty > 0; qty--)
    {
      float magazine_spring = ceil((remaining_qty * mag_spring_conf[magazine]) / capacity);
      Serial.printf("magazine_spring before CEIL: %2.2f\n", magazine_spring);
      motor_no_init = int(magazine_spring);
      for (int j = 0; j < magazine; j++)
      {
        motor_no_init = motor_no_init + int(mag_spring_conf[j]);
      }
      Serial.printf("motor_no_init %d\n", motor_no_init);
      motor_feedback = digitalRead(MOTOR_FEEDBACK_PIN);

      TickType_t previousTick = xTaskGetTickCount();
      if (motor_feedback == drop_sensor_ideal_status)
      {
        // motor_run(motor_no_init);
        switch (motor_no_init)
        {
        case 1:
          relay_SetStatus(ON, ON, ON, ON); // turn on RELAY_1
          break;
        case 2:
          relay_SetStatus(OFF, ON, ON, ON); // turn on RELAY_2
          break;
        case 3:
          relay_SetStatus(ON, OFF, ON, ON); // turn on RELAY_3
          break;
        case 4:
          relay_SetStatus(OFF, OFF, ON, ON); // turn on RELAY_4
          break;
        case 5:
          relay_SetStatus(ON, ON, OFF, ON); // turn on RELAY_5
          break;
        case 6:
          relay_SetStatus(OFF, ON, OFF, ON); // turn on RELAY_6
          break;
        case 7:
          relay_SetStatus(ON, OFF, OFF, ON); // turn on RELAY_7
          break;
        case 8:
          relay_SetStatus(OFF, OFF, OFF, ON); // turn on RELAY_8
          break;
          // default:
          //   break;
        }
        buzzer();
        Serial.println(previousTick);
        Serial.println(xTaskGetTickCount());
        while (motor_feedback == drop_sensor_ideal_status)
        {
          digitalWrite(led_strip, HIGH);
          motor_feedback = digitalRead(MOTOR_FEEDBACK_PIN);
          yield();
          if (xTaskGetTickCount() - previousTick >= max_motor_run_period)
          {
            motor_feedback = drop_sensor_active_status;
            Serial.println("Spring Error");
            spring_error_flag = true;
            OK_flag = false;
          }
        }
      }
      else // Automically receiving drop sensor feedback
      {
        digitalWrite(led_strip, LOW);
        if (xTaskGetTickCount() - previousTick < min_motor_run_period)
        {
          motor_feedback = drop_sensor_active_status;
          Serial.println("Into drop sensor block");
          drop_sensor_flag = true;
          OK_flag = false;
        }
      }
      relay_SetStatus(OFF, OFF, OFF, OFF); // All Relay OFF (Deactivate Demux)
      xEventGroupSetBits(EventGroupHandle, STATUS_BIT);
      vTaskDelay(10);
      remaining_qty = remaining_qty - 1;
      Serial.println();
      Serial.print("Printing motor feedback after the loop ");
      Serial.println(digitalRead(MOTOR_FEEDBACK_PIN));
      motor_feedback = drop_sensor_active_status;
      vTaskDelay(10);
    }
    Serial.println("ProductDispense-> " + String(uxTaskGetStackHighWaterMark(ProductDispense_Handle)));
  }
}

void rfid_read(void *pvParameters)
{
  RfidQHandle = (QueueHandle_t)pvParameters;
  // String rfid_msg;
  xStruct xRFID;
  TickType_t lastRead = 0;
  Serial.println("RFID_Read-> " + String(uxTaskGetStackHighWaterMark(rfid_Handle)));
  for (;;)
  {
    if (RFID_1.available())
    {
      while (RFID_1.available() > 0 && Text.length() < 20) // Quitting Loop After Reading
      {
        vTaskDelay(1);
        c = RFID_1.read();
        Text += c;
        isRead = 1;
      }

      lastRead = xTaskGetTickCount();

      if (isRead == 1)
      {
        Text = Text.substring(1, 11);
        Text = Text.substring(4, 10);
        while ((xTaskGetTickCount() - lastRead) < pdMS_TO_TICKS(5000))
        {
          if (RFID_1.available() > 0)
          {
            RFID_1.read();
          }
        }
        xRFID.rfid_msg = Text;
        xRFID.mag = "1";
        Serial.println(xRFID.rfid_msg);
        xQueueSendToBack(RfidQHandle, &xRFID, portMAX_DELAY);
      }
      isRead = 0;
      Text = "";
      lastRead = xTaskGetTickCount();
      xRFID = {"", ""};
      Serial.println("RFID_Read-> " + String(uxTaskGetStackHighWaterMark(rfid_Handle)));
    }

    if (RFID_2.available())
    {
      while (RFID_2.available() > 0 && Text.length() < 20) // Quitting Loop After Reading
      {
        vTaskDelay(1);
        c = RFID_2.read();
        Text += c;
        isRead = 1;
      }

      lastRead = xTaskGetTickCount();

      if (isRead == 1)
      {
        Text = Text.substring(1, 11);
        Text = Text.substring(4, 10);
        while ((xTaskGetTickCount() - lastRead) < pdMS_TO_TICKS(5000))
        {
          if (RFID_2.available() > 0)
          {
            RFID_2.read();
          }
        }
        // rfid_msg = Text;
        xRFID.rfid_msg = Text;
        xRFID.mag = "2";
        xQueueSendToBack(RfidQHandle, &xRFID, portMAX_DELAY);
      }
      isRead = 0;
      Text = "";
      lastRead = xTaskGetTickCount();
      xRFID = {"", ""};
    }

    if (RFID_3.available())
    {
      while (RFID_3.available() > 0 && Text.length() < 20) // Quitting Loop After Reading
      {
        vTaskDelay(1);
        c = RFID_3.read();
        Text += c;
        isRead = 1;
      }

      lastRead = xTaskGetTickCount();

      if (isRead == 1)
      {
        Text = Text.substring(1, 11);
        Text = Text.substring(4, 10);
        while ((xTaskGetTickCount() - lastRead) < pdMS_TO_TICKS(5000))
        {
          if (RFID_3.available() > 0)
          {
            RFID_3.read();
          }
        }
        // rfid_msg = Text;
        xRFID.rfid_msg = Text;
        xRFID.mag = "3";
        xQueueSendToBack(RfidQHandle, &xRFID, portMAX_DELAY);
      }
      isRead = 0;
      Text = "";
      lastRead = xTaskGetTickCount();
      xRFID = {"", ""};
    }

    if (RFID_4.available())
    {
      while (RFID_4.available() > 0 && Text.length() < 20) // Quitting Loop After Reading
      {
        vTaskDelay(1);
        c = RFID_4.read();
        Text += c;
        isRead = 1;
      }

      lastRead = xTaskGetTickCount();

      if (isRead == 1)
      {
        Text = Text.substring(1, 11);
        Text = Text.substring(4, 10);
        while ((xTaskGetTickCount() - lastRead) < pdMS_TO_TICKS(5000))
        {
          if (RFID_4.available() > 0)
          {
            RFID_4.read();
          }
        }
        // rfid_msg = Text;
        xRFID.rfid_msg = Text;
        xRFID.mag = "4";
        xQueueSendToBack(RfidQHandle, &xRFID, portMAX_DELAY);
      }
      isRead = 0;
      Text = "";
      lastRead = xTaskGetTickCount();
      xRFID = {"", ""};
    }
  }
}

void rfid_publish(void *pvParameters)
{
  RfidQHandle = (QueueHandle_t)pvParameters;
  // String rfid_msg;
  xStruct xRFID;

  // char num[20];
  // int i, r, len, hex = 0;

  for (;;)
  {
    vTaskDelay(1);
    if (xQueueReceive(RfidQHandle, &xRFID, portMAX_DELAY) == pdTRUE)
    {
      String hexString = xRFID.rfid_msg;
      unsigned int decValue = 0;
      int nextInt;

      for (int i = 0; i < hexString.length(); i++)
      {

        nextInt = int(hexString.charAt(i));
        if (nextInt >= 48 && nextInt <= 57)
          nextInt = map(nextInt, 48, 57, 0, 9);
        if (nextInt >= 65 && nextInt <= 70)
          nextInt = map(nextInt, 65, 70, 10, 15);
        if (nextInt >= 97 && nextInt <= 102)
          nextInt = map(nextInt, 97, 102, 10, 15);
        nextInt = constrain(nextInt, 0, 15);

        decValue = (decValue * 16) + nextInt;
      }

      StaticJsonDocument<200> rootRFID;
      rootRFID["ru"] = ru_tag;
      rootRFID["rfid"] = String(decValue);
      rootRFID["mag"] = xRFID.mag;
      // rootRFID.prettyPrintTo(msgRFID, sizeof(msgRFID));
      serializeJson(rootRFID, msgRFID);
      Serial.print(msgRFID);
      client.publish(RFID_TOPIC, msgRFID);
      buzzer();
    }
    Serial.println("RFID_Read-> " + String(uxTaskGetStackHighWaterMark(rfid_publishHandle)));
  }
}

void FirmwareUpdate(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    // xEventGroupWaitBits(EventGroupHandle, OTA_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // Serial.println("OTA-> " + String(uxTaskGetStackHighWaterMark(OTA_Handle)));
    // client.publish(OtaUpdateInfoTopic.c_str(), updateMessage.c_str()); // notify via mqtt
    // float requested_version = preferences.getFloat("_version");
    // if (Version < (preferences.getFloat("_version")))
    // {
    //   Serial.println("Update available");
    // }
    
  }
}

void Callbackresponse(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, C_RES_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    if (res == "true")
    {
      switch (res_flag)
      {
      case 0:
      {
        // Serial.println("RES-> " + String(uxTaskGetStackHighWaterMark(Callbackresponse_Handle)));
        Serial.println(res_flag);
        Serial.println("Pinging && Red INDICATOR_PIN light on");
        INDICATOR_PIN_off();
        break;
      }
      case 1:
      {
        // Serial.println("RES-> " + String(uxTaskGetStackHighWaterMark(Callbackresponse_Handle)));
        Serial.print("Res flag ");
        Serial.println(res_flag);
        Serial.println("Pinging && Ready to reboot");
        ESP.restart();
        break;
      }
      case 2:
      {
        // Serial.println("RES-> " + String(uxTaskGetStackHighWaterMark(Callbackresponse_Handle)));
        // Serial.println("Pinging && Green INDICATOR_PIN light on");
        INDICATOR_PIN_on();
        break;
      }
      default:
      {
        Serial.println("Anarchy rips apart the mightiest empires"); // If you have reach this line, reach out to original author of this code :)
        break;
      }
      }
    }
  }
}

void FillBuffer(void *pvParameters)
{

  for (;;)
  {
  }
}