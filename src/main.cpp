#include "main.h"

const char *Version = "3.1";

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
  STATUS_BIT = BIT_8
} EventBits;

/*---> Handelers to control task, Queues & Events <---*/
SemaphoreHandle_t dispenseSemephore, pingTranSemephore, OtaSemephore;
QueueHandle_t RfidQHandle;
EventGroupHandle_t EventGroupHandle = NULL;
TaskHandle_t ConfigurationPortal_Handle, ioControl_handle, StartAP_Handle, loadConfig_Handle;
TaskHandle_t WiFi_Handle, MQTT_Handle, MQTTReconnect_Handle, TranIdPost_Handle, ProductDispense_Handle;
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
void rfid_read(void *pvParameters);
void rfid_publish(void *pvParameters);

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

/*----->> Callback functions <<-----*/
void handlePortal()
{
  if (server.method() == HTTP_POST)
  {
    Serial.println("Copying the data into NVS");

    preferences.putString("_ssid", server.arg("ssid"));
    preferences.putString("_password", server.arg("password"));
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

    // Sending web response html file
    server.send(200, "text/html", RESPONSE);
    Serial.println("Settings saved to SPIFFS restarting device");
    delay(1000);
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
      Serial.println("play song  ONE");
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
}

void setup()
{
  Serial.begin(115200);
  if (!preferences.begin("credentials", false))
  {
    Serial.println("NVS Error");
    return;
  }
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

  EventGroupHandle = xEventGroupCreate();
  RfidQHandle = xQueueCreate(2, sizeof(xStruct));
  dispenseSemephore = xSemaphoreCreateBinary();

  if (EventGroupHandle != NULL)
  {
    xTaskCreatePinnedToCore(WiFiConnectivity, "WiFi", (_1KB * 2), NULL, 2, &WiFi_Handle, cpu_0);                     // 456
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
  }
  else
  {
    Serial.println("Task couldnt created successfully");
  }
}

void loop() {}

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
    WiFi.disconnect();
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
    // xEventGroupWaitBits(EventGroupHandle, OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
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
        roott["status_code"] = OK;
        dispenseBlink();
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
  int Q, RQ;
  for (;;)
  {
    xEventGroupWaitBits(EventGroupHandle, DISPENSE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
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
        motor_run(motor_no_init);
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
      xEventGroupSetBits(EventGroupHandle, STATUS_BIT);
      vTaskDelay(10);
      relay_SetStatus(OFF, OFF, OFF, OFF); // All Relay OFF (Deactivate Demux)
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
        xQueueSendToBack(RfidQHandle, &xRFID, portMAX_DELAY);
      }
      isRead = 0;
      Text = "";
      lastRead = xTaskGetTickCount();
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
    }
  }
}

void rfid_publish(void *pvParameters)
{
  RfidQHandle = (QueueHandle_t)pvParameters;
  // String rfid_msg;
  xStruct xRFID;

  char num[20];
  int i, r, len, hex = 0;

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
