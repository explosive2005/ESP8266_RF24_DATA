#include <ESP8266WiFi.h>

#include <PubSubClient.h>

#include <WiFiManager.h>

#include <ArduinoOTA.h>

#include <ESP8266mDNS.h>

#include <ESP8266WebServer.h>

#include <SPI.h>

#include <RF24.h>

#include <ArduinoJson.h>

#include <LittleFS.h>



// Debug LED (built-in LED on most ESP8266 boards)

#define LED_PIN LED_BUILTIN



// NRF24L01 pins for ESP8266

#define CE_PIN 4    // D2

#define CSN_PIN 15  // D8



// Configuration flag

bool shouldSaveConfig = false;

bool isConnected = false;



// MQTT Settings structure

struct MQTTSettings {

  char server[40];

  char port[6];

  char username[40];

  char password[40];

} mqttSettings;



// Discovery configurations for each sensor type

struct DiscoveryConfig {

  const char* component;

  const char* device_class;

  const char* unit;

  const char* value_template;

};



const DiscoveryConfig SENSOR_CONFIGS[] = {

  { "sensor", "temperature", "°F", "{{ value_json.temperature }}" },

  { "sensor", "humidity", "%", "{{ value_json.humidity }}" },

  { "sensor", "voltage", "V", "{{ value_json.battery_voltage }}" }

};



// RF24 radio setup

RF24 radio(CE_PIN, CSN_PIN);

const uint64_t pipes[6] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL,

                            0xF0F0F0F0E4LL, 0xF0F0F0F0E5LL, 0xF0F0F0F0E6LL };



// MQTT client

WiFiClient espClient;

PubSubClient client(espClient);

const char* discovery_prefix = "homeassistant";



// Web server for configuration

ESP8266WebServer webServer(80);



// Sensor data structure - MUST MATCH THE SENSOR NODE'S TRANSMISSION EXACTLY

struct SensorData {

  uint16_t humidity;        // Humidity in 0.01% units (e.g., 55.23% -> 5523)

  uint16_t temperature;     // Temperature in 0.01°F units (e.g., 75.50°F -> 7550)

  uint16_t batteryVoltage;  // Battery voltage in mV (e.g., 3.3V -> 3300)

  uint8_t sensorID;         // Unique ID of the sensor

};



// --- MQTT Reconnection Variables ---

long lastMqttReconnectAttempt = 0;

const long MQTT_RECONNECT_INTERVAL = 5000;  // Try to reconnect every 5 seconds



// Callback notifying us of the need to save config

void saveConfigCallback() {

  Serial.println("Should save config");

  shouldSaveConfig = true;

}



// Save MQTT configuration to LittleFS

void saveMQTTConfig() {

  Serial.println("Saving config");

  DynamicJsonDocument doc(1024);



  doc["mqtt_server"] = mqttSettings.server;

  doc["mqtt_port"] = mqttSettings.port;

  doc["mqtt_user"] = mqttSettings.username;

  doc["mqtt_pass"] = mqttSettings.password;



  File configFile = LittleFS.open("/config.json", "w");

  if (!configFile) {

    Serial.println("Failed to open config file for writing");

    return;

  }



  serializeJson(doc, configFile);

  configFile.close();

}



// Load MQTT configuration from LittleFS

bool loadMQTTConfig() {

  if (LittleFS.exists("/config.json")) {

    File configFile = LittleFS.open("/config.json", "r");

    if (configFile) {

      size_t size = configFile.size();

      std::unique_ptr<char[]> buf(new char[size]);

      configFile.readBytes(buf.get(), size);



      DynamicJsonDocument doc(1024);

      DeserializationError error = deserializeJson(doc, buf.get());



      if (!error) {

        strlcpy(mqttSettings.server, doc["mqtt_server"] | "", sizeof(mqttSettings.server));

        strlcpy(mqttSettings.port, doc["mqtt_port"] | "1883", sizeof(mqttSettings.port));

        strlcpy(mqttSettings.username, doc["mqtt_user"] | "", sizeof(mqttSettings.username));

        strlcpy(mqttSettings.password, doc["mqtt_pass"] | "", sizeof(mqttSettings.password));

        return true;

      } else {

        Serial.println("Failed to parse config file");

      }

    } else {

      Serial.println("Failed to open config file for reading");

    }

  } else {

    Serial.println("Config file does not exist");

  }

  return false;

}



void setupWebServer() {

  // Reset page

  webServer.on("/reset", []() {

    webServer.send(200, "text/html",

                   "<html><body>"

                   "<h1>Reset Settings</h1>"

                   "<p>This will erase all WiFi and MQTT settings and restart the device.</p>"

                   "<form method='post'><input type='submit' value='reset'></form>"

                   "</body></html>");

   WiFiManager wifiManager;

    wifiManager.resetSettings();

    Serial.println("Device Should reset");

  ESP.restart();

 

  });



 webServer.on("/reset", HTTP_POST, []() {  



    webServer.send(200, "text/html",

      "<html><body>"

      "<h1>Device Resetting</h1>"

      "<p>The device will now reset. Please reconnect to the AP to reconfigure.</p>"

      "</body></html>");

    delay(1000);  

   

  });



  // Status page

  webServer.on("/", []() {

    String status = "<html><body>"

                    "<h1>NRF24 Gateway Status</h1>"

                    "<p>WiFi: "

                    + WiFi.SSID() + " (" + WiFi.RSSI() + " dBm)</p>"

                                                         "<p>IP: "

                    + WiFi.localIP().toString() + "</p>"

                                                  "<p>MQTT Server: "

                    + String(mqttSettings.server) + ":" + String(mqttSettings.port) + "</p>"

                                                                                      "<p>MQTT Status: "

                    + String(client.connected() ? "Connected" : "Disconnected") + "</p>"

                                                                                  "<p><a href='/reset'>Reset Settings</a></p>"

                                                                                  "</body></html>";

    webServer.send(200, "text/html", status);

  });



  webServer.begin();

}



void setupWiFiManager() {

  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);



  // Custom parameters for MQTT

  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", mqttSettings.server, 40);

  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", mqttSettings.port, 6);

  WiFiManagerParameter custom_mqtt_user("user", "MQTT username", mqttSettings.username, 40);

  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", mqttSettings.password, 40);



  wifiManager.addParameter(&custom_mqtt_server);

  wifiManager.addParameter(&custom_mqtt_port);

  wifiManager.addParameter(&custom_mqtt_user);

  wifiManager.addParameter(&custom_mqtt_pass);



  // Set config portal timeout

  wifiManager.setConfigPortalTimeout(180);



  // Set AP name

  String apName = "NRF24_Gateway_" + String(ESP.getChipId(), HEX);



  if (!wifiManager.autoConnect(apName.c_str())) {

    Serial.println("Failed to connect and hit timeout");

    delay(3000);

    ESP.restart();

  }



  // Read updated parameters

  strlcpy(mqttSettings.server, custom_mqtt_server.getValue(), sizeof(mqttSettings.server));

  strlcpy(mqttSettings.port, custom_mqtt_port.getValue(), sizeof(mqttSettings.port));

  strlcpy(mqttSettings.username, custom_mqtt_user.getValue(), sizeof(mqttSettings.username));

  strlcpy(mqttSettings.password, custom_mqtt_pass.getValue(), sizeof(mqttSettings.password));



  // Save the custom parameters to FS

  if (shouldSaveConfig) {

    saveMQTTConfig();

  }

}



void setupOTA() {

  // Hostname defaults to esp8266-[ChipID]

  String hostname = "NRF24-Gateway-" + String(ESP.getChipId(), HEX);

  ArduinoOTA.setHostname(hostname.c_str());



  ArduinoOTA.onStart([]() {

    String type;

    if (ArduinoOTA.getCommand() == U_FLASH) {

      type = "sketch";

    } else {  // U_FS

      type = "filesystem";

    }

    Serial.println("Start updating " + type);

  });



  ArduinoOTA.onEnd([]() {

    Serial.println("\nEnd");

  });



  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {

    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

  });



  ArduinoOTA.onError([](ota_error_t error) {

    Serial.printf("Error[%u]: ", error);

    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");

    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");

    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");

    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");

    else if (error == OTA_END_ERROR) Serial.println("End Failed");

  });



  ArduinoOTA.begin();

}



void setupRadio() {

  if (!radio.begin()) {

    Serial.println("Radio hardware not responding!");

    // Consider adding a loop or restart here if radio is critical

    return;

  }



  radio.setPALevel(RF24_PA_MAX);             // Set power level

  radio.setChannel(101);                      // Set channel away from WiFi

  radio.setDataRate(RF24_250KBPS);           // Slower data rate for better range

  //radio.setPayloadSize(sizeof(SensorData));  // Ensure payload size matches the new struct



  // Open reading pipes for all possible sensors

  for (uint8_t i = 1; i < 6; i++) {

    radio.openReadingPipe(i, pipes[i]);

  }



  radio.startListening();

}



void publishDiscoveryConfig(uint8_t sensor_id) {

  Serial.println("Discovery Is Publishing");

  StaticJsonDocument<512> doc;

  char buffer[512];

  char discovery_topic[128];



  char device_name[32];

  snprintf(device_name, sizeof(device_name), "nrf24_sensor_%d", sensor_id);



  char state_topic[64];

  snprintf(state_topic, sizeof(state_topic), "sensors/nrf24/%d", sensor_id);



  // Publish discovery config for each measurement type

  for (const auto& config : SENSOR_CONFIGS) {

    doc.clear();



    JsonObject device = doc.createNestedObject("device");

    device["identifiers"][0] = device_name;

    device["name"] = device_name;

    device["manufacturer"] = "DIY";

    device["model"] = "NRF24L01 Sensor";



    char unique_id[64];

    char name[64];

    snprintf(unique_id, sizeof(unique_id), "nrf24_%d_%s", sensor_id, config.device_class);

    snprintf(name, sizeof(name), "NRF24 Sensor %d %s", sensor_id, config.device_class);



    doc["name"] = name;

    doc["unique_id"] = unique_id;

    doc["device_class"] = config.device_class;

    doc["state_topic"] = state_topic;

    doc["unit_of_measurement"] = config.unit;

    doc["value_template"] = config.value_template;



    snprintf(discovery_topic, sizeof(discovery_topic), "%s/%s/%s/config",

             discovery_prefix, config.component, unique_id);



    serializeJson(doc, buffer);

    client.publish(discovery_topic, buffer, true);

  }

}



void publishSensorData(const SensorData& data) {

  static bool discovery_published[6] = { false, false, false, false, false, false };



  if (data.sensorID > 0 && data.sensorID < 6) {  // Use data.sensorID

    if (!discovery_published[data.sensorID]) {

      publishDiscoveryConfig(data.sensorID);

      discovery_published[data.sensorID] = true;

    }

  } else {

    Serial.printf("Invalid sensorID received: %d\n", data.sensorID);

    return;

  }



  // Convert received uint16_t data to float with correct units

  float temperature_F = (data.temperature / 100.0);  // F to C

  float humidity_pct = data.humidity / 100.0;                           // 0.01% to %

  float battery_V = data.batteryVoltage / 1000.0;                       // mV to V



  StaticJsonDocument<200> doc;

  char mqtt_topic[50];

  char mqtt_payload[200];



  snprintf(mqtt_topic, sizeof(mqtt_topic), "sensors/nrf24/%d", data.sensorID);  // Use data.sensorID



  doc["temperature"] = temperature_F;

  doc["humidity"] = humidity_pct;      // New: Add humidity to payload

  doc["battery_voltage"] = battery_V;  // Changed key to match value_template

  doc["sensor_id"] = data.sensorID;    // Use data.sensorID



  serializeJson(doc, mqtt_payload);

  client.publish(mqtt_topic, mqtt_payload, true);



  digitalWrite(LED_PIN, LOW);  // LED on

  delay(50);

  digitalWrite(LED_PIN, HIGH);  // LED off

}



void reconnectMQTT() {

  if (!client.connected() && (millis() - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL)) {

    Serial.print("Attempting MQTT connection...");

    String clientId = "ESP8266Gateway-";

    clientId += String(random(0xffff), HEX);



    if (client.connect(clientId.c_str(), mqttSettings.username, mqttSettings.password)) {

      Serial.println("connected");

      digitalWrite(LED_PIN, HIGH);  // LED off when connected

      isConnected = true;

     

    } else {

      Serial.print("failed, rc=");

      Serial.print(client.state());

      Serial.println(" retrying...");

      digitalWrite(LED_PIN, LOW);  // LED on when disconnected

      isConnected = false;



    }

    lastMqttReconnectAttempt = millis();

  }

}



void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);  // LED on during setup



  if (!LittleFS.begin()) {

    Serial.println("Failed to mount LittleFS file system");

    return;

  } else {

    Serial.println("LittleFS mounted successfully");

  }



  loadMQTTConfig();

  setupWiFiManager();

  setupOTA();

  setupWebServer();



  client.setServer(mqttSettings.server, atoi(mqttSettings.port));

client.setBufferSize(1024);

  setupRadio();



  digitalWrite(LED_PIN, HIGH);  // LED off after setup complete

}



void loop() {

  ArduinoOTA.handle();

  webServer.handleClient();

  client.loop();



  reconnectMQTT();



  // Check for radio data

  if (radio.available() && isConnected) {

    SensorData receivedData;

    radio.read(&receivedData, sizeof(receivedData));



    // Perform basic validation on raw data before conversion

    if (receivedData.sensorID > 0 && receivedData.sensorID < 6 && receivedData.temperature >= 0 && receivedData.temperature <= 15000 &&  // Approx 0F to 150F

        receivedData.humidity >= 0 && receivedData.humidity <= 10000 &&                                                                  // Approx 0% to 100%

        receivedData.batteryVoltage >= 2000 && receivedData.batteryVoltage <= 6000) {                                                    // Approx 2.0V to 5.0V



      publishSensorData(receivedData);



      // Print converted values for easier debugging

      float temperature_F = (receivedData.temperature / 100.0);

      float humidity_pct = receivedData.humidity / 100.0;

      float battery_V = receivedData.batteryVoltage / 1000.0;



      Serial.print("Received from sensor ");

      Serial.print(receivedData.sensorID);

      Serial.print(": Temp=");

      Serial.print(temperature_F, 2);  // 2 decimal places

      Serial.print("°F, Humidity=");

      Serial.print(humidity_pct, 2);  // 2 decimal places

      Serial.print("%, Battery=");

      Serial.print(battery_V, 2);  // 2 decimal places

      Serial.println("V");

    } else {

      Serial.print("Received invalid raw data from sensor ");

      Serial.print(receivedData.sensorID);

      Serial.print(": Raw Temp=");

      Serial.print(receivedData.temperature);

      Serial.print(", Raw Hum=");

      Serial.print(receivedData.humidity);

      Serial.print(", Raw Bat=");

      Serial.print(receivedData.batteryVoltage);

      Serial.println(" - Data ignored.");

    }

  }

}
