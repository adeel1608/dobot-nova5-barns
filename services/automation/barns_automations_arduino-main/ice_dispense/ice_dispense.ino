#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>



/* 
#include <esp_task_wdt.h>

int WATCHDO_TIMEOUT_S = 
in void loop {
esp_task_wdt_reset
}
esp_task_wdt_init(WATCHDO_TIMEOUT_S, true);
*/



// Wi-Fi credentials
const char* ssid       = "muhammed-network";
const char* password   = "QSS2030QSS";

// MQTT broker config
const char* mqttServer   = "192.168.200.254";
//const char* mqttServer   = "10.73.233.150";
const int   mqttPort     = 1883;
const char* mqttUser     = "admin";
const char* mqttPassword = "admin123";

// Motor pins
const int IN1 = 22;
const int IN2 = 21;
const int IND = 2;
float timer = 0;
//digitalWrite(IN1, LOW);

String tamp_status = "Failed";

WiFiClient espClient;
PubSubClient client(espClient);

// Tamping routine
void tampering() {
  // Extend
//  digitalWrite(IND, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(25000);


  // Stop
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);
//  digitalWrite(IND, HIGH);
  delay(1000);
}



void setup() {
  // Start Serial
  Serial.begin(115200);

  // Setup motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);


  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
//  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
//    Serial.print(".");

  }
//  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setKeepAlive(120);

  while (!client.connected()) {
//    Serial.print("Connecting to MQTT...");
    if (client.connect("Tampering Machine", mqttUser, mqttPassword)) {
      Serial.println(" connected");
    } else {
//      Serial.print(" failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }

  // Subscribe only to the tamping topic
  client.subscribe("automation_ice", 1);
  Serial.println("Subscribed to topic: automation_tampering");
}

// Send JSON response over MQTT
void sendResponse() {
  StaticJsonDocument<128> response;
  if (tamp_status == "success") {
    response["status"] = "success";
  } else {
    response["status"] = "Failed";
    response["error"] = "Invalid or missing input";
  }

  char buffer[128];
  serializeJson(response, buffer);
  client.publish("automation/response", buffer);
  Serial.println("Response sent:");
//  Serial.println(buffer);
  delay(1000);
  ESP.restart();
}

// MQTT message handler
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.println("]");

  char jsonBuffer[256];
  if (length >= sizeof(jsonBuffer)) length = sizeof(jsonBuffer) - 1;
  memcpy(jsonBuffer, payload, length);
  jsonBuffer[length] = '\0';

  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, jsonBuffer)) {
    Serial.println("Failed to parse JSON");
    tamp_status = "Failed";
    sendResponse();
    return;
  }

  int command = doc["ice"].as<int>();
//  float timer = doc["timer"].as<float>();
//  Serial.print("ice");
//  Serial.println(command);

  if (command == 1) {
//    tampering(timer);
    tampering();
    tamp_status = "success";
  } else {
    tamp_status = "Failed";
    Serial.println("Invalid tamping command");
  }
  sendResponse();
}


void reconnectMqtt() {
  // Ensure Wi-Fi is still up
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost – reconnecting…");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println("\nRe-connected to Wi-Fi");
  }

  // Reconnect to MQTT
  while (!client.connected()) {
    Serial.print("Connecting to MQTT … ");
    if (client.connect("Tampering Machine", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("automation_tampering", 1);
      client.setKeepAlive(120);
    } else {
//      Serial.printf("failed, rc=%d – retrying in 2 s\n", client.state());/ // IF NOT CONNECTING REMOVE THIS TO DEBUG
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnectMqtt();    // <-- reconnect if needed
  }
  client.loop();
}
