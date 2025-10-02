// Coffee Grinder Dispenser – with robust Wi‑Fi & MQTT reconnection
// ---------------------------------------------------------------
// Keeps trying to reconnect any time Wi‑Fi or the MQTT broker drops.
// ---------------------------------------------------------------

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ───────── Wi‑Fi credentials ─────────
const char* ssid       = "muhammed-network";
const char* password   = "QSS2030QSS";

// ───────── MQTT broker config ────────
const char* mqttServer   = "192.168.200.254";
//const char* mqttServer   = "192.168.200.104";
const int   mqttPort     = 1883;
const char* mqttUser     = "admin";
const char* mqttPassword = "admin123";

// Globals for reconnect timing
unsigned long lastReconnectTry = 0;
const unsigned long RECONNECT_INTERVAL = 2000;  // try every 2 s

String dispense_status = "Failed";

// ───────── GPIO mapping ─────────────
#define SINGLE_PIN1 33
#define DOUBLE_PIN2 25
#define TRIPLE_PIN3 32
#define IND         2

// ───────── Globals ──────────────────
WiFiClient   espClient;
PubSubClient client(espClient);

// ───────── Forward declarations ─────
bool reconnectMqtt();
void connectWiFi();

// ───────── Shot helpers ─────────────
inline void singleshot() {
  digitalWrite(SINGLE_PIN1, LOW);
  digitalWrite(DOUBLE_PIN2, HIGH);
  digitalWrite(TRIPLE_PIN3, HIGH);
  delay(500);
  digitalWrite(SINGLE_PIN1, HIGH);
  delayMicroseconds(500);
}

inline void doubleshot() {
  digitalWrite(DOUBLE_PIN2, LOW);
  digitalWrite(SINGLE_PIN1, HIGH);
  digitalWrite(TRIPLE_PIN3, HIGH);
  delay(500);
  digitalWrite(DOUBLE_PIN2, HIGH);
  delayMicroseconds(500);
}

inline void tripleshot() {
  digitalWrite(TRIPLE_PIN3, LOW);
  digitalWrite(SINGLE_PIN1, HIGH);
  digitalWrite(DOUBLE_PIN2, HIGH);
  delay(500);
  digitalWrite(TRIPLE_PIN3, HIGH);
  delay(500);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting…");
    if (client.connect("Grinder", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("automation_grinding");
      Serial.println("connected to topic grin");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// ───────── Response publishing ──────
void sendResponse() {
  StaticJsonDocument<128> response;
  response["status"] = dispense_status;
  if (dispense_status != "success") response["error"] = "Invalid input";

  char buffer[128];
  serializeJson(response, buffer);
  client.publish("automation/response", buffer);
  Serial.printf("Response sent: %s\n", buffer);
}

// ───────── MQTT callback ────────────
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message received [%s]\n", topic);

  char jsonBuffer[256];
  if (length >= sizeof(jsonBuffer)) length = sizeof(jsonBuffer) - 1;
  memcpy(jsonBuffer, payload, length);
  jsonBuffer[length] = '\0';

  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, jsonBuffer)) {
    Serial.println("Failed to parse JSON");
    return;
  }

  int shots = doc["shots_number"].as<int>();
  Serial.printf("Shots: %d\n", shots);

  digitalWrite(IND, HIGH);
  delay(500);

  if      (shots == 1) { singleshot();  dispense_status = "success"; }
  else if (shots == 2) { doubleshot();  dispense_status = "success"; }
  else if (shots == 3) { tripleshot();  dispense_status = "success"; }
  else                 { Serial.println("Invalid shots_number"); dispense_status = "Failed"; }

  digitalWrite(IND, LOW);
  sendResponse();
}

// ───────── Wi‑Fi connect/maintain ───

// ───────── MQTT connect/maintain ────

// ───────── Arduino setup ────────────
void setup() {
  Serial.begin(115200);

  pinMode(SINGLE_PIN1, OUTPUT);
  pinMode(DOUBLE_PIN2, OUTPUT);
  pinMode(TRIPLE_PIN3, OUTPUT);
  digitalWrite(SINGLE_PIN1, HIGH);
  digitalWrite(DOUBLE_PIN2, HIGH);
  digitalWrite(TRIPLE_PIN3, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // (we don’t block here—loop() will heal if Wi-Fi isn’t up immediately)

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);


// ───────── Arduino loop ─────────────
  // Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  }

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
