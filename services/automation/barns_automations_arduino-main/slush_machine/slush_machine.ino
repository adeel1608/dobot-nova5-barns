#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "ibra";
const char* password = "ibrahim1";

// MQTT broker config
const char* mqttServer = "192.168.100.239";

const int mqttPort = 1883;
const char* mqttUser = "admin";
const char* mqttPassword = "admin123";
String slush_dispensed = "Failed";
float timer;


WiFiClient espClient;
PubSubClient client(espClient);

// Define the pin for the Single button action
#define IND 2
#define SIG1 32
#define SIG2 25
#define SIG3 33
#define SIG4 26


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set up the pin for the single button control
  pinMode(SIG1, OUTPUT);
  pinMode(SIG2, OUTPUT);
  pinMode(SIG3, OUTPUT);
  pinMode(SIG4, OUTPUT);
  pinMode(IND, OUTPUT);
  
  digitalWrite(SIG1, LOW);  // Default state is HIGH (not pressed)
  digitalWrite(SIG2, LOW);
  digitalWrite(SIG3, LOW);
  digitalWrite(SIG4, LOW);
  digitalWrite(IND, LOW);

  // Connect to Wi‑Fi
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to Wi‑Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi‑Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT server
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Motor", mqttUser, mqttPassword)) {
      Serial.println(" connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }

  client.subscribe("automation_slush", 1);
  Serial.println("Subscribed to topic: automation");
}

void onecup() {
  // Simulate a button press on SINGLE_PIN
  //Serial.println("Performing single button press...");
  digitalWrite(SIG1, HIGH);  // Simulate pressing (active LOW)
  digitalWrite(IND, HIGH);
  delay(1000);                     // Press duration: 800 ms
  digitalWrite(SIG1, LOW); // Release the button
  digitalWrite(IND, LOW);
  delay(500);                     // Post-press delay: 200 ms
  //Serial.println("Single button press completed.");
}


void twocup() {
  // Simulate a button press on SINGLE_PIN
  //Serial.println("Performing single button press...");
  digitalWrite(SIG2, HIGH);  // Simulate pressing (active LOW)
  digitalWrite(IND, HIGH);
  delay(5000);                     // Press duration: 800 ms
  digitalWrite(SIG2, LOW); // Release the button
  digitalWrite(IND, LOW);
  delay(500);                     // Post-press delay: 200 ms
  //Serial.println("Single button press completed.");
}


void loop() {
  client.loop();  // handle MQTT messages
}


void callback(char* topic, byte* payload, unsigned int length){
  
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.println("]");

  char jsonBuffer[256];
  if (length >= sizeof(jsonBuffer)) length = sizeof(jsonBuffer) - 1;
  memcpy(jsonBuffer, payload, length);
  jsonBuffer[length] = '\0';

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonBuffer);

  if (error) {
    Serial.println("Failed to parse JSON");
    return;
  }

  String Slush_type= doc["Slush_type"].as<String>();
  String cup_size = doc["cup"].as<String>();
  

  Serial.printf("Slush Type: %s", Slush_type.c_str());


      // If the message is "Single", execute the single button press, wait 5 seconds, then restart
    if (Slush_type == "Slush_1") {
      delay(500);
      onecup();
      delay(500);  // Wait
      slush_dispensed = "success";
      sendResponse(slush_dispensed);
    }

    // If the message is "Single", execute the single button press, wait 5 seconds, then restart
    if (Slush_type == "Slush_2") {
      delay(500);
      twocup();
      delay(500);  // Wait
      slush_dispensed = "success";
      sendResponse(slush_dispensed);
    }
    else {
      delay(500);
      digitalWrite(IND, HIGH);
      delay(500);
      digitalWrite(IND, LOW);
      slush_dispensed = "Failed";
      sendResponse(slush_dispensed);
    }

}

void sendResponse(String slush_dispensed) {
  StaticJsonDocument<128> response;
  if (slush_dispensed == "success"){
  response["Machine"] = "Slush Machine";
  response["status"] = "success";
  response["return"] = "Slush filled successfully";
  }
  else {
    response["status"] = "Failed";
    response["error"] = "Invalid input";
  }

  char buffer[128];
  serializeJson(response, buffer);
  client.publish("automation/response", buffer);
  Serial.println("Response sent:");
  Serial.println(buffer);
  delay(3000);
  ESP.restart();
}
