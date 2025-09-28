#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01};
IPAddress ip(192, 168, 200, 211);
IPAddress gw(192, 168, 200, 1);
IPAddress sub(255, 255, 255, 0);

EthernetClient ethClient;
PubSubClient mqtt(ethClient);

char buf[64];
int idx = 0;

void callback(char* topic, byte* payload, unsigned int len) {
  payload[len] = 0;
  Serial.print("MQTT RX: ");
  Serial.println((char*)payload);
  
  // Parse JSON and extract command
  // Expected format: {"ingredient":"caramel","weight":10,"motor":"sauce1","command":"caramel_10"}
  String payloadStr = String((char*)payload);
  int commandStart = payloadStr.indexOf("\"command\":\"") + 11;
  int commandEnd = payloadStr.indexOf("\"", commandStart);
  
  if (commandStart > 10 && commandEnd > commandStart) {
    String command = payloadStr.substring(commandStart, commandEnd);
    Serial.print("Extracted command: ");
    Serial.println(command);
    
    // Forward command to Mega
    Serial1.println(command);
    Serial.print("Sent to Mega: ");
    Serial.println(command);
  } else {
    // Fallback: forward entire payload
    Serial1.println((char*)payload);
    Serial.print("Sent to Mega: ");
    Serial.println((char*)payload);
  }
}

void setup() {
  // Initialize serial without waiting for USB connection (for standalone operation)
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(2000);
  
  Serial.println("=== MQTT Bridge Starting ===");
  
  // Setup W5500
  pinMode(10, OUTPUT); // CS
  pinMode(9, OUTPUT);  // RST
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(9, HIGH);
  delay(200);
  
  // Initialize Ethernet
  Serial.println("Initializing W5500...");
  Ethernet.begin(mac, ip, gw, gw, sub);
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
  
  // Setup MQTT - Connect to RabbitMQ broker
  mqtt.setServer("192.168.200.233", 1883);  // Replace with your RabbitMQ IP
  mqtt.setCallback(callback);
  
  Serial.println("=== MQTT Bridge Ready! ===");
}

void loop() {
  // MQTT connection
  if (!mqtt.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqtt.connect("bridge01", "admin", "admin123")) {  // RabbitMQ credentials
      mqtt.subscribe("automation_dispensing");  // Subscribe to automation topic
      Serial.println("MQTT Connected to RabbitMQ!");
    } else {
      Serial.print("MQTT Failed: ");
      Serial.println(mqtt.state());
    }
  }
  mqtt.loop();
  
  // UART data processing
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[idx] = 0;
      if (strstr(buf, "Scales A=")) {
        float a, b;
        sscanf(buf, "Scales A=%fg B=%fg", &a, &b);
        sprintf(buf, "{\"A\":%.1f,\"B\":%.1f}", a, b);
        mqtt.publish("dispenser/weights", buf);
      }
      idx = 0;
    } else if (idx < 63) {
      buf[idx++] = c;
    }
  }
  
  delay(10);
} 