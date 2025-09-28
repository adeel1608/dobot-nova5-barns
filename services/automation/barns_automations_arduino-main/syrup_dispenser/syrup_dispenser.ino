#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ——— Wi-Fi credentials ———
const char* ssid       = "muhammed-network";
const char* password   = "QSS2030QSS";
//const char* ssid       = "S23";
//const char* password   = "12345678";

// ——— MQTT broker config ———
const char* mqttServer   = "192.168.200.254";
//const char* mqttServer   = "192.168.200.104";

const int   mqttPort     = 1883;
const char* mqttUser     = "admin";
const char* mqttPassword = "admin123";

// ——— Pin definitions ———
#define DIR_PIN1 4
#define STEP_PIN1 16
#define SIG1      17

#define DIR_PIN2 18
#define STEP_PIN2 19
#define SIG2      21

#define DIR_PIN3 25
#define STEP_PIN3 26
#define SIG3      27

#define DIR_PIN4 33
#define STEP_PIN4 32
#define SIG4      15

#define IND       2  // Indicator LED

WiFiClient     espClient;
PubSubClient   client(espClient);
String         dispense_status = "Failed";

// ——— Stepper “microstep” functions ———
void motor1() {
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(12);
}
void motor2() {
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(12);
}
void motor3() {
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(12);
}
void motor4() {
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(12);
}

void cleanpipe(){
  //motor1
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(12);

  //motor2
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(12);

  //motor3
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(12);

  //motor4
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(12);
}


void emptypipe(){


//set directions inverse
  digitalWrite(DIR_PIN1, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG1, LOW);
  digitalWrite(DIR_PIN2, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG2, LOW);
  digitalWrite(DIR_PIN3, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG3, LOW);
  digitalWrite(DIR_PIN4, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG4, LOW);
  
    //motor1
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN1, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN1, LOW);   delayMicroseconds(12);

  //motor2
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN2, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN2, LOW);   delayMicroseconds(12);

  //motor3
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN3, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN3, LOW);   delayMicroseconds(12);

  //motor4
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(25);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(25);
  digitalWrite(STEP_PIN4, HIGH);  delayMicroseconds(12);
  digitalWrite(STEP_PIN4, LOW);   delayMicroseconds(12);
}

// ——— Helper: run a given motor function for ‘duration’ ms ———
void runMotor(void (*motorFunc)(), unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    motorFunc();
  }
}

// ——— Publish a JSON response to “automation/response” ———
void sendResponse() {
  StaticJsonDocument<128> response;
  if (dispense_status == "success") {
    response["status"] = "success";
  } else {
    response["status"] = "Failed";
    response["error"]  = "Invalid or missing input";
  }
  char buffer[128];
  serializeJson(response, buffer);
  client.publish("automation/response", buffer);
  Serial.println("Response sent:");
  Serial.println(buffer);
}

// ——— MQTT message callback ———
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  auto err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.println("JSON parse error");
    return;
  }

  String milkType = doc["syrup_type"].as<String>();
  float  amount   = doc["amount"];
  unsigned long duration = (unsigned long)((amount / 6.75f) * 1000.0f);

  Serial.printf("Got %s: %.2f → %lums\n",
                milkType.c_str(), amount, duration);

  digitalWrite(IND, HIGH);

  // dispense based on milk type
  if (milkType == "syrup_1") {
    digitalWrite(SIG1, HIGH);
    delay(500);
    runMotor(motor1, duration);
    digitalWrite(SIG1, LOW);
    dispense_status = "success";

  } else if (milkType == "syrup_2") {
    digitalWrite(SIG2, HIGH);
    delay(500);
    runMotor(motor2, duration);
    digitalWrite(SIG2, LOW);
    dispense_status = "success";

  } else if (milkType == "syrup_3") {
    digitalWrite(SIG3, HIGH);
    delay(500);
    runMotor(motor3, duration);
    digitalWrite(SIG3, LOW);
    dispense_status = "success";

  } else if (milkType == "clean"){
    digitalWrite(SIG1, HIGH);
    delay(500);
    runMotor(cleanpipe, 30000);
    delay(500);
    runMotor(emptypipe, 30000);
    digitalWrite(SIG1, LOW);
    } 
  
  else {
    dispense_status = "Failed";
  }

  digitalWrite(IND, LOW);
  sendResponse();
  delay(1000);
}

// ——— (Re)connect to MQTT and subscribe ———
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting…");
    if (client.connect("syrup_dispenser", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("automation_syrup");
      Serial.println("connected to topic automation_syrup");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // pins
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(SIG1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(SIG2, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(SIG3, OUTPUT);
  pinMode(DIR_PIN4, OUTPUT);
  pinMode(STEP_PIN4, OUTPUT);
  pinMode(SIG4, OUTPUT);
  pinMode(IND, OUTPUT);

  // Set the initial direction
  digitalWrite(DIR_PIN1, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG1, LOW);
  digitalWrite(DIR_PIN2, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG2, LOW);
  digitalWrite(DIR_PIN3, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG3, LOW);
  digitalWrite(DIR_PIN4, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(SIG4, LOW);

  digitalWrite(IND, LOW);

  // ensure signals are low initially
  digitalWrite(SIG1, LOW);
  digitalWrite(SIG2, LOW);
  digitalWrite(SIG3, LOW);
  digitalWrite(SIG4, LOW);
  digitalWrite(IND, LOW);

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
  client.setKeepAlive(120);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
