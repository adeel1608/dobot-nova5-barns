#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <mcp2515.h>

// Network configuration
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01};
IPAddress ip(192, 168, 200, 211);
IPAddress gw(192, 168, 200, 1);
IPAddress sub(255, 255, 255, 0);

// W5500 pins (existing)
#define W5500_CS_PIN   10
#define W5500_RST_PIN  9

// MCP2515 pins (new)
#define CAN_CS_PIN     4
#define CAN_INT_PIN    2
#define CAN_SCK_PIN    3
#define CAN_MOSI_PIN   5
#define CAN_MISO_PIN   6

// CAN configuration
#define CAN_SPEED      CAN_500KBPS
#define MCP_CLOCK      MCP_8MHZ

// CAN IDs for dispensing system
#define CAN_ID_DISPENSING_CMD   0x110   // Commands to dispensing system
#define CAN_ID_DISPENSING_ACK   0x111   // ACK from dispensing system
#define CAN_ID_DISPENSING_DATA  0x112   // Scale data from dispensing system

// Liquid type mapping
#define LIQUID_WATER    1
#define LIQUID_MILK     2
#define LIQUID_SAUCE    3
#define LIQUID_CARAMEL  4
#define LIQUID_SYRUP    5
#define LIQUID_HONEY    6

EthernetClient ethClient;
PubSubClient mqtt(ethClient);
MCP2515 mcp2515(CAN_CS_PIN);

static uint8_t can_error_count = 0;

static void can_reinit_normal() {
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_CLOCK);
  mcp2515.setNormalMode();
  can_error_count = 0;
  Serial.println("CAN reinitialized to Normal mode");
}

static void can_set_loopback(bool enable) {
  if (enable) {
    mcp2515.setLoopbackMode();
    Serial.println("CAN set to Loopback mode");
  } else {
    mcp2515.setNormalMode();
    Serial.println("CAN set to Normal mode");
  }
}

char buf[128];
int idx = 0;

// CAN message structure for dispensing
struct CanDispenseCmd {
  uint8_t cmd;           // 0x01=dispense, 0x02=stop, 0x03=set_lag
  uint8_t motor_id;      // Motor number (1-24)
  uint16_t weight_dg;    // Target weight in decigrams (or lag in dg for SET_LAG)
  uint8_t liquid_type;   // Liquid type for lag compensation (unused for SET_LAG)
  uint8_t reserved[3];   // Reserved bytes (for SET_LAG: reserved[0] used as speed 0|1)
};

static const uint8_t CMD_DISPENSE = 0x01;
static const uint8_t CMD_STOP     = 0x02;
static const uint8_t CMD_SET_LAG  = 0x03;

uint8_t getLiquidTypeId(const char* liquid_name) {
  if (strstr(liquid_name, "water")) return LIQUID_WATER;
  if (strstr(liquid_name, "milk")) return LIQUID_MILK;
  if (strstr(liquid_name, "caramel")) return LIQUID_CARAMEL;
  if (strstr(liquid_name, "syrup")) return LIQUID_SYRUP;
  if (strstr(liquid_name, "honey")) return LIQUID_HONEY;
  return LIQUID_SAUCE; // Default
}

uint8_t getMotorIdFromName(const char* motor_name) {
  // milk1-8 = IDs 1-8, sauce1-15 = IDs 9-23, rinser = ID 24
  if (strstr(motor_name, "milk")) {
    int num = atoi(motor_name + 4);  // Extract number from "milk1"
    return (num >= 1 && num <= 8) ? num : 1;
  }
  if (strstr(motor_name, "sauce")) {
    int num = atoi(motor_name + 5);  // Extract number from "sauce1"
    return (num >= 1 && num <= 15) ? (8 + num) : 9;
  }
  if (strstr(motor_name, "rinser")) return 24;
  return 9; // Default to sauce1
}

static const char* getMotorNameById(uint8_t motor_id) {
  static char buf[12];
  if (motor_id >= 1 && motor_id <= 8) {
    sprintf(buf, "milk%u", (unsigned)motor_id);
    return buf;
  }
  if (motor_id >= 9 && motor_id <= 23) {
    sprintf(buf, "sauce%u", (unsigned)(motor_id - 8));
    return buf;
  }
  if (motor_id == 24) return "rinser";
  return nullptr;
}

void sendCANDispenseCommand(uint8_t motor_id, float weight, uint8_t liquid_type) {
  struct can_frame canMsg;
  canMsg.can_id = CAN_ID_DISPENSING_CMD;
  canMsg.can_dlc = 8;
  
  CanDispenseCmd cmd;
  cmd.cmd = CMD_DISPENSE;  // Dispense command
  cmd.motor_id = motor_id;
  cmd.weight_dg = (uint16_t)(weight * 10);  // Convert to decigrams
  cmd.liquid_type = liquid_type;
  cmd.reserved[0] = 0;
  cmd.reserved[1] = 0;
  cmd.reserved[2] = 0;
  
  memcpy(canMsg.data, &cmd, sizeof(cmd));
  
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&canMsg);
    if (txres != MCP2515::ERROR_OK) {
      Serial.print("CAN TX error: "); Serial.println((int)txres);
      if (++can_error_count >= 3) { can_reinit_normal(); }
    }
  }
  
  Serial.print("CAN TX: Motor=");
  Serial.print(motor_id);
  Serial.print(" Weight=");
  Serial.print(weight, 1);
  Serial.print("g Type=");
  Serial.println(liquid_type);
}

static void sendCANSetLag(uint8_t motor_id, float lag_g, uint8_t speed01) {
  struct can_frame canMsg;
  canMsg.can_id = CAN_ID_DISPENSING_CMD;
  canMsg.can_dlc = 8;

  CanDispenseCmd cmd;
  cmd.cmd = CMD_SET_LAG;   // Set per-motor lag override
  cmd.motor_id = motor_id;
  cmd.weight_dg = (uint16_t)(lag_g * 10); // reuse as lag in decigrams
  cmd.liquid_type = 0; // unused
  cmd.reserved[0] = speed01 ? 1 : 0; // speed profile (0 or 1)
  cmd.reserved[1] = 0;
  cmd.reserved[2] = 0;

  memcpy(canMsg.data, &cmd, sizeof(cmd));

  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&canMsg);
    if (txres != MCP2515::ERROR_OK) {
      Serial.print("CAN TX SET_LAG error: "); Serial.println((int)txres);
      if (++can_error_count >= 3) { can_reinit_normal(); }
    } else {
      Serial.print("CAN TX SET_LAG: motor="); Serial.print(motor_id);
      Serial.print(" lag="); Serial.print(lag_g, 1);
      Serial.print("g speed="); Serial.println(speed01 ? 1 : 0);
    }
  }
}

void sendCANStopCommand() {
  struct can_frame canMsg;
  canMsg.can_id = CAN_ID_DISPENSING_CMD;
  canMsg.can_dlc = 8;
  
  CanDispenseCmd cmd;
  cmd.cmd = CMD_STOP;  // Stop command
  cmd.motor_id = 0;
  cmd.weight_dg = 0;
  cmd.liquid_type = 0;
  
  memcpy(canMsg.data, &cmd, sizeof(cmd));
  mcp2515.sendMessage(&canMsg);
  
  Serial.println("CAN TX: STOP command");
}

void processCAN() {
  struct can_frame canMsg;
  
  while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Handle incoming command frames (Linux SocketCAN or other nodes)
    if (canMsg.can_id == CAN_ID_DISPENSING_CMD && canMsg.can_dlc >= 8) {
      CanDispenseCmd cmd;
      memcpy(&cmd, canMsg.data, sizeof(cmd));

      if (cmd.cmd == CMD_DISPENSE) {
        // Forward to Mega via UART as bridge command
        // Format expected by Mega: "CAN:motor_id,weight_dg,liquid_type"
        uint16_t weight_dg = cmd.weight_dg; // already in decigrams
        Serial1.print("CAN:");
        Serial1.print((int)cmd.motor_id);
        Serial1.print(",");
        Serial1.print((int)weight_dg);
        Serial1.print(",");
        Serial1.println((int)cmd.liquid_type);

        Serial.print("CAN RX CMD -> UART: motor="); Serial.print(cmd.motor_id);
        Serial.print(" weight_dg="); Serial.print(weight_dg);
        Serial.print(" type="); Serial.println(cmd.liquid_type);
      }
      else if (cmd.cmd == CMD_SET_LAG) {
        // Apply per-motor lag override on Mega via UART
        const char* motor_name = getMotorNameById(cmd.motor_id);
        float lag_g = ((float)cmd.weight_dg) / 10.0f;
        uint8_t speed01 = cmd.reserved[0] ? 1 : 0;
        if (motor_name) {
          Serial.print("CAN RX SET_LAG -> UART: ");
          Serial.print(motor_name); Serial.print(" speed "); Serial.print(speed01);
          Serial.print(" lag "); Serial.println(lag_g, 1);
          Serial1.print("LAGM "); Serial1.print(motor_name); Serial1.print(" "); Serial1.print((int)speed01); Serial1.print(" "); Serial1.println(lag_g, 1);
        } else {
          Serial.println("CAN RX SET_LAG: invalid motor_id");
        }
      }

      // Send ACK on CAN to confirm receipt
      struct can_frame ackMsg;
      ackMsg.can_id  = CAN_ID_DISPENSING_ACK;
      ackMsg.can_dlc = 8;
      ackMsg.data[0] = 0x01; // ACK indicator
      ackMsg.data[1] = 0x00;
      ackMsg.data[2] = 0x00;
      ackMsg.data[3] = 0x00;
      ackMsg.data[4] = 0x00;
      ackMsg.data[5] = 0x00;
      ackMsg.data[6] = 0x00;
      ackMsg.data[7] = 0x00;
      {
        MCP2515::ERROR txres = mcp2515.sendMessage(&ackMsg);
        if (txres != MCP2515::ERROR_OK) {
          Serial.print("CAN TX ACK error: "); Serial.println((int)txres);
          if (++can_error_count >= 3) { can_reinit_normal(); }
        } else {
          Serial.println("CAN TX ACK: 0x111 [01 00 00 00 00 00 00 00]");
        }
      }
      continue;
    }
    else if (canMsg.can_id == CAN_ID_DISPENSING_ACK) {
      Serial.print("CAN RX ACK: ");
      Serial.println(canMsg.data[0] == 0x01 ? "SUCCESS" : "ERROR");
    }
    else if (canMsg.can_id == CAN_ID_DISPENSING_DATA) {
      // Scale data from Mega
      int16_t scale_a = (canMsg.data[1] << 8) | canMsg.data[0];
      int16_t scale_b = (canMsg.data[3] << 8) | canMsg.data[2];
      uint8_t status = canMsg.data[4];
      uint8_t job_state = canMsg.data[5];
      uint8_t leak_status = canMsg.data[6];
      
      Serial.print("CAN RX Data: A=");
      Serial.print(scale_a / 10.0, 1);
      Serial.print("g B=");
      Serial.print(scale_b / 10.0, 1);
      Serial.print("g State=");
      Serial.print(job_state);
      if (leak_status) Serial.print(" LEAK!");
      Serial.println();
    }
    else {
      // Log any other frames for diagnostics
      Serial.print("CAN RX id=0x"); Serial.print(canMsg.can_id, HEX);
      Serial.print(" dlc="); Serial.print(canMsg.can_dlc);
      Serial.print(" data=");
      for (uint8_t i=0;i<canMsg.can_dlc;i++){ if (canMsg.data[i]<16) Serial.print('0'); Serial.print(canMsg.data[i], HEX); Serial.print(' ');} 
      Serial.println();
    }
  }
}

void callback(char* topic, byte* payload, unsigned int len) {
  payload[len] = 0;
  Serial.print("MQTT RX: ");
  Serial.println((char*)payload);
  
  // Parse JSON and handle different command types
  String payloadStr = String((char*)payload);
  String topicStr = String(topic);

  // Per-motor lag tuning over MQTT
  if (topicStr == "automation_dispensing_lag") {
    // Expect JSON: {"motor":"sauce9","speed":1,"lag":12.5}
    int mStart = payloadStr.indexOf("\"motor\":\"") + 9;
    int mEnd = payloadStr.indexOf("\"", mStart);
    int sStart = payloadStr.indexOf("\"speed\":") + 8;
    int sEnd = payloadStr.indexOf(",", sStart);
    if (sEnd == -1) sEnd = payloadStr.indexOf("}", sStart);
    int lStart = payloadStr.indexOf("\"lag\":") + 7;
    int lEnd = payloadStr.indexOf(",", lStart);
    if (lEnd == -1) lEnd = payloadStr.indexOf("}", lStart);

    if (mStart > 8 && mEnd > mStart && sStart > 7 && sEnd > sStart && lStart > 6 && lEnd > lStart) {
      String motor = payloadStr.substring(mStart, mEnd);
      int speed = payloadStr.substring(sStart, sEnd).toInt();
      float lag = payloadStr.substring(lStart, lEnd).toFloat();
      motor.toLowerCase();
      Serial.print("Apply LAGM via MQTT: "); Serial.print(motor); Serial.print(" "); Serial.print(speed); Serial.print(" "); Serial.println(lag, 1);
      Serial1.print("LAGM "); Serial1.print(motor); Serial1.print(" "); Serial1.print(speed); Serial1.print(" "); Serial1.println(lag, 1);

      // Also broadcast over CAN so other nodes can observe and to allow CAN-only configs
      uint8_t motor_id = getMotorIdFromName(motor.c_str());
      sendCANSetLag(motor_id, lag, (uint8_t)(speed ? 1 : 0));
    } else {
      Serial.println("Invalid payload for automation_dispensing_lag. Expected {\"motor\":\"sauce9\",\"speed\":0|1,\"lag\":<g>} ");
    }
    return;
  }

  // Special: raw CAN control channel for simple tests (no UART forwarding)
  if (topicStr == "automation_dispensing_can") {
    // Heartbeat trigger: send 0x10F with FF FF 00 00 00 00 00 00
    if (payloadStr == "heartbeat" || payloadStr.indexOf("\"heartbeat\"") >= 0) {
      struct can_frame hb;
      hb.can_id  = 0x10F;
      hb.can_dlc = 8;
      hb.data[0] = 0xFF; hb.data[1] = 0xFF; hb.data[2] = 0x00; hb.data[3] = 0x00;
      hb.data[4] = 0x00; hb.data[5] = 0x00; hb.data[6] = 0x00; hb.data[7] = 0x00;
      {
        MCP2515::ERROR txres = mcp2515.sendMessage(&hb);
        if (txres != MCP2515::ERROR_OK) {
          Serial.print("CAN TX HB error: "); Serial.println((int)txres);
          if (++can_error_count >= 3) { can_reinit_normal(); }
        } else {
          Serial.println("CAN TX HB: 0x10F [FF FF 00 00 00 00 00 00]");
        }
      }
      return;
    }
    if (payloadStr == "loopback_on") { can_set_loopback(true); return; }
    if (payloadStr == "loopback_off") { can_set_loopback(false); return; }
  }
  
  // Handle dispensing commands via CAN (also forwarded to Mega)
  if (strstr(topic, "automation_dispensing")) {
    // Parse JSON: {"ingredient":"caramel","weight":10,"motor":"sauce1","command":"caramel_10"}
    int ingredientStart = payloadStr.indexOf("\"ingredient\":\"") + 14;
    int ingredientEnd = payloadStr.indexOf("\"", ingredientStart);
    String ingredient = payloadStr.substring(ingredientStart, ingredientEnd);
    
    int weightStart = payloadStr.indexOf("\"weight\":") + 9;
    int weightEnd = payloadStr.indexOf(",", weightStart);
    if (weightEnd == -1) weightEnd = payloadStr.indexOf("}", weightStart);
    float weight = payloadStr.substring(weightStart, weightEnd).toFloat();
    
    int motorStart = payloadStr.indexOf("\"motor\":\"") + 9;
    int motorEnd = payloadStr.indexOf("\"", motorStart);
    String motor = payloadStr.substring(motorStart, motorEnd);
    
    // Convert to CAN format and send
    uint8_t motor_id = getMotorIdFromName(motor.c_str());
    uint8_t liquid_type = getLiquidTypeId(ingredient.c_str());
    
    sendCANDispenseCommand(motor_id, weight, liquid_type);
    
    Serial.print("Converted to CAN: ");
    Serial.print(ingredient);
    Serial.print("_");
    Serial.print(weight, 1);
    Serial.print(" -> Motor=");
    Serial.print(motor_id);
    Serial.print(" Type=");
    Serial.println(liquid_type);

    // ALSO forward to Mega over UART so it works without a CAN responder
    // Format expected by Mega: "CAN:motor_id,weight_dg,liquid_type"
    uint16_t weight_dg = (uint16_t)(weight * 10.0f);
    Serial1.print("CAN:");
    Serial1.print((int)motor_id);
    Serial1.print(",");
    Serial1.print((int)weight_dg);
    Serial1.print(",");
    Serial1.println((int)liquid_type);
  }
  // Handle other automation commands via UART (existing)
  else {
    // Extract command for Mega (existing logic)
    int commandStart = payloadStr.indexOf("\"command\":\"") + 11;
    int commandEnd = payloadStr.indexOf("\"", commandStart);
    
    if (commandStart > 10 && commandEnd > commandStart) {
      String command = payloadStr.substring(commandStart, commandEnd);
      Serial.print("UART to Mega: ");
      Serial.println(command);
      
      // Forward command to Mega via UART
      Serial1.println(command);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(2000);
  
  Serial.println("=== Enhanced MQTT+CAN Bridge Starting ===");
  
  // Setup W5500 Ethernet (existing)
  pinMode(W5500_CS_PIN, OUTPUT);
  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(10);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(200);
  
  // Initialize Ethernet
  Serial.println("Initializing W5500...");
  Ethernet.begin(mac, ip, gw, gw, sub);
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
  
  // Setup MCP2515 CAN (new)
  Serial.println("Initializing MCP2515...");
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_CLOCK);
  mcp2515.setNormalMode();
  Serial.println("CAN bus initialized");
  
  // Setup MQTT
  mqtt.setServer("192.168.200.233", 1883);
  mqtt.setCallback(callback);
  
  Serial.println("=== Enhanced MQTT+CAN Bridge Ready! ===");
}

void loop() {
  // MQTT connection
  if (!mqtt.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqtt.connect("enhanced_bridge01", "admin", "admin123")) {
      mqtt.subscribe("automation_dispensing");
      mqtt.subscribe("automation_dispensing_can");  // New CAN topic
      mqtt.subscribe("automation_milk");
      mqtt.subscribe("automation_grinding");
      mqtt.subscribe("automation_tampering");
      mqtt.subscribe("automation_slush");
      mqtt.subscribe("automation_ice");
      Serial.println("MQTT Connected!");
    } else {
      Serial.print("MQTT Failed: ");
      Serial.println(mqtt.state());
    }
  }
  mqtt.loop();
  
  // Process CAN messages
  processCAN();
  
  // UART data processing (existing)
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[idx] = 0;
      if (strstr(buf, "Scales A=") || strstr(buf, "LEAK_EMERGENCY")) {
        // Forward important data to MQTT
        mqtt.publish("dispenser/status", buf);
      }
      idx = 0;
    } else if (idx < 127) {
      buf[idx++] = c;
    }
  }
  
  delay(10);
} 