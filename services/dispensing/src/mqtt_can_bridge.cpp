#include <Arduino.h>
#include <SPI.h>
#define ENABLE_MQTT 0
#if ENABLE_MQTT
#include <Ethernet.h>
#include <PubSubClient.h>
#endif
#include <mcp2515.h>

// Network configuration (only if MQTT enabled)
#if ENABLE_MQTT
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01};
IPAddress ip(192, 168, 200, 211);
IPAddress gw(192, 168, 200, 1);
IPAddress sub(255, 255, 255, 0);
#endif

// W5500 pins (existing)
#if ENABLE_MQTT
#define W5500_CS_PIN   10
#define W5500_RST_PIN  9
#endif

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

// Frother CAN IDs
#define CAN_ID_FROTHER_CMD      0x320   // Frother command
#define CAN_ID_FROTHER_EVT      0x321   // Frother events/telemetry

// Liquid type mapping
#define LIQ_NORMAL_WATER                 1
#define LIQ_WHOLE_FAT_MILK               2
#define LIQ_LOW_FAT_MILK                 3
#define LIQ_OAT_MILK                     4
#define LIQ_SOY_MILK                     5
#define LIQ_ALMOND_MILK                  6
#define LIQ_LACTOSE_FREE_MILK            7
#define LIQ_WHITE_CHOCOLATE_SAUCE        8
#define LIQ_CARAMEL_SAUCE                9
#define LIQ_CONDENSE_MILK_SAUCE          10
#define LIQ_HAZELNUT_SYRUP               11
#define LIQ_VANILLA_SYRUP                12
#define LIQ_CARAMEL_SYRUP                13
#define LIQ_PEACHED_ICED_SYRUP           14
#define LIQ_PASSION_FRUIT_ICED_SYRUP     15
#define LIQ_ICE_TEA_SYRUP                16

#if ENABLE_MQTT
EthernetClient ethClient;
PubSubClient mqtt(ethClient);
#endif
MCP2515 mcp2515(CAN_CS_PIN);

static uint8_t can_error_count = 0;
static bool pending_dispense_ack = false;
static bool frother_busy = false;
static uint32_t last_heartbeat_ms = 0;

// Forward declaration to satisfy helper calls
static void can_reinit_normal();

// ====================== Frother helpers (CAN <-> UART) ======================
static inline void can_send_evt_ack(uint8_t cmd, uint8_t accepted, uint8_t busy) {
  struct can_frame msg; msg.can_id = CAN_ID_FROTHER_EVT; msg.can_dlc = 8;
  msg.data[0] = 0x00; // ACK
  msg.data[1] = cmd;
  msg.data[2] = accepted ? 1 : 0;
  msg.data[3] = busy ? 1 : 0;
  msg.data[4] = 0; msg.data[5] = 0; msg.data[6] = 0; msg.data[7] = 0;
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&msg);
    if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
  }
}

static inline void can_send_evt_done(uint8_t cmd, uint8_t errMask) {
  struct can_frame msg; msg.can_id = CAN_ID_FROTHER_EVT; msg.can_dlc = 8;
  msg.data[0] = 0x01; // DONE
  msg.data[1] = cmd;
  msg.data[2] = errMask;
  msg.data[3] = 0; msg.data[4] = 0; msg.data[5] = 0; msg.data[6] = 0; msg.data[7] = 0;
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&msg);
    if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
  }
}

static inline void can_send_evt_error(uint8_t code) {
  struct can_frame msg; msg.can_id = CAN_ID_FROTHER_EVT; msg.can_dlc = 8;
  msg.data[0] = 0x02; // ERROR
  msg.data[1] = code;
  msg.data[2] = 0; msg.data[3] = 0; msg.data[4] = 0; msg.data[5] = 0; msg.data[6] = 0; msg.data[7] = 0;
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&msg);
    if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
  }
}

static inline void can_send_evt_temp(uint16_t tempCx100, uint8_t valid) {
  struct can_frame msg; msg.can_id = CAN_ID_FROTHER_EVT; msg.can_dlc = 8;
  msg.data[0] = 0x03; // TEMP
  msg.data[1] = (uint8_t)((tempCx100 >> 8) & 0xFF);
  msg.data[2] = (uint8_t)(tempCx100 & 0xFF);
  msg.data[3] = valid ? 1 : 0;
  msg.data[4] = 0; msg.data[5] = 0; msg.data[6] = 0; msg.data[7] = 0;
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&msg);
    if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
  }
}

static inline void can_send_evt_state(uint8_t mode, uint8_t busy, uint8_t errMask) {
  struct can_frame msg; msg.can_id = CAN_ID_FROTHER_EVT; msg.can_dlc = 8;
  msg.data[0] = 0x04; // STATE
  msg.data[1] = mode;
  msg.data[2] = busy ? 1 : 0;
  msg.data[3] = errMask;
  msg.data[4] = 0; msg.data[5] = 0; msg.data[6] = 0; msg.data[7] = 0;
  {
    MCP2515::ERROR txres = mcp2515.sendMessage(&msg);
    if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
  }
}

static void can_reinit_normal() {
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_CLOCK);
  mcp2515.setNormalMode();
  can_error_count = 0;
}

char buf[96];
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
  if (strstr(liquid_name, "normal_water")) return LIQ_NORMAL_WATER;
  if (strstr(liquid_name, "whole_fat_milk")) return LIQ_WHOLE_FAT_MILK;
  if (strstr(liquid_name, "low_fat_milk")) return LIQ_LOW_FAT_MILK;
  if (strstr(liquid_name, "oat_milk")) return LIQ_OAT_MILK;
  if (strstr(liquid_name, "soy_milk")) return LIQ_SOY_MILK;
  if (strstr(liquid_name, "almond_milk")) return LIQ_ALMOND_MILK;
  if (strstr(liquid_name, "lactose_free_milk")) return LIQ_LACTOSE_FREE_MILK;
  if (strstr(liquid_name, "white_chocolate_sauce")) return LIQ_WHITE_CHOCOLATE_SAUCE;
  if (strstr(liquid_name, "caramel_sauce")) return LIQ_CARAMEL_SAUCE;
  if (strstr(liquid_name, "condense_milk_sauce")) return LIQ_CONDENSE_MILK_SAUCE;
  if (strstr(liquid_name, "hazelnut_syrup")) return LIQ_HAZELNUT_SYRUP;
  if (strstr(liquid_name, "vanilla_syrup")) return LIQ_VANILLA_SYRUP;
  if (strstr(liquid_name, "caramel_syrup")) return LIQ_CARAMEL_SYRUP;
  if (strstr(liquid_name, "peached_iced_syrup")) return LIQ_PEACHED_ICED_SYRUP;
  if (strstr(liquid_name, "passion_fruit_iced_syrup")) return LIQ_PASSION_FRUIT_ICED_SYRUP;
  if (strstr(liquid_name, "ice_tea_syrup")) return LIQ_ICE_TEA_SYRUP;
  return LIQ_CARAMEL_SAUCE; // sensible default
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
      if (++can_error_count >= 3) { can_reinit_normal(); }
    }
  }
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
      if (++can_error_count >= 3) { can_reinit_normal(); }
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
    if (canMsg.can_id == CAN_ID_FROTHER_CMD && canMsg.can_dlc >= 2) {
      // Payload: b0=cmd, b1=flags(hasArgs), b2.. args as u16 scaled as specified
      uint8_t cmd  = canMsg.data[0];
      uint8_t flags = canMsg.data[1];
      // Forward to Mega over UART in compact CSV: FR_CMD:cmd,flags,arg1,arg2,arg3
      Serial1.print("FR_CMD:");
      Serial1.print((int)cmd);
      Serial1.print(",");
      Serial1.print((int)flags);
      // Up to 3 u16 args (big endian as CAN suggested), but we just pass raw bytes orderless as values decoded here
      if (canMsg.can_dlc >= 4) {
        uint16_t a1 = ((uint16_t)canMsg.data[2] << 8) | (uint16_t)canMsg.data[3];
        Serial1.print(","); Serial1.print((int)a1);
      }
      if (canMsg.can_dlc >= 6) {
        uint16_t a2 = ((uint16_t)canMsg.data[4] << 8) | (uint16_t)canMsg.data[5];
        Serial1.print(","); Serial1.print((int)a2);
      }
      if (canMsg.can_dlc >= 8) {
        uint16_t a3 = ((uint16_t)canMsg.data[6] << 8) | (uint16_t)canMsg.data[7];
        Serial1.print(","); Serial1.print((int)a3);
      }
      Serial1.println();
      // Immediate ACK on CAN
      can_send_evt_ack(cmd, 1, frother_busy ? 1 : 0);
      continue;
    }
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
        // Defer ACK on CAN until job completes
        pending_dispense_ack = true;
      }
      else if (cmd.cmd == CMD_SET_LAG) {
        // Apply per-motor lag override on Mega via UART
        const char* motor_name = getMotorNameById(cmd.motor_id);
        float lag_g = ((float)cmd.weight_dg) / 10.0f;
        uint8_t speed01 = cmd.reserved[0] ? 1 : 0;
        if (motor_name) {
          Serial1.print("LAGM "); Serial1.print(motor_name); Serial1.print(" "); Serial1.print((int)speed01); Serial1.print(" "); Serial1.println(lag_g, 1);
        }
        // Immediate ACK for SET_LAG
        struct can_frame ackLag;
        ackLag.can_id  = CAN_ID_DISPENSING_ACK;
        ackLag.can_dlc = 8;
        ackLag.data[0] = 0x01; ackLag.data[1] = 0; ackLag.data[2] = 0; ackLag.data[3] = 0;
        ackLag.data[4] = 0;    ackLag.data[5] = 0; ackLag.data[6] = 0; ackLag.data[7] = 0;
        {
          MCP2515::ERROR txres = mcp2515.sendMessage(&ackLag);
          if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
        }
      }
      continue;
    }
    else if (canMsg.can_id == CAN_ID_DISPENSING_DATA) {
      // (optional) ignore or handle scale data
    }
  }
}

#if ENABLE_MQTT
void callback(char* topic, byte* payload, unsigned int len) {
  payload[len] = 0;
  String payloadStr = String((char*)payload);
  String topicStr = String(topic);
  if (topicStr == "automation_dispensing_lag") {
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
      Serial1.print("LAGM "); Serial1.print(motor); Serial1.print(" "); Serial1.print(speed); Serial1.print(" "); Serial1.println(lag, 1);
      uint8_t motor_id = getMotorIdFromName(motor.c_str());
      sendCANSetLag(motor_id, lag, (uint8_t)(speed ? 1 : 0));
    }
    return;
  }
  if (strstr(topic, "automation_dispensing")) {
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
    uint8_t motor_id = getMotorIdFromName(motor.c_str());
    uint8_t liquid_type = getLiquidTypeId(ingredient.c_str());
    sendCANDispenseCommand(motor_id, weight, liquid_type);
    uint16_t weight_dg = (uint16_t)(weight * 10.0f);
    Serial1.print("CAN:");
    Serial1.print((int)motor_id);
    Serial1.print(",");
    Serial1.print((int)weight_dg);
    Serial1.print(",");
    Serial1.println((int)liquid_type);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(300);
  
  // Setup W5500 Ethernet (if enabled)
#if ENABLE_MQTT
  pinMode(W5500_CS_PIN, OUTPUT);
  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(10);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(80);
  Ethernet.begin(mac, ip, gw, gw, sub);
#endif
  
  // Setup MCP2515 CAN
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_CLOCK);
  mcp2515.setNormalMode();
  
  // Setup MQTT
#if ENABLE_MQTT
  mqtt.setServer("192.168.200.233", 1883);
  mqtt.setCallback(callback);
#endif
}

void loop() {
  #if ENABLE_MQTT
  if (!mqtt.connected()) {
    if (mqtt.connect("bridge01", "admin", "admin123")) {
      mqtt.subscribe("automation_dispensing");
      mqtt.subscribe("automation_dispensing_lag");
    }
  }
  mqtt.loop();
  #endif
  processCAN();
  
  // Periodic CAN heartbeat every 30 seconds: ID 0x3FF, payload FF FF
  if (millis() - last_heartbeat_ms >= 30000UL) {
    struct can_frame hb;
    hb.can_id = 0x110;
    hb.can_dlc = 2;
    hb.data[0] = 0xFF; hb.data[1] = 0xFF;
    {
      MCP2515::ERROR txres = mcp2515.sendMessage(&hb);
      if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
    }
    last_heartbeat_ms = millis();
  }
  
  // UART bridge of status from Mega → MQTT
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[idx] = 0;
      if (strstr(buf, "FR_EVT:")) {
        // Parse frother event line formats
        String s = String(buf);
        if (s.startsWith("FR_EVT:DONE,")) {
          int p1 = s.indexOf(',');
          int p2 = s.indexOf(',', p1 + 1);
          uint8_t cmd = (uint8_t)s.substring(p1 + 1, p2).toInt();
          uint8_t err = (uint8_t)s.substring(p2 + 1).toInt();
          frother_busy = 0;
          can_send_evt_done(cmd, err);
        } else if (s.startsWith("FR_EVT:ERROR,")) {
          int p1 = s.indexOf(',');
          uint8_t code = (uint8_t)s.substring(p1 + 1).toInt();
          frother_busy = 0;
          can_send_evt_error(code);
        } else if (s.startsWith("FR_EVT:TEMP,")) {
          int p1 = s.indexOf(',');
          int p2 = s.indexOf(',', p1 + 1);
        long t_parsed = s.substring(p1 + 1, p2).toInt();
        if (t_parsed < 0) { t_parsed = 0; }
        if (t_parsed > 65535) { t_parsed = 65535; }
        uint16_t t = (uint16_t)t_parsed;
        uint8_t valid = (uint8_t)s.substring(p2 + 1).toInt();
        can_send_evt_temp(t, valid);
        } else if (s.startsWith("FR_EVT:STATE,")) {
          int p1 = s.indexOf(',');
          int p2 = s.indexOf(',', p1 + 1);
          int p3 = s.indexOf(',', p2 + 1);
          uint8_t mode = (uint8_t)s.substring(p1 + 1, p2).toInt();
          uint8_t busy = (uint8_t)s.substring(p2 + 1, p3).toInt();
          uint8_t err = (uint8_t)s.substring(p3 + 1).toInt();
          frother_busy = busy ? 1 : 0;
          can_send_evt_state(mode, busy, err);
        }
      } else if (strstr(buf, "Scales ") || strstr(buf, "LEAK")) {
        // MQTT disabled: skip publishing
      }
      // When Mega reports completion, send deferred CAN ACK
      if (pending_dispense_ack && strstr(buf, "State=COMPLETED")) {
        struct can_frame ackMsg;
        ackMsg.can_id  = CAN_ID_DISPENSING_ACK;
        ackMsg.can_dlc = 8;
        ackMsg.data[0] = 0x01; ackMsg.data[1] = 0; ackMsg.data[2] = 0; ackMsg.data[3] = 0;
        ackMsg.data[4] = 0;    ackMsg.data[5] = 0; ackMsg.data[6] = 0; ackMsg.data[7] = 0;
        {
          MCP2515::ERROR txres = mcp2515.sendMessage(&ackMsg);
          if (txres != MCP2515::ERROR_OK) { if (++can_error_count >= 3) { can_reinit_normal(); } }
        }
        pending_dispense_ack = false;
      }
      idx = 0;
    } else if (idx < (int)sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
  delay(5);
} 