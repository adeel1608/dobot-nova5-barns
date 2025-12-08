#include <Arduino.h>
#include <Wire.h>

/* ================= No RS485 - Direct UART ================= */
// RS485 removed - using direct UART to MQTT bridge

/* ================= UART to MQTT Bridge ================= */
#define BRIDGE_SERIAL    Serial2       // UART2 (RX2=17, TX2=16) to Micro
#define BRIDGE_BAUD      115200
#define BRIDGE_CONFIG    SERIAL_8N1

/* ================= I2C scale addresses ================= */
#define SLV_A 0x28  // Milk scale
#define SLV_B 0x29  // Sauce scale

/* ======== Bridge registers (updated from the two Nanos) ======== */
volatile int16_t  regW_A = 0;    // decigrams
volatile uint16_t regS_A = 0;    // status
volatile int16_t  regW_B = 0;
volatile uint16_t regS_B = 0;

/* ================= MOTOR CONTROL CONFIGURATION ================= */
// Motor control pins based on YOUR ACTUAL PCB (from working test)
struct MotorChannel {
  uint8_t motor_pin;  // Individual motor control pin (MILK/SAUCE)
  uint8_t speed_pin;  // Global speed control pin (D23)
  bool    active;     // Current state
  const char* label;  // Motor name
};

// Your ACTUAL working motor configuration
static MotorChannel MOTORS[] = {
  {2,  23, false, "MILK1"},   // MILK1=D2, SPEED=D23 (global)
  {3,  23, false, "MILK2"},   // MILK2=D3, SPEED=D23 (global)
  {4,  23, false, "MILK3"},   // MILK3=D4, SPEED=D23 (global)
  {5,  23, false, "MILK4"},   // MILK4=D5, SPEED=D23 (global)
  {6,  23, false, "MILK5"},   // MILK5=D6, SPEED=D23 (global)
  {7,  23, false, "MILK6"},   // MILK6=D7, SPEED=D23 (global)
  {8,  23, false, "MILK7"},   // MILK7=D8, SPEED=D23 (global)
  {9,  23, false, "MILK8"},   // MILK8=D9, SPEED=D23 (global)
  {10, 23, false, "SAUCE1"},  // SAUCE1=D10, SPEED=D23 (global)
  {11, 23, false, "SAUCE2"},  // SAUCE2=D11, SPEED=D23 (global)
  {12, 23, false, "SAUCE3"},  // SAUCE3=D12, SPEED=D23 (global)
  {25, 23, false, "SAUCE4"},  // SAUCE4=D25, SPEED=D23 (global)
  {27, 23, false, "SAUCE5"},  // SAUCE5=D27, SPEED=D23 (global)
  {29, 23, false, "SAUCE6"},  // SAUCE6=D29, SPEED=D23 (global)
  {31, 23, false, "SAUCE7"},  // SAUCE7=D31, SPEED=D23 (global)
  {33, 23, false, "SAUCE8"},  // SAUCE8=D33, SPEED=D23 (global)
  {35, 23, false, "SAUCE9"},  // SAUCE9=D35, SPEED=D23 (global)
  {37, 23, false, "SAUCE10"}, // SAUCE10=D37, SPEED=D23 (global)
  {39, 23, false, "SAUCE11"}, // SAUCE11=D39, SPEED=D23 (global)
  {41, 23, false, "SAUCE12"}, // SAUCE12=D41, SPEED=D23 (global)
  {43, 23, false, "SAUCE13"}, // SAUCE13=D43, SPEED=D23 (global)
  {45, 23, false, "SAUCE14"}, // SAUCE14=D45, SPEED=D23 (global)
  {47, 23, false, "SAUCE15"}, // SAUCE15=D47, SPEED=D23 (global)
  {49, 23, false, "RINSER"},  // RINSER=D49, SPEED=D23 (global)
};

static const uint8_t NUM_MOTORS = sizeof(MOTORS) / sizeof(MOTORS[0]);

/* ================= LEAK DETECTION CONFIGURATION ================= */
#define LEAK_SENSOR_1_PIN    A5   // Leak sensor 1 input
#define LEAK_SENSOR_2_PIN    A8   // Leak sensor 2 input  
#define LEAK_POWER_1_PIN     A6   // Power pin for leak sensor 1
#define LEAK_POWER_2_PIN     A9   // Power pin for leak sensor 2

#define LEAK_THRESHOLD       0  // Disable leak detection temporarily
#define LEAK_CHECK_INTERVAL  100  // Check every 100ms

struct LeakDetector {
  uint8_t sensor_pin;
  uint8_t power_pin;
  bool leak_detected;
  uint32_t last_check;
  const char* label;
};

static LeakDetector LEAK_SENSORS[] = {
  {LEAK_SENSOR_1_PIN, LEAK_POWER_1_PIN, false, 0, "LEAK1"},
  {LEAK_SENSOR_2_PIN, LEAK_POWER_2_PIN, false, 0, "LEAK2"}
};

static const uint8_t NUM_LEAK_SENSORS = sizeof(LEAK_SENSORS) / sizeof(LEAK_SENSORS[0]);
static bool leak_emergency_triggered = false;

/* ================= CAN BUS CONFIGURATION ================= */
// Note: MCP2515 is connected to Arduino Micro, not Mega
// Mega receives CAN commands via UART from Micro
// This section is for reference and future direct CAN integration

#define CAN_ID_DISPENSING_CMD   0x110   // Commands to dispensing system
#define CAN_ID_DISPENSING_ACK   0x111   // ACK from dispensing system  
#define CAN_ID_DISPENSING_DATA  0x112   // Scale data from dispensing system

// Liquid type mapping (detailed; matches LIQUID_LAGS names)
// These numeric IDs are carried over CAN from the Micro bridge
enum LiquidTypeId {
  LIQ_NORMAL_WATER = 1,
  LIQ_WHOLE_FAT_MILK = 2,
  LIQ_LOW_FAT_MILK = 3,
  LIQ_OAT_MILK = 4,
  LIQ_SOY_MILK = 5,
  LIQ_ALMOND_MILK = 6,
  LIQ_LACTOSE_FREE_MILK = 7,
  LIQ_WHITE_CHOCOLATE_SAUCE = 8,
  LIQ_CARAMEL_SAUCE = 9,
  LIQ_CONDENSE_MILK_SAUCE = 20,  // sauce12
  LIQ_HAZELNUT_SYRUP = 11,
  LIQ_VANILLA_SYRUP = 12,
  LIQ_CARAMEL_SYRUP = 13,
  LIQ_PEACHED_ICED_SYRUP = 14,
  LIQ_PASSION_FRUIT_ICED_SYRUP = 15,
  LIQ_ICE_TEA_SYRUP = 16
};

// Motor control state (no Modbus registers needed)
volatile uint16_t motorStatus = 0;                // Motor status bits
volatile bool globalSpeedState = false;          // Global speed pin state

/* ================= AUTOMATED DISPENSING SYSTEM ================= */
struct CupType {
  const char* name;
  float weight;
};

// Cup database
const CupType CUPS[] = {
  {"H7",  5.3},  {"H9",  13.1}, {"H12", 17.0},
  {"C7",  6.7},  {"C9",  10.3}, {"C12", 11.0}, {"C12B", 14.2}
};
const uint8_t NUM_CUPS = sizeof(CUPS) / sizeof(CUPS[0]);

// Motor mapping
struct MotorMap {
  const char* name;
  uint8_t pin;
};

const MotorMap MOTOR_MAP[] = {
  {"milk1", 2}, {"milk2", 3}, {"milk3", 4}, {"milk4", 5},
  {"milk5", 6}, {"milk6", 7}, {"milk7", 8}, {"milk8", 9},
  {"sauce1", 10}, {"sauce2", 11}, {"sauce3", 12}, {"sauce4", 25},
  {"sauce5", 27}, {"sauce6", 29}, {"sauce7", 31}, {"sauce8", 33},
  {"sauce9", 35}, {"sauce10", 37}, {"sauce11", 39}, {"sauce12", 41},
  {"sauce13", 43}, {"sauce14", 45}, {"sauce15", 47},
  {"rinser", 49}
};
const uint8_t NUM_MOTOR_MAP = sizeof(MOTOR_MAP) / sizeof(MOTOR_MAP[0]);

// Forward declaration (defined later)
static float getLiquidLag(const char* liquid_name, bool speed_enabled);

// Per-motor lag overrides (runtime tunable)
static float motorLagOverrideSpeed0[NUM_MOTOR_MAP];
static float motorLagOverrideSpeed1[NUM_MOTOR_MAP];

static int getMotorIndexByName(const char* motor_name) {
  for (uint8_t i = 0; i < NUM_MOTOR_MAP; i++) {
    if (strcmp(MOTOR_MAP[i].name, motor_name) == 0) {
      return (int)i;
    }
  }
  return -1;
}

static void initLagOverrides() {
  for (uint8_t i = 0; i < NUM_MOTOR_MAP; i++) {
    motorLagOverrideSpeed0[i] = -1.0f;
    motorLagOverrideSpeed1[i] = -1.0f;
  }
}

static float getEffectiveLag(const char* liquid_name, const char* motor_name, bool speed_enabled) {
  int idx = getMotorIndexByName(motor_name);
  if (idx >= 0) {
    float v = speed_enabled ? motorLagOverrideSpeed1[idx] : motorLagOverrideSpeed0[idx];
    if (v >= 0.0f) return v;
  }
  // Fallback to liquid-based lag if no per-motor override exists
  return getLiquidLag(liquid_name, speed_enabled);
}

// Dispensing state
enum DispenseState {
  IDLE,
  DISPENSING,
  COMPLETED
};

struct DispenseJob {
  DispenseState state;
  uint8_t scale_addr;     // SLV_A or SLV_B
  uint8_t motor_pin;      // Motor pin number
  float cup_weight;       // Expected cup weight
  float target_weight;    // Target dispensing weight
  float cup_tolerance;    // Cup detection tolerance (±g)
  float minimum_target;   // Minimum target weight (stop when reached)
  uint32_t stable_time;   // Time weight has been stable
  float last_weight;      // Last stable weight reading
  float tare_offset;      // Weight offset after taring
  bool motor_running;     // Motor state
  uint32_t timeout_ms;    // Job timeout
  uint32_t job_start_time; // Job start timestamp
};

DispenseJob current_job = {IDLE, 0, 0, 0, 0, 0.2, 0, 0, 0, 0, false, 60000, 0};
bool global_speed_enabled = false;
// Frother bridge state (Nano Every on Serial1)
static bool frother_active = false;
static char nanoBuf[96];
static int nanoIdx = 0;
// Rinser pulse control (Mega pin 49)
static bool rinser_active = false;
static uint32_t rinser_end_ms = 0;
// Liquid-specific lag compensation
struct LiquidLag {
  const char* name;
  float lag_speed0;  // Lag for speed 0 (faster)
  float lag_speed1;  // Lag for speed 1 (slower)
};

const LiquidLag LIQUID_LAGS[] = {
  {"normal_water",   18.0, 11.0},  
  {"whole_fat_milk",   25.0, 15.0},  
  {"low_fat_milk",   25.0, 15.0},  
  {"oat_milk",   25.0, 15.0},  
  {"soy_milk",   25.0, 15.0},  
  {"almond_milk",   25.0, 15.0},  
  {"lactose_free_milk",   25.0, 15.0},  
  {"white_chocolate_sauce",   1.0, 1.0},  
  {"caramel_sauce",   25.0, 15.0},  
  {"condense_milk_sauce",    15.0, 9.0},   
  {"hazelnut_syrup",   12.0, 7.0},   
  {"vanilla_syrup", 1.0,  1.0},   
  {"caramel_syrup",   15.0, 8.0},   
  {"peached_iced_syrup",   2.0,  1.5},   
  {"passion_fruit_iced_syrup",   2.0,  1.5},
  {"ice_tea_syrup",   2.0,  1.5},
};
const uint8_t NUM_LIQUID_LAGS = sizeof(LIQUID_LAGS) / sizeof(LIQUID_LAGS[0]);

float custom_lag_speed0 = 11.0; // Default lag values  
float custom_lag_speed1 = 7.0;

/* ================= LEAK DETECTION FUNCTIONS ================= */
static void initLeakDetectors() {
  // Initialize leak sensor power pins (set HIGH to power sensors)
  for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
    pinMode(LEAK_SENSORS[i].power_pin, OUTPUT);
    digitalWrite(LEAK_SENSORS[i].power_pin, HIGH);  // Power ON
    pinMode(LEAK_SENSORS[i].sensor_pin, INPUT_PULLUP);  // Use pullup to avoid floating
    LEAK_SENSORS[i].leak_detected = false;
    LEAK_SENSORS[i].last_check = 0;
  }
  
  // Wait for sensors to stabilize
  delay(100);
  
  // Show initial readings
  Serial.println(F("Leak detection initialized: A5,A8 (sensors), A6,A9 (power HIGH)"));
  for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
    int initial_value = analogRead(LEAK_SENSORS[i].sensor_pin);
    Serial.print(F("  ")); Serial.print(LEAK_SENSORS[i].label);
    Serial.print(F(" initial reading: ")); Serial.println(initial_value);
  }
}

static bool checkLeakSensors() {
  uint32_t now = millis();
  bool leak_found = false;
  
  for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
    if (now - LEAK_SENSORS[i].last_check >= LEAK_CHECK_INTERVAL) {
      LEAK_SENSORS[i].last_check = now;
      
      int sensor_value = analogRead(LEAK_SENSORS[i].sensor_pin);
      bool current_leak = sensor_value < LEAK_THRESHOLD;  // LOW value = leak (liquid bridges sensor)
      
      // DEBUG: Print sensor values occasionally
      static uint32_t last_debug = 0;
      if (millis() - last_debug > 5000) {
        Serial.print("LEAK DEBUG: A5="); Serial.print(analogRead(A5));
        Serial.print(", A8="); Serial.println(analogRead(A8));
        last_debug = millis();
      }
      
      // Detect new leak
      if (current_leak && !LEAK_SENSORS[i].leak_detected) {
        LEAK_SENSORS[i].leak_detected = true;
        Serial.print(F("🚨 LEAK DETECTED on ")); 
        Serial.print(LEAK_SENSORS[i].label);
        Serial.print(F(" (pin A")); Serial.print(LEAK_SENSORS[i].sensor_pin - A0);
        Serial.print(F(", value=")); Serial.print(sensor_value); Serial.println(F(")"));
        leak_found = true;
      }
      // Detect leak cleared
      else if (!current_leak && LEAK_SENSORS[i].leak_detected) {
        LEAK_SENSORS[i].leak_detected = false;
        Serial.print(F("✅ Leak cleared on ")); Serial.println(LEAK_SENSORS[i].label);
      }
    }
  }
  
  return leak_found;
}

/* ================= MOTOR CONTROL FUNCTIONS ================= */
static void initMotors() {
  // Initialize individual motor pins
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTORS[i].motor_pin, OUTPUT);
    digitalWrite(MOTORS[i].motor_pin, LOW);
    MOTORS[i].active = false;
  }
  
  // Initialize global speed pin (D23)
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
}

static void setMotorControl(uint8_t motor_idx, bool motor_on, bool speed_on) {
  if (motor_idx >= NUM_MOTORS) return;
  
  MotorChannel &motor = MOTORS[motor_idx];
  
  // Control individual motor pin
  digitalWrite(motor.motor_pin, motor_on ? HIGH : LOW);
  motor.active = motor_on;
  
  // Control global speed pin (independent of individual motors)
  if (speed_on != globalSpeedState) {
    digitalWrite(23, speed_on ? HIGH : LOW);
    globalSpeedState = speed_on;
  }
  
  // Update status register
  if (motor.active) {
    motorStatus |= (1 << motor_idx);
  } else {
    motorStatus &= ~(1 << motor_idx);
  }
}

static void setGlobalSpeed(bool speed_on) {
  digitalWrite(23, speed_on ? HIGH : LOW);
  globalSpeedState = speed_on;
}

static void stopAllMotors() {
  // Stop all individual motors
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(MOTORS[i].motor_pin, LOW);
    MOTORS[i].active = false;
  }
  motorStatus = 0;
}

static void emergencyStop() {
  // Stop all motors AND turn off global speed
  stopAllMotors();
  setGlobalSpeed(false);
  current_job.state = IDLE;
  current_job.motor_running = false;
}

static void handleLeakEmergency() {
  if (!leak_emergency_triggered) {
    leak_emergency_triggered = true;
    Serial.println(F("🚨🚨 LEAK EMERGENCY - STOPPING ALL MOTORS 🚨🚨"));
    
    // Emergency stop all motors immediately
    emergencyStop();
    
    // Send alert to MQTT bridge
    BRIDGE_SERIAL.println("LEAK_EMERGENCY_DETECTED");
    // Also stop frother subsystem (Nano Every)
    Serial1.println("FR_CMD:0,0"); // OFF immediately
    
    // Auto-reset leak emergency after 5 seconds if no active dispensing
    // This prevents permanent lockout from false triggers
  }
}

/* ================= DISPENSING SYSTEM FUNCTIONS ================= */
float getCupWeight(const char* cup_name) {
  for (uint8_t i = 0; i < NUM_CUPS; i++) {
    if (strcmp(CUPS[i].name, cup_name) == 0) {
      return CUPS[i].weight;
    }
  }
  return 0; // Cup not found
}

uint8_t getMotorPin(const char* motor_name) {
  for (uint8_t i = 0; i < NUM_MOTOR_MAP; i++) {
    if (strcmp(MOTOR_MAP[i].name, motor_name) == 0) {
      return MOTOR_MAP[i].pin;
    }
  }
  return 0; // Motor not found
}

float getLiquidLag(const char* liquid_name, bool speed_enabled) {
  for (uint8_t i = 0; i < NUM_LIQUID_LAGS; i++) {
    if (strcmp(LIQUID_LAGS[i].name, liquid_name) == 0) {
      return speed_enabled ? LIQUID_LAGS[i].lag_speed1 : LIQUID_LAGS[i].lag_speed0;
    }
  }
  // Default lag if liquid not found
  return speed_enabled ? custom_lag_speed1 : custom_lag_speed0;
}

const char* getMotorNameById(uint8_t motor_id) {
  // milk1-8 = IDs 1-8, sauce1-15 = IDs 9-23, rinser = ID 24
  if (motor_id >= 1 && motor_id <= 8) {
    static char milk_name[8];
    sprintf(milk_name, "milk%d", motor_id);
    return milk_name;
  }
  if (motor_id >= 9 && motor_id <= 23) {
    static char sauce_name[10];
    sprintf(sauce_name, "sauce%d", motor_id - 8);
    return sauce_name;
  }
  if (motor_id == 24) return "rinser";
  return nullptr;
}

const char* getLiquidNameById(uint8_t liquid_type) {
  switch (liquid_type) {
    case LIQ_NORMAL_WATER: return "normal_water";
    case LIQ_WHOLE_FAT_MILK: return "whole_fat_milk";
    case LIQ_LOW_FAT_MILK: return "low_fat_milk";
    case LIQ_OAT_MILK: return "oat_milk";
    case LIQ_SOY_MILK: return "soy_milk";
    case LIQ_ALMOND_MILK: return "almond_milk";
    case LIQ_LACTOSE_FREE_MILK: return "lactose_free_milk";
    case LIQ_WHITE_CHOCOLATE_SAUCE: return "white_chocolate_sauce";
    case LIQ_CARAMEL_SAUCE: return "caramel_sauce";
    case LIQ_CONDENSE_MILK_SAUCE: return "condense_milk_sauce";
    case LIQ_HAZELNUT_SYRUP: return "hazelnut_syrup";
    case LIQ_VANILLA_SYRUP: return "vanilla_syrup";
    case LIQ_CARAMEL_SYRUP: return "caramel_syrup";
    case LIQ_PEACHED_ICED_SYRUP: return "peached_iced_syrup";
    case LIQ_PASSION_FRUIT_ICED_SYRUP: return "passion_fruit_iced_syrup";
    case LIQ_ICE_TEA_SYRUP: return "ice_tea_syrup";
    default: return "sauce";
  }
}

bool startDispenseJob(const char* cup_name, const char* motor_name, float target_weight, const char* liquid_type = NULL) {
  if (frother_active) {
    Serial.println(F("FROTHER ACTIVE - dispensing blocked"));
    return false;
  }
  if (current_job.state == DISPENSING) {
    Serial.println(F("Job already running! Use STOP first."));
    return false;
  }
  
  // Find motor pin
  uint8_t motor_pin = getMotorPin(motor_name);
  if (motor_pin == 0) {
    Serial.print(F("Unknown motor: ")); Serial.println(motor_name);
    return false;
  }
  
  // Determine scale (milk vs sauce)
  uint8_t scale_addr = SLV_A; // Default to milk
  if (strstr(motor_name, "sauce") != NULL) {
    scale_addr = SLV_B; // Use sauce scale
  }
  
  // Get current weight and tare immediately
  float current_weight = 0;
  if (scale_addr == SLV_A) {
    current_weight = regW_A / 10.0;
  } else {
    current_weight = regW_B / 10.0;
  }
  
  // Calculate lag based on liquid type or motor type
  const char* liquid_for_lag = liquid_type;
  if (!liquid_for_lag) {
    // Auto-detect from motor name
    if (strstr(motor_name, "milk") != NULL) liquid_for_lag = "milk";
    else if (strstr(motor_name, "sauce") != NULL) liquid_for_lag = "sauce";
    else liquid_for_lag = "sauce"; // Default
  }
  
  float motor_lag = getEffectiveLag(liquid_for_lag, motor_name, global_speed_enabled);
  
  // Initialize job
  current_job.state = DISPENSING;
  current_job.scale_addr = scale_addr;
  current_job.motor_pin = motor_pin;
  current_job.target_weight = target_weight;
  current_job.minimum_target = max(0.5, target_weight - motor_lag); // Never go below 0.5g
  current_job.tare_offset = current_weight; // Tare now
  current_job.motor_running = true;
  current_job.job_start_time = millis();
  
  // Turn on motor and speed immediately
  digitalWrite(motor_pin, HIGH);
  if (global_speed_enabled) digitalWrite(23, HIGH);
  
  Serial.print(F("TARED! Starting ")); Serial.print(motor_name);
  Serial.print(F(" (")); Serial.print(liquid_for_lag); Serial.print(F(")"));
  Serial.print(F(" -> ")); Serial.print(target_weight, 1); 
  Serial.print(F("g (stops at ")); Serial.print(current_job.minimum_target, 1);
  Serial.println(F("g)"));
  
  return true;
}

void processDispenseJob() {
  if (current_job.state == IDLE) return;
  
  // Get current weight from appropriate scale
  float current_weight = 0;
  if (current_job.scale_addr == SLV_A) {
    current_weight = regW_A / 10.0; // Convert decigrams to grams
  } else {
    current_weight = regW_B / 10.0;
  }
  
  if (current_job.state == DISPENSING) {
    // Calculate net weight (subtract tare offset)
    float net_weight = current_weight - current_job.tare_offset;
    
    // DEBUG: Show what's happening
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 500) { // Debug every 500ms
      Serial.print(F("DEBUG: Current=")); Serial.print(current_weight, 1);
      Serial.print(F("g, Tare=")); Serial.print(current_job.tare_offset, 1);
      Serial.print(F("g, Net=")); Serial.print(net_weight, 1);
      Serial.print(F("g, Stop at=")); Serial.print(current_job.minimum_target, 1);
      Serial.println(F("g"));
      last_debug = millis();
    }
    
    // Check if minimum target reached
    if (net_weight >= current_job.minimum_target) {
      // Stop motor
      digitalWrite(current_job.motor_pin, LOW);
      current_job.motor_running = false;
      current_job.state = COMPLETED;
      
      Serial.print(F("STOP! Net weight: ")); Serial.print(net_weight, 1);
      Serial.print(F("g (Target: ")); Serial.print(current_job.target_weight, 1);
      Serial.println(F("g) - Remove cup"));
    }
  }
  else if (current_job.state == COMPLETED) {
    // Check if cup is removed (weight drops significantly)
    if (current_weight < (current_job.tare_offset - 2.0)) { // Cup removed
      if (abs(current_weight - current_job.last_weight) < 0.1) {
        current_job.stable_time++;
        if (current_job.stable_time > 10) { // Stable for 2+ seconds
          Serial.println(F("Cup removed! Auto-taring... Ready for next job."));
          current_job.state = IDLE;
          current_job.stable_time = 0;
          // Auto-tare after cup removal would happen here
        }
      } else {
        current_job.stable_time = 0;
      }
      current_job.last_weight = current_weight;
    }
  }
}

/* ================= No CRC16 needed - UART communication ================= */
// CRC16 removed - using simple UART communication

/* ================= I2C bridge (same protocol as before) ================= */
static inline bool i2cWrite1(uint8_t addr, uint8_t b) {
  Wire.beginTransmission(addr);
  Wire.write(b);
  return Wire.endTransmission(false) == 0;
}
static inline bool i2cRequest(uint8_t addr, uint8_t n, uint32_t to_ms=50) {
  uint32_t t0 = millis();
  uint8_t got = Wire.requestFrom(addr, n);
  if (got == n) return true;
  while (millis() - t0 < to_ms) {
    if (Wire.available() >= n) return true;
    delay(1);
  }
  return (Wire.available() >= n);
}
static bool readWeight(uint8_t addr, int16_t &dg, uint16_t &st) {
  if (!i2cWrite1(addr, 0x01)) return false;     // READ_WEIGHT
  if (!i2cRequest(addr, 3))  return false;
  st = Wire.read();
  dg = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

/* =================== No Modbus - Direct UART Communication =================== */
// Modbus code removed - using MQTT bridge via UART2

// All Modbus functions removed - using MQTT bridge via UART2

/* ========================= Setup / Loop ========================= */
void setup() {
  // Set pin 51 as input (FAULTBUS)
  pinMode(51, INPUT);

  // Initialize UART to MQTT bridge
  BRIDGE_SERIAL.begin(BRIDGE_BAUD, BRIDGE_CONFIG);
  
  Wire.begin();
  Wire.setClock(100000);

  // Initialize motor control
  initMotors();
  
  // Initialize leak detection system
  initLeakDetectors();

  // Initialize lag overrides
  initLagOverrides();

  Serial.begin(115200);
  Serial1.begin(115200);
  delay(200);
  Serial.println(F("MEGA: Intelligent Dispensing System with MQTT Bridge"));
  Serial.println(F("UART2 (115200) to MQTT Bridge on pins 16/17"));
  Serial.print(F("Motors configured: ")); Serial.println(NUM_MOTORS);
  Serial.print(F("Leak sensors configured: ")); Serial.println(NUM_LEAK_SENSORS);
  Serial.println(F("🛡️ Leak Detection ACTIVE"));
  Serial.println(F("Automated Dispensing Ready!"));
}

// Modbus frame reading removed - using MQTT bridge via UART2

uint32_t tPoll = 0;
const uint32_t POLL_MS = 200;

// Enhanced command processing for dispensing system
void processSerialCommands() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  // Parse MQTT commands: CMD:liquid=caramel,qty=10
  if (cmd.startsWith("CMD:")) {
    String params = cmd.substring(4); // Remove "CMD:"
    int comma = params.indexOf(',');
    
    if (comma > 0) {
      String liquid_param = params.substring(0, comma);
      String qty_param = params.substring(comma + 1);
      
      // Parse liquid=caramel
      if (liquid_param.startsWith("liquid=")) {
        String liquid_type = liquid_param.substring(7);
        
        // Parse qty=10
        if (qty_param.startsWith("qty=")) {
          float target_weight = qty_param.substring(4).toFloat();
          
          // Auto-select motor based on liquid type
          String motor_name = "sauce1"; // Default motor
          if (liquid_type.indexOf("milk") >= 0) {
            motor_name = "milk1";
          }
          
          Serial.print(F("MQTT Command: ")); Serial.print(liquid_type);
          Serial.print(F(" -> ")); Serial.print(target_weight, 1); Serial.println(F("g"));
          
          startDispenseJob("AUTO", motor_name.c_str(), target_weight, liquid_type.c_str());
          return;
        }
      }
    }
    Serial.println(F("Invalid CMD format. Expected: CMD:liquid=type,qty=amount"));
    return;
  }
  
  // Parse dispensing commands: H7_milk_1_100 or C9_sauce_10_8 or H7_sauce_1_caramel_10
  if (cmd.indexOf('_') > 0) {
    int underscore1 = cmd.indexOf('_');
    int underscore2 = cmd.indexOf('_', underscore1 + 1);
    int underscore3 = cmd.indexOf('_', underscore2 + 1);
    int underscore4 = cmd.indexOf('_', underscore3 + 1);
    
    if (underscore1 > 0 && underscore2 > 0 && underscore3 > 0) {
      String cup_name = cmd.substring(0, underscore1);
      String liquid_type = cmd.substring(underscore1 + 1, underscore2);
      String motor_num = cmd.substring(underscore2 + 1, underscore3);
      
      String target_str, specific_liquid;
      if (underscore4 > 0) {
        // Format: H7_sauce_1_caramel_10
        specific_liquid = cmd.substring(underscore3 + 1, underscore4);
        target_str = cmd.substring(underscore4 + 1);
      } else {
        // Format: H7_sauce_1_10  
        target_str = cmd.substring(underscore3 + 1);
        specific_liquid = liquid_type; // Use liquid_type as default
      }
      
      float target_weight = target_str.toFloat();
      
      // Build motor name
      String motor_name = liquid_type + motor_num;
      motor_name.toLowerCase();
      specific_liquid.toLowerCase();
      
      Serial.print(F("Dispensing: ")); Serial.print(cup_name);
      Serial.print(F(" + ")); Serial.print(motor_name);
      Serial.print(F(" (")); Serial.print(specific_liquid);
      Serial.print(F(") -> ")); Serial.print(target_weight, 1); Serial.println(F("g"));
      
      startDispenseJob(cup_name.c_str(), motor_name.c_str(), target_weight, specific_liquid.c_str());
      return;
    }
  }
  
  cmd.toUpperCase();
  
  if (cmd.startsWith("SPEED ")) {
    int space1 = cmd.indexOf(' ');
    if (space1 > 0) {
      bool speed_on = cmd.substring(space1 + 1).toInt() != 0;
      global_speed_enabled = speed_on;
      setGlobalSpeed(speed_on);
      Serial.print(F("Global speed: ")); Serial.println(speed_on ? "ON" : "OFF");
    } else {
      Serial.println(F("Usage: SPEED <0|1>"));
    }
  }
  else if (cmd == "STOP") {
    emergencyStop();
    Serial.println(F("EMERGENCY STOP - Job cancelled"));
  }
  else if (cmd == "RESET_LEAK") {
    leak_emergency_triggered = false;
    for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
      LEAK_SENSORS[i].leak_detected = false;
    }
    Serial.println(F("🔄 Leak emergency reset - System ready"));
  }
  else if (cmd == "STATUS") {
    Serial.print(F("Job state: "));
    switch (current_job.state) {
      case IDLE: Serial.println(F("IDLE")); break;
      case DISPENSING: Serial.println(F("DISPENSING")); break;
      case COMPLETED: Serial.println(F("COMPLETED")); break;
    }
    
    // Show leak detection status
    Serial.print(F("🛡️ Leak Emergency: "));
    Serial.println(leak_emergency_triggered ? F("TRIGGERED") : F("OK"));
    for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
      Serial.print(F("  ")); Serial.print(LEAK_SENSORS[i].label);
      Serial.print(F(" (A")); Serial.print(LEAK_SENSORS[i].sensor_pin - A0);
      Serial.print(F("): ")); 
      int value = analogRead(LEAK_SENSORS[i].sensor_pin);
      Serial.print(value);
      Serial.println(LEAK_SENSORS[i].leak_detected ? F(" LEAK!") : F(" OK"));
    }
    
    if (current_job.state != IDLE) {
      Serial.print(F("Motor pin: ")); Serial.print(current_job.motor_pin);
      Serial.print(F(", Target: ")); Serial.print(current_job.target_weight, 1);
      Serial.print(F("g, Current: "));
      float current_weight = (current_job.scale_addr == SLV_A) ? regW_A / 10.0 : regW_B / 10.0;
      float net_weight = current_weight - current_job.tare_offset;
      Serial.print(net_weight, 1); Serial.println(F("g"));
    }
  }
  else if (cmd.startsWith("LAGM ")) {
    // LAGM <motor_name> <speed> <lag>
    int s1 = cmd.indexOf(' ');
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    if (s1 > 0 && s2 > 0 && s3 > 0) {
      String motor = cmd.substring(s1 + 1, s2);
      int speed = cmd.substring(s2 + 1, s3).toInt();
      float lag = cmd.substring(s3 + 1).toFloat();
      motor.toLowerCase();
      int idx = getMotorIndexByName(motor.c_str());
      if (idx >= 0) {
        if (speed == 0) motorLagOverrideSpeed0[idx] = lag; else motorLagOverrideSpeed1[idx] = lag;
        Serial.print(F("Per-motor lag set: ")); Serial.print(motor);
        Serial.print(F(" speed ")); Serial.print(speed);
        Serial.print(F(" -> ")); Serial.println(lag, 1);
      } else {
        Serial.print(F("Unknown motor: ")); Serial.println(motor);
      }
    } else {
      Serial.println(F("Usage: LAGM <motor> <0|1> <lag_g>"));
    }
  }
  else if (cmd.startsWith("LAG ")) {
    int space1 = cmd.indexOf(' ');
    int space2 = cmd.indexOf(' ', space1 + 1);
    
    if (space1 > 0 && space2 > 0) {
      // LAG <speed> <lag_value>
      int speed = cmd.substring(space1 + 1, space2).toInt();
      float lag = cmd.substring(space2 + 1).toFloat();
      
      if (speed == 0) {
        custom_lag_speed0 = lag;
        Serial.print(F("Speed 0 lag set to: ")); Serial.println(lag, 1);
      } else if (speed == 1) {
        custom_lag_speed1 = lag;
        Serial.print(F("Speed 1 lag set to: ")); Serial.println(lag, 1);
      } else {
        Serial.println(F("Speed must be 0 or 1"));
      }
    } else {
      Serial.println(F("Usage: LAG <speed> <lag> (e.g. LAG 1 5.5)"));
    }
  }
  else if (cmd == "HELP") {
    Serial.println(F("Automated Dispensing Commands:"));
    Serial.println(F("  <cup>_<liquid>_<motor>_<weight> - Start dispensing"));
    Serial.println(F("  SPEED <0|1>                     - Global speed control"));
    Serial.println(F("  LAG <1.0-10.0>                  - Motor stop lag compensation"));
    Serial.println(F("  STOP                            - Emergency stop"));
    Serial.println(F("  RESET_LEAK                      - Reset leak emergency"));
    Serial.println(F("  STATUS                          - Show job status"));
    Serial.println(F("  HELP                            - Show this help"));
    Serial.println(F("Examples:"));
    Serial.println(F("  H7_milk_1_100        - H7 cup + milk1 -> 100g"));
    Serial.println(F("  C9_sauce_10_8        - C9 cup + sauce10 -> 8g"));
    Serial.println(F("  H7_sauce_1_caramel_10 - H7 cup + sauce1 (caramel) -> 10g"));
    Serial.println(F("  SPEED 1              - Enable global speed"));
    Serial.println(F("  LAG 1 5.0            - Set speed 1 lag to 5g"));
    Serial.println(F("🛡️ Leak Detection: A5,A8 (sensors), A6,A9 (power HIGH)"));
    Serial.println(F("Cup types: H7(5.3g), H9(13.1g), H12(17.0g), C7(6.7g), C9(10.3g), C12(11.0g), C12B(14.2g)"));
    Serial.println(F("Liquids: water, milk, sauce, caramel, syrup, honey"));
  }
  else if (cmd.length() > 0) {
    Serial.println(F("Unknown command. Type HELP for available commands."));
  }
}

// Process commands from MQTT/CAN bridge via UART2
void processBridgeCommands() {
  if (!BRIDGE_SERIAL.available()) return;
  
  String cmd = BRIDGE_SERIAL.readStringUntil('\n');
  cmd.trim();
  
  Serial.print("BRIDGE RX: "); Serial.println(cmd); // DEBUG

  // Frother commands from Micro → forward to Nano Every on Serial1
  if (cmd.startsWith("FR_CMD:")) {
    Serial1.println(cmd); // pass-through to Nano
    return;
  }
  
  // Handle CAN commands (format: "CAN:motor_id,weight_dg,liquid_type")
  if (cmd.startsWith("CAN:")) {
    String params = cmd.substring(4); // Remove "CAN:"
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > 0) {
      uint8_t motor_id = params.substring(0, comma1).toInt();
      uint16_t weight_dg = params.substring(comma1 + 1, comma2).toInt();
      uint8_t liquid_type = params.substring(comma2 + 1).toInt();
      
      float target_weight = weight_dg / 10.0;
      const char* motor_name = getMotorNameById(motor_id);
      const char* liquid_name = getLiquidNameById(liquid_type);
      
      Serial.print("CAN Command: Motor="); Serial.print(motor_id);
      Serial.print(" Weight="); Serial.print(target_weight, 1);
      Serial.print("g Type="); Serial.println(liquid_type);
      
      // Special handling: rinser (motor_id=24) → 5s pulse on D49, ignore scales
      if (motor_id == 24) {
        digitalWrite(49, HIGH);
        rinser_active = true;
        rinser_end_ms = millis() + 5000UL;
        return;
      }

      if (motor_name && liquid_name) {
        startDispenseJob("CAN", motor_name, target_weight, liquid_name);
        // Send ACK back to Micro via UART
        BRIDGE_SERIAL.println("ACK:SUCCESS");
      } else {
        Serial.println("Invalid motor ID or liquid type");
        BRIDGE_SERIAL.println("ACK:ERROR");
      }
      return;
    }
  }
  
  // Handle MQTT commands (e.g., "caramel_10") - existing logic
  if (cmd.length() > 0) {
    // Process the command directly (same as USB serial commands)
    if (cmd.indexOf('_') > 0) {
      // Parse liquid command: caramel_10
      int underscore = cmd.indexOf('_');
      String liquid_type = cmd.substring(0, underscore);
      float target_weight = cmd.substring(underscore + 1).toFloat();
      
      Serial.print("MQTT Command: "); Serial.print(liquid_type);
      Serial.print(" -> "); Serial.print(target_weight, 1); Serial.println("g");
      
      // Auto-select motor (sauce1 for most liquids)
      String motor_name = "sauce1";
      if (liquid_type.indexOf("milk") >= 0) {
        motor_name = "milk1";
      }
      
      startDispenseJob("AUTO", motor_name.c_str(), target_weight, liquid_type.c_str());
    }
  }
}

void loop() {
  // 🛡️ PRIORITY 1: Check for leaks during active dispensing (safety critical)
  if (current_job.state == DISPENSING && checkLeakSensors()) {
    handleLeakEmergency();
  }
  
  // Handle local serial commands (USB)
  processSerialCommands();
  
  // Handle commands from MQTT bridge (UART2)
  processBridgeCommands();

  // Pump frother events from Nano → Micro
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      nanoBuf[nanoIdx] = 0;
      String line = String(nanoBuf);
      // Update frother_active based on events
      if (line.startsWith("FR_EVT:STATE,")) {
        int p1 = line.indexOf(',');
        int p2 = line.indexOf(',', p1 + 1);
        int p3 = line.indexOf(',', p2 + 1);
        // mode = substring(p1+1,p2) not needed here
        uint8_t busy = (uint8_t)line.substring(p2 + 1, p3).toInt();
        frother_active = busy ? true : false;
      } else if (line.startsWith("FR_EVT:DONE,") || line.startsWith("FR_EVT:ERROR,")) {
        frother_active = false;
      }
      // Forward raw line to Micro over BRIDGE
      BRIDGE_SERIAL.println(nanoBuf);
      // Also echo to Mega USB Serial for direct monitoring
      Serial.println(nanoBuf);
      nanoIdx = 0;
    } else if (nanoIdx < (int)sizeof(nanoBuf) - 1) {
      nanoBuf[nanoIdx++] = c;
    }
  }
  
  // Process active dispensing job (only if no leak emergency)
  if (!leak_emergency_triggered && !frother_active) {
    processDispenseJob();
  }

  // Rinser pulse completion handling and upstream notification
  if (rinser_active && (int32_t)(millis() - rinser_end_ms) >= 0) {
    digitalWrite(49, LOW);
    rinser_active = false;
    // Notify Micro (for CAN deferred ACK): include substring State=COMPLETED
    BRIDGE_SERIAL.println("State=COMPLETED RINSER");
  }
  
  // Auto-recovery: Reset leak emergency if no current leaks detected for 5 seconds
  static uint32_t leak_clear_time = 0;
  if (leak_emergency_triggered) {
    bool any_leak_active = false;
    for (uint8_t i = 0; i < NUM_LEAK_SENSORS; i++) {
      if (LEAK_SENSORS[i].leak_detected) {
        any_leak_active = true;
        break;
      }
    }
    
    if (!any_leak_active) {
      if (leak_clear_time == 0) {
        leak_clear_time = millis();
      } else if (millis() - leak_clear_time > 5000) {  // 5 seconds clear
        leak_emergency_triggered = false;
        leak_clear_time = 0;
        Serial.println(F("🔄 Auto-recovery: Leak emergency cleared"));
      }
    } else {
      leak_clear_time = 0;  // Reset timer if leak still detected
    }
  }
  
  // 1) Periodically poll both I²C slaves to refresh the 4 registers
  if ((uint32_t)(millis() - tPoll) >= POLL_MS) {
    tPoll = millis();

    int16_t dg; uint16_t st;
    if (readWeight(SLV_A, dg, st)) { noInterrupts(); regW_A = dg; regS_A = st; interrupts(); }
    if (readWeight(SLV_B, dg, st)) { noInterrupts(); regW_B = dg; regS_B = st; interrupts(); }

    // Send data to MQTT bridge via UART2
    if (current_job.state == IDLE) {
      // Standard format for MQTT bridge
      BRIDGE_SERIAL.print("A="); BRIDGE_SERIAL.print(regW_A / 10.0, 1);
      BRIDGE_SERIAL.print(" g B="); BRIDGE_SERIAL.print(regW_B / 10.0, 1);
      BRIDGE_SERIAL.print(" g sA="); BRIDGE_SERIAL.print(regS_A);
      BRIDGE_SERIAL.print(" sB="); BRIDGE_SERIAL.println(regS_B);
      
      // Local debug (reduced)
      Serial.print("Scales A="); Serial.print(regW_A / 10.0, 1); 
      Serial.print("g B="); Serial.print(regW_B / 10.0, 1); Serial.println("g");
    } else {
      // Dispensing format for MQTT bridge
      BRIDGE_SERIAL.print("Milk="); BRIDGE_SERIAL.print(regW_A / 10.0, 1); BRIDGE_SERIAL.print("g ");
      BRIDGE_SERIAL.print("Sauce="); BRIDGE_SERIAL.print(regW_B / 10.0, 1); BRIDGE_SERIAL.print("g ");
      BRIDGE_SERIAL.print("State=");
      switch (current_job.state) {
        case DISPENSING: BRIDGE_SERIAL.print("DISPENSING"); break;
        case COMPLETED: BRIDGE_SERIAL.print("COMPLETED"); break;
      }
      if (current_job.motor_running) BRIDGE_SERIAL.print(" MOTOR_ON");
      BRIDGE_SERIAL.println();
      
      // Local debug
      Serial.print("Dispensing: Milk="); Serial.print(regW_A / 10.0, 1);
      Serial.print("g Sauce="); Serial.print(regW_B / 10.0, 1);
      Serial.print("g State="); 
      switch (current_job.state) {
        case DISPENSING: Serial.print("DISPENSING"); break;
        case COMPLETED: Serial.print("COMPLETED"); break;
      }
      if (current_job.motor_running) Serial.print(" MOTOR_ON");
      Serial.println();
    }
  }

  // No more Modbus - using MQTT bridge via UART2
}
