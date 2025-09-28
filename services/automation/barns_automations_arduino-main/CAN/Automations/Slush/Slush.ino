// ===== ESP32-S3 + MCP2515 Slush Node (ID 0x102) =====
// Library: "mcp2515" by autowp (Arduino Library Manager)

#include <SPI.h>
#include <mcp2515.h>

// ---- MCP2515 SPI pins ----
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_CS     5
// #define PIN_INT  2   // optional; we poll

// ---- H-Bridge Inputs (two motors) ----
// Motor1: IN1, IN2  |  Motor2: IN3, IN4
#define IN1 33
#define IN2 25
#define IN3 32
#define IN4 15

// Indicator LED (GPIO2)
#define INDICATOR_LED 2

// ---- CAN / MCP settings ----
#define BUS_SPEED         CAN_500KBPS
#define DEVICE_ID         0x102     // Slush node
#define DEFAULT_PULSE_MS  0
static const uint32_t RETURN_MS = 20000UL; // fixed backward/return = 20 s

// Heartbeat (every 30 s) -> payload: FF FF 00 00 00 00 00 00
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000UL;
static uint32_t       hb_next_ms = 0;

// CMD0 (timed) ACK control
static bool cmd0_ack_pending = false;

// MCP2515 debug regs
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// ---- Motor control ----
enum Dir : uint8_t { STOP = 0, FWD = 1, BWD = 2 };

struct Motor {
  uint8_t in1, in2;
  Dir     dir;            // current direction
  // Two-stage "forward then backward" sequence (for cmds 1 & 2)
  uint8_t  stage;         // 0=idle, 1=forward phase, 2=backward phase
  uint32_t stage_end_ms;  // when current stage ends
  uint32_t stage_dur_ms;  // forward duration (from CAN)
  // Simple timed stop (used by command 0)
  uint32_t stop_at_ms;    // 0 if not scheduled
};

Motor M1{IN1, IN2, STOP, 0, 0, 0, 0};
Motor M2{IN3, IN4, STOP, 0, 0, 0, 0};

static inline void motorApplyPins(const Motor& m, Dir d) {
  if (d == FWD) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (d == BWD) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else { // STOP (coast). Use HIGH/HIGH for active brake if needed.
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }
}

static inline void motorSet(Motor& m, Dir d) {
  m.dir = d;
  motorApplyPins(m, d);
}

static inline void motorStop(Motor& m) {
  m.stage = 0;
  m.stage_end_ms = 0;
  m.stage_dur_ms = 0;
  m.stop_at_ms = 0;
  motorSet(m, STOP);
}

// ---- Repositioning (startup) ----
static bool     repositioning = false;
static uint32_t reposition_until_ms = 0;

// ---- SPI helper for debug reads ----
static uint8_t readReg(uint8_t addr) {
  const uint8_t CMD_READ = 0x03;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(addr);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

// ---- ACK: send "done" with same ID, data[0] = 0x01 ----
static void sendDoneAck() {
  struct can_frame tx = {};
  tx.can_id  = DEVICE_ID;
  tx.can_dlc = 1;
  tx.data[0] = 0x01;
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) Serial.println("ACK sent (0x01)");
  else                        Serial.printf("ACK send error=%d\n", (int)e);
}

// ---- Heartbeat: ID=DEVICE_ID, data = FF FF 00 00 00 00 00 00 ----
static void sendHeartbeat() {
  struct can_frame tx = {};
  tx.can_id  = DEVICE_ID;
  tx.can_dlc = 8;
  tx.data[0] = 0xFF;
  tx.data[1] = 0xFF;
  // others remain zero (struct zero-initialized)
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) Serial.println("Heartbeat sent");
  else                        Serial.printf("Heartbeat send error=%d\n", (int)e);
}

// ---- MCP setup ----
static bool setBitrateAuto() {
  MCP2515::ERROR e = mcp.setBitrate(BUS_SPEED, MCP_8MHZ);
  if (e == MCP2515::ERROR_OK) { Serial.println("MCP bitrate OK @8MHz"); return true; }
  Serial.printf("setBitrate err=%d @8MHz, trying 16MHz...\n", (int)e);
  e = mcp.setBitrate(BUS_SPEED, MCP_16MHZ);
  if (e == MCP2515::ERROR_OK) { Serial.println("MCP bitrate OK @16MHz"); return true; }
  Serial.printf("setBitrate err=%d @16MHz\n", (int)e);
  return false;
}

static bool initMCP() {
  mcp.reset();
  delay(20);
  uint8_t cs = readReg(REG_CANSTAT), cc = readReg(REG_CANCTRL);
  Serial.printf("After reset: CANSTAT=0x%02X CANCTRL=0x%02X\n", cs, cc);

  if (!setBitrateAuto()) return false;

  // Accept only our ID (standard frames)
  mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp.setFilter(MCP2515::RXF0, false, DEVICE_ID);
  mcp.setFilter(MCP2515::RXF1, false, DEVICE_ID);
  mcp.setFilter(MCP2515::RXF2, false, DEVICE_ID);
  mcp.setFilter(MCP2515::RXF3, false, DEVICE_ID);
  mcp.setFilter(MCP2515::RXF4, false, DEVICE_ID);
  mcp.setFilter(MCP2515::RXF5, false, DEVICE_ID);

  mcp.setNormalMode();
  delay(5);
  cs = readReg(REG_CANSTAT);
  uint8_t opmod = (cs >> 5) & 0x07;
  Serial.printf("NormalMode: CANSTAT=0x%02X (OPMOD=%u)\n", cs, opmod);
  return (opmod == 0);
}

static inline uint16_t u16_le(uint8_t lo, uint8_t hi) { return (uint16_t)lo | ((uint16_t)hi << 8); }

// ---- CAN handler ----
// b0=01 blink
// b1 command (0..4): 0=both BACKWARD; 1=M1 FWD then fixed 20s BACKWARD; 2=M2 same; 3,4 unused
// b2..b3 duration_s (LE, optional) — used only for the forward phase of cmds 1/2; cmd 0 uses it as overall time if present
static void handleFrame(const struct can_frame& rx) {
  if (rx.can_id & CAN_EFF_FLAG) return;            // std only
  uint16_t id = rx.can_id & CAN_SFF_MASK;
  if (id != DEVICE_ID) return;

  Serial.printf("RX ID=0x%03X DLC=%d Data:", id, rx.can_dlc);
  for (int i = 0; i < rx.can_dlc; i++) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  // b0: indicator
  if (rx.can_dlc >= 1 && rx.data[0] == 0x01) {
    digitalWrite(INDICATOR_LED, HIGH);
    delay(50);
    digitalWrite(INDICATOR_LED, LOW);
  }

  if (repositioning) {
    Serial.println("Ignoring command: repositioning in progress");
    return;
  }

  if (rx.can_dlc >= 2) {
    uint8_t cmd = rx.data[1];
    uint32_t dur_s = 0;
    if (rx.can_dlc >= 4) dur_s = u16_le(rx.data[2], rx.data[3]);
    else if (rx.can_dlc >= 3) dur_s = rx.data[2];

    // forward phase default if not provided (keeps behavior deterministic)
    uint32_t dur_ms = (dur_s > 0) ? dur_s * 1000UL : (DEFAULT_PULSE_MS > 0 ? DEFAULT_PULSE_MS : 1000UL);

    switch (cmd) {
      case 0x00: {
        // Both motors backward; timed if dur provided (>0) else latched
        M1.stage = M2.stage = 0; // cancel staged sequences
        M1.stage_end_ms = M2.stage_end_ms = 0;
        motorSet(M1, BWD);
        motorSet(M2, BWD);
        if (dur_s > 0 || DEFAULT_PULSE_MS > 0) {
          uint32_t until = millis() + dur_ms;
          M1.stop_at_ms = until;
          M2.stop_at_ms = until;
          cmd0_ack_pending = true;  // schedule ACK when both finish
          Serial.printf("CMD0: Both BACKWARD for %lus\n", (unsigned long)dur_s);
        } else {
          M1.stop_at_ms = M2.stop_at_ms = 0;
          cmd0_ack_pending = false;
          Serial.println("CMD0: Both BACKWARD (latched)");
        }
      } break;

      case 0x01: {
        // Motor1: forward for T, then backward for fixed 20s, then stop
        motorStop(M1);
        motorSet(M1, FWD);
        M1.stage = 1;
        M1.stage_dur_ms = dur_ms;               // forward phase uses CAN duration
        M1.stage_end_ms = millis() + dur_ms;
        Serial.printf("CMD1: M1 FORWARD for %lums then BACKWARD for %lus\n",
                      (unsigned long)dur_ms, (unsigned long)(RETURN_MS/1000));
      } break;

      case 0x02: {
        // Motor2: forward for T, then backward for fixed 20s, then stop
        motorStop(M2);
        motorSet(M2, FWD);
        M2.stage = 1;
        M2.stage_dur_ms = dur_ms;
        M2.stage_end_ms = millis() + dur_ms;
        Serial.printf("CMD2: M2 FORWARD for %lums then BACKWARD for %lus\n",
                      (unsigned long)dur_ms, (unsigned long)(RETURN_MS/1000));
      } break;

      case 0x03:
      case 0x04:
        Serial.printf("CMD%u: unused (ignored)\n", (unsigned)cmd);
        break;

      default:
        Serial.printf("Unknown command 0x%02X\n", cmd);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Slush Node (MCP2515) — Dual Motor ===");

  pinMode(INDICATOR_LED, OUTPUT);
  digitalWrite(INDICATOR_LED, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  motorStop(M1);
  motorStop(M2);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  bool ok = false;
  for (int i = 0; i < 4 && !ok; i++) {
    ok = initMCP();
    if (!ok) { Serial.println("Retrying MCP init..."); delay(120); }
  }
  if (!ok) {
    Serial.println("FAILED: MCP2515 did not enter Normal mode.");
    while (1) delay(500);
  }

  Serial.printf("Ready. DEVICE_ID=0x%03X @500k\n", DEVICE_ID);
  Serial.println("Format: b0=01 blink, b1=cmd(0..4), b2..b3=duration_s (LE, optional)");

  // Startup repositioning: both motors backward for 10s
  Serial.println("repositioning");
  repositioning = true;
  motorSet(M1, BWD);
  motorSet(M2, BWD);
  reposition_until_ms = millis() + 10000UL;

  // Heartbeat now, then every 30 s
  sendHeartbeat();
  hb_next_ms = millis() + HEARTBEAT_INTERVAL_MS;
}

void loop() {
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleFrame(rx);
  }

  uint32_t now = millis();

  // Finish repositioning
  if (repositioning && (int32_t)(now - reposition_until_ms) >= 0) {
    motorStop(M1);
    motorStop(M2);
    repositioning = false;
    Serial.println("repositioning done");
  }

  // Timed stop for CMD0
  if (!repositioning) {
    bool m1_stopped_now = false, m2_stopped_now = false;

    if (M1.stop_at_ms && (int32_t)(now - M1.stop_at_ms) >= 0) {
      motorStop(M1);
      m1_stopped_now = true;
      Serial.printf("M1 STOP (timed)\n");
    }
    if (M2.stop_at_ms && (int32_t)(now - M2.stop_at_ms) >= 0) {
      motorStop(M2);
      m2_stopped_now = true;
      Serial.printf("M2 STOP (timed)\n");
    }

    // If this was a timed CMD0, ACK once after both motors have stopped
    if (cmd0_ack_pending &&
        M1.stop_at_ms == 0 && M2.stop_at_ms == 0 &&
        M1.dir == STOP && M2.dir == STOP &&
        (m1_stopped_now || m2_stopped_now)) {
      sendDoneAck();
      cmd0_ack_pending = false;
    }
  }

  // Stage machine for CMD1/CMD2 (forward then fixed 20s backward)
  auto runStages = [&](Motor& m, const char* name){
    if (repositioning) return;
    if (m.stage == 1 && (int32_t)(now - m.stage_end_ms) >= 0) {
      motorSet(m, BWD);
      m.stage = 2;
      m.stage_end_ms = now + RETURN_MS;  // ALWAYS 20 s on the way back
      Serial.printf("%s -> BACKWARD (return for %lus)\n", name, (unsigned long)(RETURN_MS/1000));
    }
    if (m.stage == 2 && (int32_t)(now - m.stage_end_ms) >= 0) {
      motorStop(m);
      Serial.printf("%s -> STOP (returned to pose)\n", name);
      sendDoneAck(); // ACK at completion of the full sequence
    }
  };
  runStages(M1, "M1");
  runStages(M2, "M2");

  // Heartbeat scheduler
  if ((int32_t)(now - hb_next_ms) >= 0) {
    sendHeartbeat();
    hb_next_ms = now + HEARTBEAT_INTERVAL_MS;
  }

  delay(2);
}
