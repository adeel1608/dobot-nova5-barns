// ===== ESP32-S3 + MCP2515 Tamper Node (ID 0x103) =====
// Library: "mcp2515" by autowp (Arduino Library Manager)

#include <SPI.h>
#include <mcp2515.h>

// ---- MCP2515 SPI pins ----
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_CS     5
// #define PIN_INT  2   // optional; we poll

// ---- Tamper pins ----
#define IN1 32        // motor dir A
#define IN2 33        // motor dir B
#define IND 2         // indicator LED (GPIO2)

// ---- CAN / MCP settings ----
#define BUS_SPEED   CAN_500KBPS
#define DEVICE_ID   0x103   // Tamper node

// ---- Heartbeat (every 30 s) -> payload: FF FF 00 00 00 00 00 00 ----
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000UL;
static uint32_t       hb_next_ms = 0;

// MCP2515 debug regs
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// --- Helpers ---
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

static void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
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
  // remaining bytes are zero
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) Serial.println("Heartbeat sent");
  else                        Serial.printf("Heartbeat send error=%d\n", (int)e);
}

static void tamperingRoutine() {
  // Extend
  digitalWrite(IND, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(1350);

  // Stop
  motorStop();
  delay(500);

  // Retract
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(2000);

  // Stop
  motorStop();
  digitalWrite(IND, HIGH);
  delay(500);
  digitalWrite(IND, LOW);
}

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

  // Accept only our ID
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
  Serial.printf("NormalMode: CANSTAT=0x%02X (OPMOD=%u; 0=Normal)\n", cs, opmod);
  return (opmod == 0);
}

// --- CAN handler ---
static void handleFrame(const struct can_frame& rx) {
  if (rx.can_id & CAN_EFF_FLAG) return;                 // std only
  uint16_t id = rx.can_id & CAN_SFF_MASK;
  if (id != DEVICE_ID) return;

  Serial.printf("RX ID=0x%03X DLC=%d Data:", id, rx.can_dlc);
  for (int i=0; i<rx.can_dlc; ++i) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  // b0: indicator blink if 0x01
  if (rx.can_dlc >= 1 && rx.data[0] == 0x01) {
    digitalWrite(IND, HIGH);
    delay(50);
    digitalWrite(IND, LOW);
  }

  // b1: action
  if (rx.can_dlc >= 2) {
    uint8_t act = rx.data[1];

    if (act == 0x01) {
      // run routine (blocking)
      Serial.println("Tampering routine START");
      tamperingRoutine();
      Serial.println("Tampering routine END");
      sendDoneAck();               // ACK after routine completes
    } else if (act == 0x00) {
      // stop immediately (best-effort; routine is blocking)
      motorStop();
      Serial.println("Tampering STOP");
      sendDoneAck();               // immediate ACK on stop
    } else {
      Serial.printf("Unknown tamper code 0x%02X\n", act);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Tamper Node (MCP2515) ===");

  pinMode(IND, OUTPUT);
  digitalWrite(IND, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorStop();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  bool ok = false;
  for (int i=0; i<4 && !ok; ++i) {
    ok = initMCP();
    if (!ok) { Serial.println("Retrying MCP init..."); delay(120); }
  }
  if (!ok) {
    Serial.println("FAILED: MCP2515 did not enter Normal mode.");
    while (1) delay(500);
  }

  Serial.printf("Ready. DEVICE_ID=0x%03X @500k\n", DEVICE_ID);
  Serial.println("Frame: b0=01 blink, b1=01 run routine, b1=00 stop.");

  // Heartbeat: send immediately, then every 30 s
  sendHeartbeat();
  hb_next_ms = millis() + HEARTBEAT_INTERVAL_MS;
}

void loop() {
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleFrame(rx);
  }

  // Heartbeat scheduler (note: routine is blocking, so heartbeat may be delayed during motion)
  uint32_t now = millis();
  if ((int32_t)(now - hb_next_ms) >= 0) {
    sendHeartbeat();
    hb_next_ms = now + HEARTBEAT_INTERVAL_MS;
  }

  delay(2);
}
