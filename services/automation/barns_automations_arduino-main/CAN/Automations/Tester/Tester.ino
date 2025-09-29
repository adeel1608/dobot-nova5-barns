// ===== ESP32-S3 + MCP2515 Grinder Node (LED-only, ID 0x106) =====
// Library: "mcp2515" by autowp (Arduino Library Manager)
#include <SPI.h>
#include <mcp2515.h>

// ---- MCP2515 SPI pins on your ESP32-S3 ----
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_CS     5
// #define PIN_INT  2   // optional (polling is fine)

// ---- LED output (we use the board LED as the actuator) ----
#define INDICATOR_LED 2   // built-in on many ESP32-S3 boards

// ---- CAN / MCP settings ----
#define BUS_SPEED        CAN_500KBPS
#define DEVICE_ID        0x103        // per your script
#define DEFAULT_PULSE_MS 0            // not used, we default to 1s if no duration provided
static const uint16_t BLINK_MS = 50;  // short blink on b0=0x01

// Heartbeat every 30s: payload FF FF 00 00 00 00 00 00
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000UL;
static uint32_t       hb_next_ms = 0;

// MCP2515 debug regs
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// ---- Process state ----
static bool     process_active = false;
static uint32_t process_off_at_ms = 0;

static uint8_t readReg(uint8_t addr) {
  const uint8_t CMD_READ = 0x03;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(addr);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return val;
}

static inline uint16_t u16_le(uint8_t lo, uint8_t hi) { return (uint16_t)lo | ((uint16_t)hi << 8); }

// ---- ACK: send "done" with same ID, data[0] = 0x01 ----
static void sendDoneAck() {
  struct can_frame tx = {};
  tx.can_id  = DEVICE_ID;   // same device ID
  tx.can_dlc = 1;
  tx.data[0] = 0x01;        // "done"
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) {
    Serial.println("ACK sent (0x01)");
  } else {
    Serial.printf("ACK send error=%d\n", (int)e);
  }
}

// ---- Heartbeat: ID=DEVICE_ID, data = FF FF 00 00 00 00 00 00 ----
static void sendHeartbeat() {
  struct can_frame tx = {};
  tx.can_id  = DEVICE_ID;
  tx.can_dlc = 8;
  tx.data[0] = 0xFF;
  tx.data[1] = 0xFF;
  // rest are zero by default-initialization
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) {
    Serial.println("Heartbeat sent");
  } else {
    Serial.printf("Heartbeat send error=%d\n", (int)e);
  }
}

static void stopProcess() {
  digitalWrite(INDICATOR_LED, LOW);
  process_active = false;
  process_off_at_ms = 0;
}

static void startProcessSeconds(uint32_t dur_s) {
  if (process_active) {
    Serial.println("Process interrupted by new command");
    stopProcess();
  }
  if (dur_s == 0) dur_s = 1; // deterministic default if FF/missing/zero
  digitalWrite(INDICATOR_LED, HIGH);
  process_active = true;
  process_off_at_ms = millis() + dur_s * 1000UL;
  Serial.printf("Process START, LED=ON for %lus\n", (unsigned long)dur_s);
}

static bool tryBitrateAuto() {
  MCP2515::ERROR e = mcp.setBitrate(BUS_SPEED, MCP_8MHZ);
  if (e == MCP2515::ERROR_OK) { Serial.println("MCP setBitrate OK @8MHz"); return true; }
  Serial.printf("setBitrate err=%d @8MHz, trying 16MHz...\n", (int)e);
  e = mcp.setBitrate(BUS_SPEED, MCP_16MHZ);
  if (e == MCP2515::ERROR_OK) { Serial.println("MCP setBitrate OK @16MHz"); return true; }
  Serial.printf("setBitrate err=%d @16MHz\n", (int)e);
  return false;
}

static bool initMCP() {
  mcp.reset();
  delay(20);

  uint8_t cs = readReg(REG_CANSTAT);
  uint8_t cc = readReg(REG_CANCTRL);
  Serial.printf("After reset: CANSTAT=0x%02X CANCTRL=0x%02X\n", cs, cc);

  if (!tryBitrateAuto()) return false;

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
  Serial.printf("NormalMode: CANSTAT=0x%02X (OPMOD=%u; 0=Normal)\n", cs, opmod);
  return (opmod == 0);
}

// ---- CAN handler ----
// Expect: ID=0x106, Data: [b0=0x01, b1=0x01, b2=seconds, (optional b3 seconds_hi)]
// Stop:   ID=0x106, Data: [b0=0x01, b1=0x00]
static void handleFrame(const struct can_frame& rx) {
  if (rx.can_id & CAN_EFF_FLAG) return;          // only standard
  uint16_t id = rx.can_id & CAN_SFF_MASK;
  if (id != DEVICE_ID) return;

  Serial.printf("RX ID=0x%03X DLC=%d Data:", id, rx.can_dlc);
  for (int i = 0; i < rx.can_dlc; i++) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  // b0: indicator blink (non-blocking-ish)
  if (rx.can_dlc >= 1 && rx.data[0] == 0x01) {
    digitalWrite(INDICATOR_LED, HIGH);
    delay(BLINK_MS);
    digitalWrite(INDICATOR_LED, LOW);
  }

  if (rx.can_dlc >= 2) {
    uint8_t cmd = rx.data[1];  // 0x01 = run, 0x00 = stop
    if (cmd == 0x00) {
      stopProcess();
      Serial.println("STOP command: LED OFF");
      sendDoneAck();           // immediate ACK on stop
      return;
    }
    if (cmd == 0x01) {
      // duration parsing: b2 (and optional b3 LE) => seconds
      uint32_t dur_s = 0;
      if (rx.can_dlc >= 4)      dur_s = u16_le(rx.data[2], rx.data[3]);
      else if (rx.can_dlc >= 3) dur_s = rx.data[2];
      startProcessSeconds(dur_s);
      return;
    }
    Serial.printf("Unknown cmd 0x%02X (ignored)\n", cmd);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Grinder LED Node (MCP2515) ===");

  pinMode(INDICATOR_LED, OUTPUT);
  digitalWrite(INDICATOR_LED, LOW);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  bool ok = false;
  for (int i = 0; i < 4 && !ok; ++i) {
    ok = initMCP();
    if (!ok) { Serial.println("Retrying MCP init..."); delay(120); }
  }
  if (!ok) {
    Serial.println("FAILED: MCP2515 not entering Normal mode. Check clock/wiring.");
    while (1) delay(500);
  }

  Serial.printf("Ready. DEVICE_ID=0x%03X  bitrate=500k\n", DEVICE_ID);
  Serial.println("Frame: ID 0x106  [b0=01, b1=01, b2=<sec>, (b3=<sec_hi> opt)]  |  Stop: [b0=01, b1=00]");

  // Send first heartbeat immediately, then every 30s
  sendHeartbeat();
  hb_next_ms = millis() + HEARTBEAT_INTERVAL_MS;
}

void loop() {
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleFrame(rx);
  }

  uint32_t now = millis();

  // process timer
  if (process_active && (int32_t)(now - process_off_at_ms) >= 0) {
    stopProcess();
    Serial.println("Process DONE, LED OFF");
    sendDoneAck();  // completion response: same ID, data[0]=0x01
  }

  // heartbeat timer
  if ((int32_t)(now - hb_next_ms) >= 0) {
    sendHeartbeat();
    hb_next_ms = now + HEARTBEAT_INTERVAL_MS;
  }

  delay(2);
}
