// ===== ESP32-S3 + MCP2515 Grinder Node (ID 0x101) =====
// Library: "mcp2515" by autowp (Arduino Library Manager)
#include <SPI.h>
#include <mcp2515.h>

// ---- MCP2515 SPI pins on your ESP32-S3 ----
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_CS     5
// #define PIN_INT  2   // optional (polling is fine)

// ---- Grinder outputs ----
#define SINGLE_PIN1 33
#define DOUBLE_PIN2 25
#define TRIPLE_PIN3 32

// Indicator LED (built-in on many boards is GPIO2)
#define INDICATOR_LED 2

// ---- CAN / MCP settings ----
#define BUS_SPEED        CAN_500KBPS
#define DEVICE_ID        0x101   // <— change per machine (100 + N)
#define DEFAULT_PULSE_MS 0       // if duration=0: 0=latch ON; >0=auto-off after this ms

// Pulse timing for "button press" simulation
static const uint16_t PRESS_MS = 120;   // press width for start/stop pulses

// Heartbeat every 30s: payload FF FF 00 00 00 00 00 00
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000UL;
static uint32_t       hb_next_ms = 0;

// MCP2515 debug regs
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// Relay state/timer
struct Relay {
  uint8_t  pin;
  bool     active;        // true after "start" press
  uint32_t off_at_ms;     // when to issue the "stop" press; 0 if not timing
} relays[3] = {
  { SINGLE_PIN1, false, 0 },
  { DOUBLE_PIN2, false, 0 },
  { TRIPLE_PIN3, false, 0 },
};

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

// ---- Helper: simulate a momentary button press (active LOW) ----
static inline void pressPulse(uint8_t pin, uint16_t ms = PRESS_MS) {
  digitalWrite(pin, LOW);
  delay(ms);
  digitalWrite(pin, HIGH);
}

// ---- Helper: send CAN "done" ack (b0 = 0x01) ----
static void sendDoneAck() {
  struct can_frame tx = {};
  tx.can_id  = DEVICE_ID;
  tx.can_dlc = 1;
  tx.data[0] = 0x01;   // "done"
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
  // rest are zero by default
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) {
    Serial.println("Heartbeat sent");
  } else {
    Serial.printf("Heartbeat send error=%d\n", (int)e);
  }
}

static void relayOffAll() {
  for (auto &r : relays) {
    digitalWrite(r.pin, HIGH);
    r.active = false;
    r.off_at_ms = 0;
  }
}

static void relayOn(int idx, uint32_t dur_s) {
  // single-active policy
  relayOffAll();
  auto &r = relays[idx];

  // First "start" press
  pressPulse(r.pin);      // emulate button press to start
  r.active = true;

  if (dur_s > 0) {
    r.off_at_ms = millis() + dur_s * 1000UL;  // schedule "stop" press
  } else if (DEFAULT_PULSE_MS > 0) {
    r.off_at_ms = millis() + DEFAULT_PULSE_MS;
  } else {
    r.off_at_ms = 0;      // no timed stop — acknowledge completion now
    sendDoneAck();
  }
}

static bool tryBitrateAuto() {
  // Try 8 MHz first, then 16 MHz
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
  mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);   // exact match mask
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

static inline uint16_t u16_le(uint8_t lo, uint8_t hi) { return (uint16_t)lo | ((uint16_t)hi << 8); }

static void handleFrame(const struct can_frame& rx) {
  if (rx.can_id & CAN_EFF_FLAG) return;          // only standard
  uint16_t id = rx.can_id & CAN_SFF_MASK;
  if (id != DEVICE_ID) return;                   // filtered anyway

  // Log
  Serial.printf("RX ID=0x%03X DLC=%d Data:", id, rx.can_dlc);
  for (int i=0;i<rx.can_dlc;i++) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  // b0: indicator
  if (rx.can_dlc >= 1 && rx.data[0] == 0x01) {
    digitalWrite(INDICATOR_LED, HIGH);
    delay(50);
    digitalWrite(INDICATOR_LED, LOW);
  }

  // b1: grind selection (00/01/02/03), b2..b3: optional duration seconds (LE)
  if (rx.can_dlc >= 2) {
    uint8_t sel = rx.data[1];
    uint32_t dur_s = 0;
    if (rx.can_dlc >= 4) dur_s = u16_le(rx.data[2], rx.data[3]);

    switch (sel) {
      case 0x00: // stop all
        relayOffAll();
        Serial.println("All relays OFF");
        sendDoneAck();
        break;
      case 0x01: // single
        relayOn(0, dur_s);
        Serial.printf("SINGLE start (GPIO%d) dur=%lus\n", relays[0].pin, (unsigned long)dur_s);
        break;
      case 0x02: // double
        relayOn(1, dur_s);
        Serial.printf("DOUBLE start (GPIO%d) dur=%lus\n", relays[1].pin, (unsigned long)dur_s);
        break;
      case 0x03: // triple
        relayOn(2, dur_s);
        Serial.printf("TRIPLE start (GPIO%d) dur=%lus\n", relays[2].pin, (unsigned long)dur_s);
        break;
      default:
        Serial.printf("Unknown grind code 0x%02X\n", sel);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Grinder Node (MCP2515) ===");

  pinMode(INDICATOR_LED, OUTPUT);
  digitalWrite(INDICATOR_LED, LOW);

  pinMode(SINGLE_PIN1, OUTPUT);
  pinMode(DOUBLE_PIN2, OUTPUT);
  pinMode(TRIPLE_PIN3, OUTPUT);
  relayOffAll();

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  bool ok = false;
  for (int i=0; i<4 && !ok; ++i) {
    ok = initMCP();
    if (!ok) { Serial.println("Retrying MCP init..."); delay(120); }
  }
  if (!ok) {
    Serial.println("FAILED: MCP2515 not entering Normal mode. Check clock/wiring.");
    while (1) delay(500);
  }

  Serial.printf("Ready. DEVICE_ID=0x%03X  bitrate=500k\n", DEVICE_ID);
  Serial.println("Frame:  ID 0x101  b0=01 (blink)  b1=00/01/02/03  b2..b3=duration_s (LE)");

  // Heartbeat: send immediately, then every 30 s
  sendHeartbeat();
  hb_next_ms = millis() + HEARTBEAT_INTERVAL_MS;
}

void loop() {
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleFrame(rx);
  }

  // timers: when a timed run finishes, issue a stop press and ACK
  uint32_t now = millis();
  for (auto &r : relays) {
    if (r.active && r.off_at_ms != 0 && (int32_t)(now - r.off_at_ms) >= 0) {
      // "Stop" press (momentary)
      pressPulse(r.pin);
      r.active = false;
      r.off_at_ms = 0;
      Serial.printf("Relay GPIO%d -> STOP (timed)\n", r.pin);
      sendDoneAck();
    }
  }

  // heartbeat timer
  if ((int32_t)(now - hb_next_ms) >= 0) {
    sendHeartbeat();
    hb_next_ms = now + HEARTBEAT_INTERVAL_MS;
  }

  delay(2);
}
