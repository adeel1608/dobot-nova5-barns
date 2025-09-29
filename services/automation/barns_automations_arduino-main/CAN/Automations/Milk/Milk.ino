// ===== ESP32-S3 + MCP2515 Milk Dispenser (4 pumps per board) =====
// Flash this on both boards. Change DEVICE_ID to 0x104 or 0x105.
// Library: "mcp2515" by autowp (Arduino Library Manager)

#include <SPI.h>
#include <mcp2515.h>

// ---------- CAN node identity ----------
#define DEVICE_ID   0x104          // <-- set to 0x104 (board 1) or 0x105 (board 2)

// ---------- MCP2515 SPI pins (do NOT reuse for pumps) ----------
#define PIN_SCK   18
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_CS     5
// #define PIN_INT  2  // optional (polling is fine)

// ---------- Indicator ----------
#define IND_LED    2   // indicator blink when b0==0x01

// ---------- Pump pins (avoid 5,18,19,23) ----------
// Each pump: DIR, STEP, EN (SIG). You can tweak these as you like.
#define DIR1   4   //Yellow - DIR
#define STEP1  16  //Blue - PULL
#define EN1    17  //White - EN //Motor 6

#define DIR2   14
#define STEP2  12
#define EN2    13 //Motor 5

#define DIR3   25
#define STEP3  26
#define EN3    27 //Motor 8

#define DIR4   33
#define STEP4  32
#define EN4    15 //Motor 7

// ---------- CAN / MCP settings ----------
#define BUS_SPEED   CAN_500KBPS
#define MCP_TRY_8MHZ_FIRST  1

// ---------- Calibration ----------
static const float ML_PER_SEC = 6.75f;   // flow rate (mL/s) -> adjust per your calibration
static const bool  DIR_CLOCKWISE = true; // default flow direction (set per plumbing)

// ---------- Heartbeat ----------
static const uint32_t HEARTBEAT_INTERVAL_MS = 30000UL; // every 30s
static uint32_t       hb_next_ms = 0;

// ---------- Internals ----------
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// Pump struct
struct Pump {
  uint8_t dir, step, en;
} pumps[4] = {
  { DIR1, STEP1, EN1 },
  { DIR2, STEP2, EN2 },
  { DIR3, STEP3, EN3 },
  { DIR4, STEP4, EN4 },
};

// microstep pulse for a given STEP pin
inline void stepPulse(uint8_t stepPin) {
  digitalWrite(stepPin, HIGH);  delayMicroseconds(25);
  digitalWrite(stepPin, LOW);   delayMicroseconds(25);
  digitalWrite(stepPin, HIGH);  delayMicroseconds(12);
  digitalWrite(stepPin, LOW);   delayMicroseconds(12);
}

// run pump for duration_ms (blocking)
void runPumpMs(Pump& p, unsigned long duration_ms, bool forward=true) {
  digitalWrite(p.en, HIGH);                  // enable driver (active-HIGH)
  digitalWrite(p.dir, forward ? HIGH : LOW); // set direction
  unsigned long t0 = millis();
  while ((millis() - t0) < duration_ms) {
    stepPulse(p.step);
  }
  digitalWrite(p.en, LOW);                   // disable driver
}

// tiny SPI reg peek for logs
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

static bool setBitrateAuto() {
#if MCP_TRY_8MHZ_FIRST
  if (mcp.setBitrate(BUS_SPEED, MCP_8MHZ) == MCP2515::ERROR_OK) { Serial.println("MCP @8MHz OK"); return true; }
  Serial.println("8MHz failed, trying 16MHz...");
  if (mcp.setBitrate(BUS_SPEED, MCP_16MHZ) == MCP2515::ERROR_OK) { Serial.println("MCP @16MHz OK"); return true; }
#else
  if (mcp.setBitrate(BUS_SPEED, MCP_16MHZ) == MCP2515::ERROR_OK) { Serial.println("MCP @16MHz OK"); return true; }
  Serial.println("16MHz failed, trying 8MHz...");
  if (mcp.setBitrate(BUS_SPEED, MCP_8MHZ) == MCP2515::ERROR_OK) { Serial.println("MCP @8MHz OK"); return true; }
#endif
  return false;
}

static bool initMCP() {
  mcp.reset();
  delay(20);
  Serial.printf("CANSTAT=0x%02X CANCTRL=0x%02X\n", readReg(REG_CANSTAT), readReg(REG_CANCTRL));
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
  uint8_t cs = readReg(REG_CANSTAT);
  Serial.printf("NormalMode: CANSTAT=0x%02X (OPMOD=%u)\n", cs, (cs >> 5) & 0x07);
  return (((cs >> 5) & 0x07) == 0);
}

// big-endian 16-bit
static inline uint16_t u16_be(uint8_t hi, uint8_t lo) { return (uint16_t(hi) << 8) | uint16_t(lo); }

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
  // remaining bytes default to 0
  MCP2515::ERROR e = mcp.sendMessage(&tx);
  if (e == MCP2515::ERROR_OK) Serial.println("Heartbeat sent");
  else                        Serial.printf("Heartbeat send error=%d\n", (int)e);
}

static void handleFrame(const struct can_frame& rx) {
  if (rx.can_id & CAN_EFF_FLAG) return;               // std only
  const uint16_t id = rx.can_id & CAN_SFF_MASK;
  if (id != DEVICE_ID) return;

  Serial.printf("RX %03X DLC=%d Data:", id, rx.can_dlc);
  for (int i=0; i<rx.can_dlc; ++i) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  // b0: indicator blink
  if (rx.can_dlc >= 1 && rx.data[0] == 0x01) {
    digitalWrite(IND_LED, HIGH); delay(50); digitalWrite(IND_LED, LOW);
  }

  if (rx.can_dlc < 4) return;

  const uint8_t  pumpNo = rx.data[1];                     // 1..8
  const uint16_t ml     = u16_be(rx.data[2], rx.data[3]); // BIG-ENDIAN amount (mL)

  // Determine which four pumps this board owns:
  const uint8_t base = (DEVICE_ID == 0x104) ? 1 : 5;      // 1..4 or 5..8
  const uint8_t last = base + 3;

  if (pumpNo < base || pumpNo > last) {
    Serial.printf("Pump %u not on this board (this board owns %u..%u)\n", pumpNo, base, last);
    return;
  }

  const uint8_t localIndex = pumpNo - base; // 0..3
  Pump& P = pumps[localIndex];

  // Convert mL -> time  (ms) = (mL / ML_PER_SEC) * 1000
  unsigned long duration_ms = (unsigned long)((float)ml / ML_PER_SEC * 1000.0f);
  if (duration_ms == 0 && ml > 0) duration_ms = 1; // minimum tick

  Serial.printf("Pump %u -> GPIO(dir=%u, step=%u, en=%u), %u mL -> %lums\n",
                pumpNo, P.dir, P.step, P.en, ml, duration_ms);

  digitalWrite(P.en, LOW); // ensure disabled before start
  delay(10);
  runPumpMs(P, duration_ms, DIR_CLOCKWISE);
  Serial.println("Dispense done.");
  sendDoneAck(); // <-- completion response
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Milk Dispenser Node (MCP2515) ===");

  pinMode(IND_LED, OUTPUT);
  digitalWrite(IND_LED, LOW);

  // init pump pins
  for (auto &p : pumps) {
    pinMode(p.dir, OUTPUT);
    pinMode(p.step, OUTPUT);
    pinMode(p.en, OUTPUT);
    digitalWrite(p.en, LOW);      // disabled
    digitalWrite(p.step, LOW);
    digitalWrite(p.dir, DIR_CLOCKWISE ? HIGH : LOW);
  }

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

  Serial.printf("Ready. DEVICE_ID=0x%03X owns pumps %u..%u @500k\n",
                DEVICE_ID,
                (DEVICE_ID==0x104)?1:5,
                (DEVICE_ID==0x104)?4:8);
  Serial.println("Format: b0=01 blink, b1=pump(1..8), b2..b3=amount mL (BIG-ENDIAN).");

  // Heartbeat: send immediately, then schedule next
  sendHeartbeat();
  hb_next_ms = millis() + HEARTBEAT_INTERVAL_MS;
}

void loop() {
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) handleFrame(rx);

  // Heartbeat scheduler (note: dispensing is blocking; heartbeat may be delayed during a pour)
  uint32_t now = millis();
  if ((int32_t)(now - hb_next_ms) >= 0) {
    sendHeartbeat();
    hb_next_ms = now + HEARTBEAT_INTERVAL_MS;
  }

  delay(2);
}
