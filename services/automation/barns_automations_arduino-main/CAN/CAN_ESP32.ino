// ===== ESP32-S3 + MCP2515 CAN RX -> control two GPIOs with per-ID commands =====
// Library: "mcp2515" by autowp (Arduino Library Manager)

#include <SPI.h>
#include <mcp2515.h>

// ==== SPI pins (use the ones that WORKED for you) ====
#define PIN_SCK    23
#define PIN_MOSI   23
#define PIN_MISO   19
#define PIN_CS      5
// #define PIN_INT   2   // optional; polling is fine

// ==== LED (output) pins ====
#define LED1_PIN   15     // ID 0x121 controls this
#define LED2_PIN   13     // ID 0x122 controls this

// ==== CAN/MCP settings ====
#define MCP_CLK    MCP_8MHZ
#define BUS_SPEED  CAN_500KBPS

// Two control IDs (standard frames)
#define ID_LED1    0x121
#define ID_LED2    0x122

// Command bytes
#define CMD_LOW    0x00   // force LOW, cancel timer
#define CMD_HIGH_T 0x01   // HIGH for duration (0 = latch HIGH until CMD_LOW)

// MCP2515 sanity registers
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;

MCP2515 mcp(PIN_CS);

// simple “timerable” output
struct LedCtrl {
  uint8_t  pin;
  uint16_t id;          // 11-bit
  bool     latched;     // true if “stay high until off”
  uint32_t until_ms;    // 0 if no timer active; else auto-OFF time
};

LedCtrl leds[2] = {
  { LED1_PIN, ID_LED1, false, 0 },
  { LED2_PIN, ID_LED2, false, 0 }
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

static bool initMCP() {
  mcp.reset();
  delay(15); // osc settle

  uint8_t canstat = readReg(REG_CANSTAT);
  uint8_t canctrl = readReg(REG_CANCTRL);
  Serial.printf("After reset: CANSTAT=0x%02X CANCTRL=0x%02X (expect 0x8x)\n", canstat, canctrl);

  MCP2515::ERROR e = mcp.setBitrate(BUS_SPEED, MCP_CLK);
  if (e != MCP2515::ERROR_OK) {
    Serial.printf("setBitrate error=%d (check MCP clock & wiring)\n", (int)e);
    return false;
  }

  mcp.setNormalMode();
  delay(5);

  canstat = readReg(REG_CANSTAT);
  uint8_t opmod = (canstat >> 5) & 0x07;
  Serial.printf("After NormalMode: CANSTAT=0x%02X (OPMOD=%u; 0=Normal)\n", canstat, opmod);
  return (opmod == 0);
}

static LedCtrl* findLedById(uint32_t id) {
  for (auto &l : leds) if (l.id == id) return &l;
  return nullptr;
}

static void setLedHigh(LedCtrl* l, uint32_t dur_s) {
  digitalWrite(l->pin, HIGH);
  if (dur_s == 0) {
    l->latched  = true;     // stay high until told to go low
    l->until_ms = 0;
    Serial.printf("GPIO%d -> HIGH (latched)\n", l->pin);
  } else {
    l->latched  = false;
    l->until_ms = millis() + dur_s * 1000UL;
    Serial.printf("GPIO%d -> HIGH for %lu s\n", l->pin, (unsigned long)dur_s);
  }
}

static void setLedLow(LedCtrl* l) {
  digitalWrite(l->pin, LOW);
  l->latched  = false;
  l->until_ms = 0;
  Serial.printf("GPIO%d -> LOW\n", l->pin);
}

static void handleFrame(const struct can_frame& rx) {
  const bool ext = rx.can_id & CAN_EFF_FLAG;
  if (ext) return; // only standard IDs here

  const uint32_t id = rx.can_id & CAN_SFF_MASK;
  LedCtrl* l = findLedById(id);
  if (!l) return; // not for us

  Serial.printf("RX STD ID=0x%03lX DLC=%d Data:", id, rx.can_dlc);
  for (uint8_t i = 0; i < rx.can_dlc; i++) Serial.printf(" %02X", rx.data[i]);
  Serial.println();

  if (rx.can_dlc == 0) return; // need at least a command byte
  const uint8_t cmd = rx.data[0];

  // duration parsing
  uint32_t dur_s = 0;
  if (rx.can_dlc >= 3) {
    dur_s = (uint32_t(rx.data[1]) << 8) | uint32_t(rx.data[2]); // 16-bit seconds (big-endian)
  } else if (rx.can_dlc >= 2) {
    dur_s = rx.data[1]; // 8-bit seconds
  }

  switch (cmd) {
    case CMD_LOW:
      setLedLow(l);
      break;
    case CMD_HIGH_T:
      setLedHigh(l, dur_s); // 0 = latch
      break;
    default:
      Serial.println("Unknown command byte (use 0x00=LOW, 0x01=HIGH+t).");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32-S3 + MCP2515: per-ID LED control ===");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  bool ok = false;
  for (int i = 0; i < 4 && !ok; i++) {
    ok = initMCP();
    if (!ok) { Serial.println("Retrying init..."); delay(120); }
  }
  if (!ok) {
    Serial.println("FAILED to enter Normal mode. Check MCP clk/wiring/levels.");
    while (1) delay(1000);
  }

  Serial.printf("LED1 GPIO %d ↔ ID 0x%03X\n", LED1_PIN, ID_LED1);
  Serial.printf("LED2 GPIO %d ↔ ID 0x%03X\n", LED2_PIN, ID_LED2);
  Serial.println("Commands:");
  Serial.println("  CMD_HIGH_T (0x01) + duration seconds (8- or 16-bit, big-endian). 0 duration = latch.");
  Serial.println("  CMD_LOW    (0x00) turns LED LOW immediately and cancels timer.");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  LED1 ON for 5s:   cansend can0 121#0105");
  Serial.println("  LED2 ON for 300s: cansend can0 122#01012C      (0x012C = 300)");
  Serial.println("  LED1 latch HIGH:  cansend can0 121#0100");
  Serial.println("  LED1 force LOW:   cansend can0 121#00");
}

void loop() {
  // process CAN frames
  struct can_frame rx;
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleFrame(rx);
  }

  // handle auto-off timers
  const uint32_t now = millis();
  for (auto &l : leds) {
    if (!l.latched && l.until_ms != 0 && (int32_t)(now - l.until_ms) >= 0) {
      setLedLow(&l);
    }
  }

  delay(2);
}
