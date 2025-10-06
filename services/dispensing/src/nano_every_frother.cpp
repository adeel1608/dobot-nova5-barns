#include <Arduino.h>

#define DEBUG_USB 1

// ===================== Pin assignments (Nano Every) =====================
// Solenoid
#define PIN_SOLENOID   2   // ACTIVE HIGH

// Motor M1 (Cleaner)
#define M1_IN1         3
#define M1_IN2         4

// Motor M2 (Control)
#define M2_IN1         5
#define M2_IN2         6

// MAX6675 Thermocouple (bit-banged)
#define TC_SO          12  // DO
#define TC_CS          10  // CS#
#define TC_SCK         13  // SCK

// ===================== UART =====================
// Nano Every: Serial1 is on pins 0/1 and connects to Mega Serial1
#define FROTH_SERIAL   Serial1

// ===================== Modes / Commands =====================
enum FrothMode : uint8_t {
  MODE_OFF     = 0,
  MODE_STBY    = 1,
  MODE_INIT    = 2,
  MODE_FROTH   = 3,
  MODE_CLEAN   = 4
};

// Defaults (scaled)
static const uint16_t DEFAULT_INIT_SECS_x100      = 300;   // 3.00 s
static const uint16_t DEFAULT_FROTH_TARGET_x100   = 6000;  // 60.00 C
static const uint16_t DEFAULT_FROTH_TIMEOUT_x100  = 18000; // 180.00 s
static const uint16_t DEFAULT_CLEAN_TVALVE_x100   = 500;   // 5.00 s
static const uint16_t DEFAULT_CLEAN_TSTEAM_x100   = 500;   // 5.00 s
static const uint16_t DEFAULT_CLEAN_TSTBY_x100    = 500;   // 5.00 s

struct RuntimeConfig {
  uint16_t init_secs_x100;
  uint16_t froth_targetCx100;
  uint16_t froth_timeoutSecs_x100;
  uint16_t clean_tValve_x100;
  uint16_t clean_tSteam_x100;
  uint16_t clean_tStby_x100;
};

static RuntimeConfig cfg = {
  DEFAULT_INIT_SECS_x100,
  DEFAULT_FROTH_TARGET_x100,
  DEFAULT_FROTH_TIMEOUT_x100,
  DEFAULT_CLEAN_TVALVE_x100,
  DEFAULT_CLEAN_TSTEAM_x100,
  DEFAULT_CLEAN_TSTBY_x100
};

// ===================== State =====================
static volatile FrothMode currentMode = MODE_OFF;
static volatile bool busy = false;
static volatile uint8_t errorMask = 0;

// For timing sequences
static uint32_t stepStartMs = 0;
static uint8_t lastReportedBusy = 0xFF;
static uint8_t lastReportedMode = 0xFF;

// Command context
static uint8_t lastCmd = 0;
static uint16_t arg1 = 0, arg2 = 0, arg3 = 0; // scaled x100

// ===================== MAX6675 (bit-banged) =====================
static uint32_t lastTcMs = 0;
static uint16_t lastTempCx100 = 0;
static bool lastTempValid = false;

static void tc_init() {
  pinMode(TC_CS, OUTPUT);
  pinMode(TC_SCK, OUTPUT);
  pinMode(TC_SO, INPUT);
  digitalWrite(TC_CS, HIGH);
  digitalWrite(TC_SCK, LOW);
}

// MAX6675: read 16-bit value, bits [14:3] are temp in 0.25 C
static void tc_read_once(uint16_t &tempCx100, bool &valid) {
  digitalWrite(TC_CS, LOW);
  delayMicroseconds(1);
  uint16_t v = 0;
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(TC_SCK, HIGH);
    delayMicroseconds(1);
    v = (uint16_t)((v << 1) | (digitalRead(TC_SO) ? 1 : 0));
    digitalWrite(TC_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(TC_CS, HIGH);
  // Bit 2 is fault; if 1 => no thermocouple
  bool fault = v & 0x0004;
  valid = !fault;
  if (valid) {
    uint16_t raw = (v >> 3) & 0x0FFF; // 12 bits of temp in 0.25C
    // Convert 0.25C units to Cx100: multiply by 25
    uint32_t tmp = (uint32_t)raw * 25U;
    if (tmp > 65535U) tmp = 65535U;
    tempCx100 = (uint16_t)tmp;
  } else {
    tempCx100 = lastTempCx100; // keep last
  }
}

static void tc_read_cached(uint16_t &tempCx100, bool &valid) {
  uint32_t now = millis();
  if (now - lastTcMs >= 250) { // ~250ms cadence
    tc_read_once(lastTempCx100, lastTempValid);
    lastTcMs = now;
  }
  tempCx100 = lastTempCx100;
  valid = lastTempValid;
}

// ===================== Actuators =====================
static inline void solenoid_set(bool on) {
  digitalWrite(PIN_SOLENOID, on ? HIGH : LOW);
}

static inline void m1_off() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
}
static inline void m1_fwd() {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
}
static inline void m1_rev() {
  digitalWrite(M1_IN1, LOW);  digitalWrite(M1_IN2, HIGH);
}

static inline void m2_off() {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}
static inline void m2_fwd() {
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
}
static inline void m2_rev() {
  digitalWrite(M2_IN1, LOW);  digitalWrite(M2_IN2, HIGH);
}

// ===================== Events =====================
static void emit_state() {
  if ((uint8_t)currentMode != lastReportedMode || (uint8_t)busy != lastReportedBusy) {
    FROTH_SERIAL.print("FR_EVT:STATE,");
    FROTH_SERIAL.print((int)currentMode);
    FROTH_SERIAL.print(",");
    FROTH_SERIAL.print((int)(busy ? 1 : 0));
    FROTH_SERIAL.print(",");
    FROTH_SERIAL.println((int)errorMask);
#if DEBUG_USB
    Serial.print("FR_EVT:STATE,");
    Serial.print((int)currentMode);
    Serial.print(",");
    Serial.print((int)(busy ? 1 : 0));
    Serial.print(",");
    Serial.println((int)errorMask);
#endif
    lastReportedMode = (uint8_t)currentMode;
    lastReportedBusy = (uint8_t)(busy ? 1 : 0);
  }
}

static void emit_temp() {
  uint16_t t; bool v;
  tc_read_cached(t, v);
  FROTH_SERIAL.print("FR_EVT:TEMP,");
  FROTH_SERIAL.print((int)t);
  FROTH_SERIAL.print(",");
  FROTH_SERIAL.println(v ? 1 : 0);
#if DEBUG_USB
  Serial.print("FR_EVT:TEMP,");
  Serial.print((int)t);
  Serial.print(",");
  Serial.println(v ? 1 : 0);
#endif
}

static void emit_done(uint8_t cmd, uint8_t err) {
  FROTH_SERIAL.print("FR_EVT:DONE,");
  FROTH_SERIAL.print((int)cmd);
  FROTH_SERIAL.print(",");
  FROTH_SERIAL.println((int)err);
#if DEBUG_USB
  Serial.print("FR_EVT:DONE,");
  Serial.print((int)cmd);
  Serial.print(",");
  Serial.println((int)err);
#endif
}

static void emit_error(uint8_t code) {
  FROTH_SERIAL.print("FR_EVT:ERROR,");
  FROTH_SERIAL.println((int)code);
#if DEBUG_USB
  Serial.print("FR_EVT:ERROR,");
  Serial.println((int)code);
#endif
}

// ===================== Control helpers =====================
static void set_mode_off() {
  m1_off(); m2_off(); solenoid_set(false);
  currentMode = MODE_OFF; busy = false; errorMask = 0;
  emit_state();
}

static void set_mode_stby() {
  m1_rev(); m2_fwd(); solenoid_set(false);
  currentMode = MODE_STBY; busy = false; errorMask = 0;
  emit_state();
}

// ===================== Command handling =====================
static void start_init(uint16_t secs_x100) {
  m1_off(); m2_rev(); solenoid_set(false);
  currentMode = MODE_INIT; busy = true; errorMask = 0;
  stepStartMs = millis();
  arg1 = secs_x100; // reuse arg1 for remaining time
  emit_state();
}

static void start_froth(uint16_t targetCx100, uint16_t timeoutSecs_x100) {
  m1_off(); m2_rev(); solenoid_set(false);
  currentMode = MODE_FROTH; busy = true; errorMask = 0;
  stepStartMs = millis();
  arg1 = targetCx100; // store target
  arg2 = timeoutSecs_x100; // store timeout
  emit_state();
}

static void start_clean(uint16_t tValve_x100, uint16_t tSteam_x100, uint16_t tStby_x100) {
  // Sequence step 1: Start → M1 Forward
  m2_off(); solenoid_set(false); m1_fwd();
  currentMode = MODE_CLEAN; busy = true; errorMask = 0;
  stepStartMs = millis();
  arg1 = tValve_x100; // use arg1 as tValve
  arg2 = tSteam_x100; // use arg2 as tSteam
  arg3 = tStby_x100;  // use arg3 as tStandby
  emit_state();
}

static void handle_cmd(uint8_t cmd, uint8_t flags, uint16_t a1_opt, uint16_t a2_opt, uint16_t a3_opt) {
  lastCmd = cmd;
  switch (cmd) {
    case 0: // OFF
      set_mode_off();
      emit_done(0, 0);
      break;
    case 1: // STANDBY
      set_mode_stby();
      emit_done(1, 0);
      break;
    case 2: { // INIT (M2 reverse for set time)
      uint16_t secs = (flags & 0x01) && a1_opt ? a1_opt : cfg.init_secs_x100;
      start_init(secs);
      break;
    }
    case 3: { // FROTH (target temp, timeout)
      uint16_t tgt = (flags & 0x01) && a1_opt ? a1_opt : cfg.froth_targetCx100;
      uint16_t to  = (flags & 0x01) && a2_opt ? a2_opt : cfg.froth_timeoutSecs_x100;
      start_froth(tgt, to);
      break;
    }
    case 4: { // CLEAN sequence
      uint16_t tv = (flags & 0x01) && a1_opt ? a1_opt : cfg.clean_tValve_x100;
      uint16_t ts = (flags & 0x01) && a2_opt ? a2_opt : cfg.clean_tSteam_x100;
      uint16_t tb = (flags & 0x01) && a3_opt ? a3_opt : cfg.clean_tStby_x100;
      start_clean(tv, ts, tb);
      break;
    }
    default:
      emit_error(10); // unknown cmd
      break;
  }
}

// ===================== Periodic runner =====================
static void run_init() {
  uint32_t elapsed = millis() - stepStartMs;
  uint32_t targetMs = (uint32_t)arg1 * 10; // x100 seconds -> ms
  if (elapsed >= targetMs) {
    // Complete INIT then enter STANDBY automatically
    m2_off();
    set_mode_stby();
    emit_done(2, 0);
  }
}

static void run_froth() {
  // Temperature monitoring cadence
  static uint32_t lastTempEmitMs = 0;
  if (millis() - lastTempEmitMs >= 300) { emit_temp(); lastTempEmitMs = millis(); }

  uint16_t t; bool v; tc_read_cached(t, v);
  if (!v) {
    // thermocouple fault -> abort
    m2_off(); busy = false; errorMask |= 0x01; // TC fault bit
    emit_state(); emit_error(1);
    emit_done(3, errorMask);
    return;
  }
  if (t >= arg1) {
    // target reached → go to STANDBY (normal position)
    m2_off();
    set_mode_stby();
    emit_done(3, 0);
    return;
  }
  uint32_t elapsed = millis() - stepStartMs;
  uint32_t timeoutMs = (uint32_t)arg2 * 10; // x100 seconds -> ms
  if (elapsed >= timeoutMs) {
    m2_off(); busy = false; errorMask |= 0x02; // timeout bit
    emit_state(); emit_error(3);
    emit_done(3, errorMask);
  }
}

static void run_clean() {
  uint32_t elapsed = millis() - stepStartMs;
  uint32_t tValveMs = (uint32_t)arg1 * 10;
  uint32_t tSteamMs = (uint32_t)arg2 * 10;
  uint32_t tStbyMs  = (uint32_t)arg3 * 10;

  if (elapsed < tValveMs) {
    // Step 1 already set (M1 Forward)
    return;
  } else if (elapsed < (tValveMs + tSteamMs)) {
    // Step 2: Solenoid ON
    solenoid_set(true);
    return;
  } else if (elapsed < (tValveMs + tSteamMs + tStbyMs)) {
    // Step 3: Solenoid OFF + M2 Reverse ON
    solenoid_set(false);
    m2_rev();
    return;
  } else {
    // Step 4: STANDBY state and DONE
    set_mode_stby();
    emit_done(4, 0);
  }
}

// ===================== Command parsing =====================
static char rxBuf[96];
static int rxIdx = 0;

static void process_serial() {
  while (FROTH_SERIAL.available()) {
    char c = FROTH_SERIAL.read();
    if (c == '\n') {
      rxBuf[rxIdx] = 0;
      // Expect: FR_CMD:cmd,flags[,a1[,a2[,a3]]]
      if (strncmp(rxBuf, "FR_CMD:", 7) == 0) {
        char *p = rxBuf + 7;
        uint8_t cmd = (uint8_t)atoi(p);
        char *comma = strchr(p, ',');
        uint8_t flags = 0;
        uint16_t a1=0,a2=0,a3=0; 
        if (comma) {
          flags = (uint8_t)atoi(comma + 1);
          // parse optional a1,a2,a3
          char *p2 = strchr(comma + 1, ',');
          if (p2) { a1 = (uint16_t)atoi(p2 + 1); char *p3 = strchr(p2 + 1, ','); if (p3) { a2 = (uint16_t)atoi(p3 + 1); char *p4 = strchr(p3 + 1, ','); if (p4) { a3 = (uint16_t)atoi(p4 + 1); } } }
        }
        handle_cmd(cmd, flags, a1, a2, a3);
      }
      rxIdx = 0;
    } else if (rxIdx < (int)sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }
}

// ===================== Arduino entry points =====================
void setup() {
  pinMode(PIN_SOLENOID, OUTPUT);
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  m1_off(); m2_off(); solenoid_set(false);

  tc_init();

  FROTH_SERIAL.begin(115200);
#if DEBUG_USB
  Serial.begin(115200);
#endif
  delay(100);

  set_mode_off();
}

void loop() {
  process_serial();

  switch (currentMode) {
    case MODE_INIT:  if (busy) run_init();  break;
    case MODE_FROTH: if (busy) run_froth(); break;
    case MODE_CLEAN: if (busy) run_clean(); break;
    default: break;
  }

  // Periodic state emission (only on changes)
  emit_state();
}


