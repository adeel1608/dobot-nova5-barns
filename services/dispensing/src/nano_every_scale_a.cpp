#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <EEPROM.h>
#include <limits.h>
#include <math.h>

/*===================== I2C address =====================*/
#ifndef I2C_ADDR
  #define I2C_ADDR 0x29        // set 0x29 on the second Nano Every
#endif

/*=================== Pin assignments ===================*/
// HX711
const uint8_t HX_DOUT = A2;
const uint8_t HX_SCK  = A3;

// Button (active LOW) and Buzzer
const uint8_t BTN_PIN = A1;
const uint8_t BUZ_PIN = A0;

// 7-segment: segments = D5..D12 (A..G,DP), digits = D2..D4
const uint8_t SEG_PINS[8] = {5,6,7,8,9,10,11,12};
const uint8_t DIG_PINS[3] = {2,3,4};

/*=================== Display helpers ===================*/
static const uint8_t LUT[10] = {
  0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,
  0b01101101,0b01111101,0b00000111,0b01111111,0b01101111
};

volatile uint8_t dispBuf[3] = {0,0,0};
volatile uint8_t curDigit = 0;
volatile bool    dispEnabled = true;

inline void allDigitsOff(){ digitalWrite(DIG_PINS[0],LOW);digitalWrite(DIG_PINS[1],LOW);digitalWrite(DIG_PINS[2],LOW); }
inline void writeSeg(uint8_t m){ for(uint8_t i=0;i<8;i++) digitalWrite(SEG_PINS[i], (m>>i)&1); }
inline void setRawMasks(uint8_t m0,uint8_t m1,uint8_t m2){ noInterrupts(); dispBuf[0]=m0;dispBuf[1]=m1;dispBuf[2]=m2; interrupts(); }

// show one decimal with DP on the ones digit: [ tens ] [ ones• ] [ tenths ]
void showNumber_1dp(float g){
  if (fabs(g) < 0.2f) g = 0.0f;
  if (g > 99.9f) g = 99.9f; if (g < -9.9f) g = -9.9f;
  bool neg = g<0; int t10 = (int)lroundf(fabs(g)*10.0f);
  uint8_t t=(t10/100)%10, o=(t10/10)%10, d=t10%10;
  uint8_t m0=0, m1=LUT[o]|0b10000000, m2=LUT[d];
  if (neg && t==0) m0 = 0b01000000; else if (t>0) m0=LUT[t];
  setRawMasks(m0,m1,m2);
}

/*=================== HX711 + filtering =================*/
HX711 scale;

// persisted config
struct Persist { uint32_t magic; int32_t offset; float cal; };
Persist cfg{0xA55A5A55, 0, -414.075f};   // start from a sane working value; will be replaced after cal

// timings
const uint32_t MUX_US=700, SAMPLE_MS=90, READ_GUARD_US=800;

// button thresholds
const uint16_t DEBOUNCE_MS=40, LONGPRESS_MS=3000;

// fixed reference for calibration
const float CAL_MASS_G=37.3f;

// EMA smoothing
float ema_g = NAN; const float EMA_ALPHA=0.25f;

// tiny ring for median-of-5
template<size_t N> struct Ring{ long v[N]; uint8_t n=0,i=0;
  void push(long x){ v[i]=x; i=(i+1)%N; if(n<N) n++; } bool full()const{ return n==N; }};
Ring<5> rawfifo;

long readRawOnce(){
  long r;
  // Quiet display around conversion to cut noise coupling
  dispEnabled=false; allDigitsOff(); delayMicroseconds(READ_GUARD_US);
  if (!scale.is_ready()) { dispEnabled=true; return LONG_MIN; }
  r = scale.read();
  delayMicroseconds(READ_GUARD_US);
  dispEnabled=true;
  return r;
}
bool readRawMedian(long &out){
  long r=readRawOnce(); if(r==LONG_MIN) return false;
  rawfifo.push(r); if(!rawfifo.full()) return false;
  long w[5]; for(uint8_t k=0;k<5;k++) w[k]=rawfifo.v[k];
  for(uint8_t a=1;a<5;a++){ long t=w[a]; int8_t b=a-1; while(b>=0&&w[b]>t){w[b+1]=w[b]; b--; } w[b+1]=t; }
  out=w[2]; return true;
}

/*================ EEPROM =================*/
void loadCfg(){ Persist p; EEPROM.get(0,p); if(p.magic==0xA55A5A55) cfg=p; }
void saveCfg(){ EEPROM.put(0,cfg); }

/*================ App state & buzzer ===============*/
#define TARGET_ON_G   8.0f
#define TARGET_OFF_G  7.6f
bool buzOn=false;

enum RunState { ARMED, DISPENSE };
RunState rs=ARMED;

static inline uint8_t makeStatus(){ uint8_t s=0; if(buzOn) s|=0x01; if(rs==ARMED) s|=0x02; return s; }
static void encodeWeightDG(int16_t &dg){ float g=isnan(ema_g)?0.0f:ema_g; long d=lroundf(g*10.0f);
  if(d<-32768)d=-32768; if(d>32767)d=32767; dg=(int16_t)d; }

/*================ I2C protocol =============*/
static uint8_t txBuf[8]; static uint8_t txLen=0;

static void onReceiveI2C(int nbytes){
  if(nbytes<=0) return;
  uint8_t cmd=Wire.read(); txLen=0;
  switch(cmd){
    case 0x55: txBuf[0]=0xA0; txLen=1; break;                 // PING
    case 0x01:{ int16_t dg; encodeWeightDG(dg);
                txBuf[0]=makeStatus(); txBuf[1]=(dg>>8)&0xFF; txBuf[2]=dg&0xFF; txLen=3; } break; // READ_WEIGHT
    case 0x02: { // TARE
      // reuse doTare; respond OK
      uint32_t t0=millis(); long acc=0; uint8_t cnt=0;
      while ((uint32_t)(millis()-t0) < 1200) { long m; if(readRawMedian(m)){ acc+=m; cnt++; } delay(5); }
      if (cnt) { cfg.offset=(int32_t)(acc/(long)cnt); saveCfg(); }
      txBuf[0]=0x00; txLen=1;
    } break;
    case 0x03: txBuf[0]=makeStatus(); txLen=1; break;         // GET_STATUS
    default:   txBuf[0]=0xFF; txLen=1; break;
  }
  while(Wire.available()) (void)Wire.read(); // drain
}
static void onRequestI2C(){ if(txLen) Wire.write(txBuf,txLen); else { uint8_t e=0xFE; Wire.write(&e,1);} }

/*================ Actions ==================*/
void doTare(uint16_t settle_ms=1200){
  uint32_t t0=millis(); long acc=0; uint8_t cnt=0;
  while((uint32_t)(millis()-t0)<settle_ms){ long m; if(readRawMedian(m)){ acc+=m; cnt++; } delay(5); }
  if(cnt){ cfg.offset=(int32_t)(acc/(long)cnt); saveCfg(); }
}

void doCalibrate(){
  // Small countdown on the display (optional visual)
  for(int i=3;i>0;i--){ setRawMasks(0, LUT[i]|0b10000000, LUT[0]); delay(300); }

  // Average delta at REF weight
  uint32_t t0=millis(); long acc=0; uint8_t cnt=0;
  while((uint32_t)(millis()-t0)<1500){ long m; if(readRawMedian(m)){ acc+=(m-cfg.offset); cnt++; } delay(5); }
  if(!cnt) return;

  float delta = (float)acc / (float)cnt;
  if (fabs(delta) < 100.0f) return;

  cfg.cal = delta / CAL_MASS_G;             // counts per gram
  if (cfg.cal > 0) cfg.cal = -cfg.cal;      // enforce negative if your load cell is “inverted”
  saveCfg();

  // Blink REF weight vs 0 as feedback
  for (int k=0;k<6;k++){ showNumber_1dp( (k&1)?0.0f:CAL_MASS_G ); delay(180); }
}

/*================ Button FSM (short=TARE, long=CAL) ===============*/
enum BtnState{UP,DEBOUNCE_DOWN,DOWN,LONG,DEBOUNCE_UP};
BtnState bs=UP; uint32_t tChange=0;

void handleButton(){
  bool pressed=(digitalRead(BTN_PIN)==LOW);
  switch(bs){
    case UP:
      if(pressed){ bs=DEBOUNCE_DOWN; tChange=millis(); }
      break;
    case DEBOUNCE_DOWN:
      if(!pressed){ bs=UP; }
      else if((uint32_t)(millis()-tChange)>=DEBOUNCE_MS){ bs=DOWN; tChange=millis(); }
      break;
    case DOWN:
      if(!pressed){ bs=DEBOUNCE_UP; tChange=millis(); }
      else if((uint32_t)(millis()-tChange)>=LONGPRESS_MS){ bs=LONG; doCalibrate(); }
      break;
    case LONG:
      if(!pressed){ bs=DEBOUNCE_UP; tChange=millis(); }
      break;
    case DEBOUNCE_UP:
      if(pressed){ bs=DOWN; }
      else if((uint32_t)(millis()-tChange)>=DEBOUNCE_MS){
        // short press (we weren’t in LONG by definition here)
        doTare();
        // safety reset
        buzOn=false; digitalWrite(BUZ_PIN,LOW); rs=ARMED;
        bs=UP;
      }
      break;
  }
}

/*================ Setup/loop ===============*/
void setup(){
  Serial.begin(115200); delay(30);

  pinMode(BTN_PIN,INPUT_PULLUP);
  pinMode(BUZ_PIN,OUTPUT); digitalWrite(BUZ_PIN,LOW);
  for(uint8_t i=0;i<8;i++){ pinMode(SEG_PINS[i],OUTPUT); digitalWrite(SEG_PINS[i],LOW); }
  for(uint8_t i=0;i<3;i++){ pinMode(DIG_PINS[i],OUTPUT); digitalWrite(DIG_PINS[i],LOW); }

  scale.begin(HX_DOUT,HX_SCK);
  loadCfg();

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.print(F("Nano I2C scale @0x")); Serial.println(I2C_ADDR,HEX);
  Serial.print(F("Boot offset=")); Serial.print(cfg.offset);
  Serial.print(F("  cal=")); Serial.println(cfg.cal,6);
}

void loop(){
  // 1) display multiplex
  static uint32_t tMux=0; if((uint32_t)(micros()-tMux)>=MUX_US){
    tMux=micros(); allDigitsOff();
    if(dispEnabled){ writeSeg(dispBuf[curDigit]); digitalWrite(DIG_PINS[curDigit],HIGH); }
    curDigit++; if(curDigit>=3) curDigit=0;
  }

  // 2) periodic HX711 sample + UI logic
  static uint32_t tSample=0; if((uint32_t)(millis()-tSample)>=SAMPLE_MS){
    tSample=millis();
    long m; if(readRawMedian(m)){
      long delta=m-cfg.offset;
      float g = (cfg.cal==0.0f)?0.0f:(delta/cfg.cal);

      if(isnan(ema_g)) ema_g=g; else ema_g = EMA_ALPHA*g + (1.0f-EMA_ALPHA)*ema_g;

      // Buzzer hysteresis (keep as before)
      if(!buzOn && ema_g>=TARGET_ON_G){ buzOn=true; digitalWrite(BUZ_PIN,HIGH); }
      else if(buzOn && ema_g<=TARGET_OFF_G){ buzOn=false; digitalWrite(BUZ_PIN,LOW); }

      showNumber_1dp(ema_g);

      // Optional throttled debug
      static uint8_t sp=0; if(++sp>=4){
        sp=0;
        Serial.print(F("raw=")); Serial.print(m);
        Serial.print(F(" off=")); Serial.print(cfg.offset);
        Serial.print(F(" med=")); Serial.print(delta);
        Serial.print(F(" cal=")); Serial.print(cfg.cal,6);
        Serial.print(F(" g=")); Serial.print(ema_g,2);
        Serial.print(F("  rs=")); Serial.println((int)rs);
      }
    }
  }

  // 3) button (short press = TARE, long press = CAL)
  handleButton();
}
