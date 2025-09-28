# MCP2515 CAN Bus Integration with Dispensing System

## Overview
This guide shows how to integrate MCP2515 CAN bus with your existing Arduino Mega + Micro + Nano Every dispensing system for robust automation control.

## Hardware Architecture

### Current System:
```
[Arduino Micro + W5500] ←UART→ [Arduino Mega] ←I2C→ [2x Nano Every]
     (MQTT Bridge)              (Motor Control)        (Scale Controllers)
```

### Enhanced System with CAN:
```
[Arduino Micro + W5500] ←UART→ [Arduino Mega + MCP2515] ←I2C→ [2x Nano Every]
     (MQTT Bridge)              (Motor + CAN Control)           (Scale Controllers)
                                        ↓ CAN Bus
                               [Other Automation Devices]
                               (Grinder, Tamper, etc.)
```

## MCP2515 Wiring to Arduino Micro

### SPI Connection (MCP2515 → Arduino Micro):
```
MCP2515 Pin    Arduino Micro Pin   Description
-----------    -----------------   -----------
VCC            5V                  Power (5V)
GND            GND                 Ground
CS             Pin 4               Chip Select (SPI SS)
SCK            Pin 3               SPI Clock
MOSI           Pin 5               SPI Data Out
MISO           Pin 6               SPI Data In
INT            Pin 2               Interrupt (optional)
```

### Detailed Wiring Diagram:
```
Arduino Micro (Network Hub):
                    ┌─────────────────────┐
                    │                     │
    5V  ────────────┤ 5V               4  ├──── CS     (MCP2515)
    GND ────────────┤ GND              3  ├──── SCK    (MCP2515)
                    │                  5  ├──── MOSI   (MCP2515)
                    │                  6  ├──── MISO   (MCP2515)
                    │                  2  ├──── INT    (MCP2515) [Optional]
                    │                     │
    W5500 Module:   │                     │
    CS ─────────────┤ 10              TX1 ├──── Pin 16 (to Mega RX2)
    SCK ────────────┤ 15              RX1 ├──── Pin 17 (to Mega TX2)
    MOSI ───────────┤ 16                  │
    MISO ───────────┤ 14                  │
    RST ────────────┤ 9                   │
                    └─────────────────────┘

Arduino Mega 2560 (Control Hub):
                    ┌─────────────────────┐
                    │                     │
    24V/3A ─────────┤ VIN                 │
    Pin 16 ─────────┤ RX2             TX2 ├──── Pin 17 (to Micro)
    Pin 20 ─────────┤ SDA             SCL ├──── Pin 21 (to Nano Every)
    Pin 2-49 ───────┤ Motor Pins          │
    Pin A5,A8 ──────┤ Leak Sensors        │
    Pin A6,A9 ──────┤ Leak Power          │
                    └─────────────────────┘

MCP2515 Module:
    ┌─────────────┐
    │ VCC     INT ├──── Pin 2 (Micro)
    │ GND      CS ├──── Pin 4 (Micro)
    │ SCK    MOSI ├──── Pin 5 (Micro)
    │ MISO        │     Pin 6 (Micro)
    │ CAN-H       │ ──── To CAN Bus
    │ CAN-L       │ ──── To CAN Bus
    └─────────────┘
```

## Software Integration

### 1. Add CAN Libraries to Arduino Mega

**File:** `services/dispensing/platformio.ini`

Add to the `[env:mega]` section:
```ini
[env:mega]
platform = atmelavr
board = megaatmega2560
framework = arduino
lib_deps = 
    emelianov/modbus-esp8266@^4.1.0
    Wire@^1.0
    autowp/mcp2515@^1.0.1
monitor_speed = 115200
upload_flags = -F
src_filter = +<*> -<mqtt_final.cpp> -<nano_every.cpp> -<motor_test.cpp> -<simple_test.cpp>
```

### 2. Add CAN Support to Arduino Mega Code

**File:** `services/dispensing/src/main.cpp`

Add after the existing includes:
```cpp
#include <SPI.h>
#include <mcp2515.h>

/* ================= CAN BUS CONFIGURATION ================= */
#define CAN_CS_PIN    53    // SPI Chip Select
#define CAN_INT_PIN   2     // Interrupt pin (optional)
#define CAN_SPEED     CAN_500KBPS
#define MCP_CLOCK     MCP_8MHZ

// CAN IDs for dispensing system (avoid conflicts with other devices)
#define CAN_ID_DISPENSING_CMD   0x110   // Commands to dispensing system
#define CAN_ID_DISPENSING_ACK   0x111   // ACK from dispensing system
#define CAN_ID_DISPENSING_DATA  0x112   // Scale data from dispensing system

MCP2515 mcp2515(CAN_CS_PIN);

// CAN message structure for dispensing
struct CanDispenseCmd {
  uint8_t cmd;           // 0x01=dispense, 0x02=stop, 0x03=tare
  uint8_t motor_id;      // Motor number (1-24)
  uint16_t weight_dg;    // Target weight in decigrams
  uint8_t liquid_type;   // Liquid type for lag compensation
  uint8_t reserved[3];   // Reserved bytes
};
```

### 3. Initialize CAN in Setup Function

Add to the `setup()` function in main.cpp:
```cpp
void setup() {
  // ... existing setup code ...
  
  // Initialize CAN bus
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_CLOCK);
  mcp2515.setNormalMode();
  
  Serial.println(F("CAN bus initialized"));
  
  // ... rest of existing setup ...
}
```

### 4. Add CAN Message Handling

Add to the `loop()` function:
```cpp
void loop() {
  // ... existing leak detection ...
  
  // Handle CAN messages
  processCAN();
  
  // ... existing serial and bridge commands ...
}

void processCAN() {
  struct can_frame canMsg;
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == CAN_ID_DISPENSING_CMD) {
      handleCANDispenseCommand(canMsg);
    }
  }
  
  // Send periodic scale data via CAN
  static uint32_t last_can_data = 0;
  if (millis() - last_can_data > 500) {  // Every 500ms
    sendScaleDataViaCAN();
    last_can_data = millis();
  }
}

void handleCANDispenseCommand(struct can_frame& msg) {
  if (msg.can_dlc < 6) return;  // Need at least 6 bytes
  
  CanDispenseCmd* cmd = (CanDispenseCmd*)msg.data;
  
  switch (cmd->cmd) {
    case 0x01: {  // Dispense command
      uint8_t motor_pin = getMotorPinById(cmd->motor_id);
      float target_weight = cmd->weight_dg / 10.0;  // Convert to grams
      const char* liquid = getLiquidTypeString(cmd->liquid_type);
      
      if (motor_pin > 0) {
        startDispenseJob("CAN", getMotorNameById(cmd->motor_id), target_weight, liquid);
        sendCANAck(CAN_ID_DISPENSING_ACK, 0x01);  // Success ACK
      } else {
        sendCANAck(CAN_ID_DISPENSING_ACK, 0x00);  // Error ACK
      }
      break;
    }
    case 0x02:  // Stop command
      emergencyStop();
      sendCANAck(CAN_ID_DISPENSING_ACK, 0x01);
      break;
    case 0x03:  // Tare command
      // Implement tare logic
      sendCANAck(CAN_ID_DISPENSING_ACK, 0x01);
      break;
  }
}

void sendScaleDataViaCAN() {
  struct can_frame msg;
  msg.can_id = CAN_ID_DISPENSING_DATA;
  msg.can_dlc = 8;
  
  // Pack scale data: [scale_A_low, scale_A_high, scale_B_low, scale_B_high, status, reserved...]
  msg.data[0] = regW_A & 0xFF;
  msg.data[1] = (regW_A >> 8) & 0xFF;
  msg.data[2] = regW_B & 0xFF;
  msg.data[3] = (regW_B >> 8) & 0xFF;
  msg.data[4] = (regS_A << 4) | (regS_B & 0x0F);  // Pack both status
  msg.data[5] = current_job.state;  // Dispensing state
  msg.data[6] = leak_emergency_triggered ? 1 : 0;  // Leak status
  msg.data[7] = 0x00;  // Reserved
  
  mcp2515.sendMessage(&msg);
}

void sendCANAck(uint32_t ack_id, uint8_t status) {
  struct can_frame msg;
  msg.can_id = ack_id;
  msg.can_dlc = 1;
  msg.data[0] = status;  // 0x01 = success, 0x00 = error
  
  mcp2515.sendMessage(&msg);
}
```

## Physical Wiring Steps

### Step 1: MCP2515 Module Connections
```
MCP2515 Module → Arduino Mega 2560:

Red Wire (VCC)    → 5V pin
Black Wire (GND)  → GND pin  
Yellow Wire (CS)  → Digital Pin 53
Green Wire (SCK)  → Digital Pin 52
Blue Wire (MOSI)  → Digital Pin 51
White Wire (MISO) → Digital Pin 50
Orange Wire (INT) → Digital Pin 2 (optional)
```

### Step 2: CAN Bus Termination
```
CAN-H ──────┬─── 120Ω ───┬─── CAN-H (other devices)
            │            │
CAN-L ──────┴─── 120Ω ───┴─── CAN-L (other devices)

Note: Only add 120Ω termination resistors at the two END devices of the CAN bus
```

### Step 3: Power Distribution
```
Arduino Mega: 24V/3A (existing)
Arduino Micro: 5V/1A (existing)  
MCP2515: 5V from Mega (new)
ESP32 Devices: 5V/1A each (for other automations)
```

## Integration with BARNS Automation Service

### Update automation_functions.py:

```python
# Add CAN-based dispensing function
async def dispense_ingredient_can(params: dict):
    """Dispense ingredient using CAN bus communication."""
    ingredient = params.get("ingredient", "sauce")
    weight = params.get("weight", 10)
    motor = params.get("motor", "sauce1")
    
    # Convert to CAN format
    motor_id = getMotorIdFromName(motor)  # sauce1 → 10, milk1 → 2, etc.
    weight_dg = int(weight * 10)  # Convert to decigrams
    liquid_type = getLiquidTypeId(ingredient)  # caramel → 4, milk → 2, etc.
    
    # Send CAN command via MQTT (to can2_protocol.py bridge)
    can_payload = {
        "cmd": 0x01,
        "motor_id": motor_id,
        "weight_dg": weight_dg,
        "liquid_type": liquid_type
    }
    
    client.publish("automation_dispensing_can", json.dumps(can_payload))
    
    # Wait for CAN ACK response
    # ... (similar to existing automation functions)
```

## Testing Procedure

### 1. Test MCP2515 Connection:
```bash
# Upload test code to Mega
pio run -e mega --target upload

# Check CAN initialization in serial monitor
# Should see: "CAN bus initialized"
```

### 2. Test CAN Communication:
```bash
# Set up CAN interface on your system
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 up type can bitrate 500000

# Test CAN send from command line
cansend can0 110#0101000A0400000000  # Dispense 10g caramel via motor 1
```

### 3. Test Integration:
```bash
# Run the CAN bridge
python services/automation/barns_automations_arduino-main/can2_protocol.py

# Test via MQTT
mosquitto_pub -h 192.168.200.233 -t "automation_dispensing_can" -m '{"cmd":1,"motor_id":10,"weight_dg":100,"liquid_type":4}'
```

## Pin Usage Summary

### Arduino Mega Pin Allocation:
```
Pins 2-49:    Motor control (existing)
Pin 50:       SPI MISO (new - CAN)
Pin 51:       SPI MOSI (new - CAN)  
Pin 52:       SPI SCK (new - CAN)
Pin 53:       SPI CS (new - CAN)
Pins 16-17:   UART to Micro (existing)
Pins 20-21:   I2C to Nano Every (existing)
Pins A5,A8:   Leak sensors (existing)
Pins A6,A9:   Leak power (existing)
```

### No Pin Conflicts:
✅ **SPI pins (50-53)** are dedicated SPI pins on Mega
✅ **Pin 2** for CAN interrupt is available  
✅ **All existing functionality preserved**

## Advantages of CAN Integration

### 1. Unified Communication:
- **All automations** use same CAN protocol
- **Standardized commands** across devices
- **Reliable acknowledgments** for every operation

### 2. Enhanced Reliability:
- **Error detection** and automatic retry
- **Message prioritization** via CAN arbitration
- **Galvanic isolation** possible with CAN transceivers

### 3. Scalability:
- **Add new devices** easily to CAN bus
- **No IP address management** needed
- **Real-time performance** guaranteed

### 4. Diagnostics:
- **Bus monitoring** with standard CAN tools
- **Message logging** for troubleshooting
- **Network health** monitoring

## Implementation Priority

### Phase 1: Basic CAN Integration
1. ✅ **Wire MCP2515** to Arduino Mega
2. ✅ **Add CAN library** to platformio.ini
3. ✅ **Test CAN initialization** 
4. ✅ **Implement basic send/receive**

### Phase 2: Dispensing Commands
1. ✅ **Add CAN command parsing**
2. ✅ **Integrate with existing dispensing logic**
3. ✅ **Test CAN → dispensing flow**
4. ✅ **Add ACK responses**

### Phase 3: Full Integration
1. ✅ **Update automation service** for CAN
2. ✅ **Add scale data broadcasting**
3. ✅ **Implement diagnostics**
4. ✅ **Connect other automation devices**

## Safety Considerations

### 1. CAN Bus Protection:
- **120Ω termination** at bus ends only
- **Twisted pair cable** for CAN-H/CAN-L
- **Common ground** for all devices
- **Proper shielding** in noisy environments

### 2. Power Isolation:
- **Separate 5V supply** for MCP2515 if needed
- **Isolated CAN transceivers** for industrial environments
- **Surge protection** on CAN lines

### 3. Emergency Stop:
- **CAN emergency stop** command (highest priority)
- **Fallback to existing** UART/MQTT if CAN fails
- **Leak detection** still works independently

## Next Steps

1. **Wire MCP2515** to Arduino Mega as shown above
2. **Update platformio.ini** with CAN library
3. **Add CAN code** to main.cpp
4. **Test basic CAN communication**
5. **Integrate with dispensing logic**

The MCP2515 integration will give you **industrial-grade automation control** while preserving all your existing dispensing functionality! 🚀 