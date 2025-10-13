# Dispensing Service

## Brief Overview

The Dispensing Service provides hardware integration and control for ingredient dispensing mechanisms including syrup pumps, milk dispensers, and other liquid/powder dispensing systems via CAN bus and MQTT protocols for precise ingredient delivery.

## Key Features

- **CAN Bus Integration**: Industrial-grade CAN protocol for motor control
- **MQTT Communication**: RabbitMQ MQTT plugin for Arduino/ESP32 devices
- **Multi-Dispenser Support**: Syrups, milk, water, powder dispensers
- **Precise Dosing**: Weight-based and volume-based dispensing
- **Motor Control**: PWM control for variable flow rates
- **Calibration System**: Auto-calibration for accuracy
- **Safety Interlocks**: Overflow prevention and timeout protection

## Architecture

```
┌──────────────────────────────────────────────┐
│         Dispensing Control Layer             │
│                                              │
│  ┌────────────────────────────────┐         │
│  │  MQTT Bridge                   │         │
│  │  (ESP32 Microcontroller)       │         │
│  │  - Subscribe: dispensing/cmd   │         │
│  │  - Publish: dispensing/status  │         │
│  └──────────┬─────────────────────┘         │
│             │                                │
│             ↓                                │
│  ┌────────────────────────────────┐         │
│  │  CAN Bus Controller            │         │
│  │  - MCP2515 CAN Module          │         │
│  │  - 250kbps baud rate           │         │
│  └──────────┬─────────────────────┘         │
│             │                                │
└─────────────┼────────────────────────────────┘
              │
       CAN Bus (Physical)
              │
    ┌─────────┼─────────┐
    ↓         ↓         ↓
┌────────┐ ┌───────┐ ┌────────┐
│Syrup   │ │ Milk  │ │ Water  │
│Pumps   │ │Disp.  │ │Disp.   │
│1-14    │ │15-18  │ │19-23   │
└────────┘ └───────┘ └────────┘
```

## Hardware Components

### Supported Dispensers

1. **Syrup Dispensers** (Pumps 1-14)
   - Peristaltic pumps
   - 0-100ml/min flow rate
   - ±2% accuracy

2. **Milk Dispensers** (Pumps 15-18)
   - Refrigerated reservoirs
   - Temperature monitoring
   - Types: whole, skim, oat, almond

3. **Water Dispensers** (Pumps 19-23)
   - Hot water (92°C)
   - Cold water (4°C)
   - Filtered water line

4. **Powder Dispensers** (Optional)
   - Auger-based delivery
   - Chocolate, matcha, protein powders

## Setup & Installation

### Prerequisites

- ESP32 development board
- MCP2515 CAN module
- Motor drivers (L298N or similar)
- Power supply (12V/24V depending on pumps)
- PlatformIO or Arduino IDE

### Hardware Wiring

See `WIRING.md` for detailed wiring diagrams.

**Basic Connections:**
```
ESP32 Pin    →  MCP2515 CAN Module
GPIO 5 (CS)  →  CS
GPIO 18 (SCK) →  SCK  
GPIO 19 (MISO) → SO
GPIO 23 (MOSI) → SI
GND          →  GND
3.3V         →  VCC
```

### Firmware Upload

```bash
cd services/dispensing

# Using PlatformIO
pio run --target upload

# Or using Arduino IDE
# Open src/mqtt_can_bridge.cpp
# Select ESP32 board
# Upload
```

### MQTT Configuration

```cpp
// In src/mqtt_can_bridge.cpp
const char* mqtt_server = "192.168.1.100";  // RabbitMQ host
const int mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_pass = "admin123";
```

## CAN Bus Protocol

### Message Format

**Standard CAN Frame:**
- ID: 11-bit identifier
- DLC: Data length (0-8 bytes)
- Data: Command payload

### Dispenser Commands

#### Dispense Command (ID: 0x100)

```
Byte 0: Pump number (1-23)
Byte 1: Amount high byte
Byte 2: Amount low byte
Byte 3: Speed (0-255)
Byte 4-7: Reserved
```

**Example: Dispense 50ml from pump 3 at 80% speed**
```
ID: 0x100
Data: [03 00 32 CC 00 00 00 00]
```

#### Status Request (ID: 0x200)

```
Byte 0: Pump number
Byte 1-7: Reserved
```

#### Calibration Command (ID: 0x300)

```
Byte 0: Pump number
Byte 1: Calibration mode (0=zero, 1=span)
Byte 2-7: Reserved
```

### Response Messages

#### Dispense Complete (ID: 0x101)

```
Byte 0: Pump number
Byte 1: Status (0=success, 1=error)
Byte 2: Actual amount high
Byte 3: Actual amount low
Byte 4-7: Reserved
```

## MQTT Integration

### Topics

- **Command**: `dispensing/command`
- **Status**: `dispensing/status`
- **Telemetry**: `dispensing/telemetry`

### Message Format

#### Dispense Command (JSON)

```json
{
  "pump": 3,
  "amount_ml": 50,
  "speed": 80
}
```

#### Status Response

```json
{
  "pump": 3,
  "status": "completed",
  "actual_amount": 49.8,
  "duration_ms": 3200
}
```

## Calibration

### Auto-Calibration Procedure

1. **Zero Calibration**: Establish baseline
2. **Span Calibration**: Calibrate full range
3. **Verification**: Test dispense accuracy

```bash
# Via MQTT
mosquitto_pub -h localhost -t "dispensing/command" \
  -m '{"pump":3,"command":"calibrate","mode":"zero"}'

# Wait 5 seconds

mosquitto_pub -h localhost -t "dispensing/command" \
  -m '{"pump":3,"command":"calibrate","mode":"span","volume":100}'
```

## Usage Examples

### From Automation Service

```python
# In automation_functions.py
async def dispense_syrup(params: dict):
    pump_number = params.get("pump_number", 9)
    amount = params.get("amount", 20)
    
    payload = json.dumps({
        "pump": pump_number,
        "amount_ml": amount,
        "speed": 80
    })
    
    client.publish("dispensing/command", payload)
    
    # Wait for completion
    response = await wait_for_response("dispensing/status", timeout=30)
    
    return {
        "success": response["status"] == "completed",
        "actual_amount": response["actual_amount"]
    }
```

### Direct MQTT Control

```bash
# Dispense 30ml from syrup pump 5
mosquitto_pub -h 192.168.1.100 -p 1883 \
  -u admin -P admin123 \
  -t "dispensing/command" \
  -m '{"pump":5,"amount_ml":30,"speed":90}'

# Monitor status
mosquitto_sub -h 192.168.1.100 -p 1883 \
  -u admin -P admin123 \
  -t "dispensing/status"
```

## Configuration Files

### platformio.ini

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    mcp_can
    PubSubClient
    ArduinoJson
monitor_speed = 115200
```

## Troubleshooting

### CAN Bus Not Responding

1. Check wiring and termination resistors (120Ω at each end)
2. Verify baud rate matches (250kbps standard)
3. Use CAN bus analyzer or oscilloscope to check signals
4. Check power supply to CAN transceivers

```cpp
// In code, verify CAN initialization
if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
  Serial.println("CAN Init OK");
} else {
  Serial.println("CAN Init Failed");
}
```

### MQTT Connection Failed

```bash
# Test MQTT broker
mosquitto_sub -h localhost -p 1883 -u admin -P admin123 -t "#" -v

# Check ESP32 serial output
pio device monitor
```

### Inaccurate Dispensing

1. Run calibration procedure
2. Check pump tubing for air bubbles
3. Verify supply reservoir has sufficient liquid
4. Clean pump heads (peristaltic pumps need periodic maintenance)

### Motor Not Running

1. Check motor driver connections
2. Verify power supply voltage and current capacity
3. Test motor directly with power supply
4. Check PWM signal with oscilloscope

## Safety Features

- **Timeout Protection**: Auto-stop after 30s of continuous operation
- **Overflow Detection**: Stop if weight exceeds expected
- **Temperature Monitoring**: Shutdown if reservoir temp out of range
- **Emergency Stop**: MQTT command to halt all dispensers

## Maintenance

### Regular Maintenance

- **Daily**: Visual inspection of reservoir levels
- **Weekly**: Clean external surfaces, check for leaks
- **Monthly**: Calibrate high-use dispensers, replace tubing if worn
- **Quarterly**: Deep clean pump heads, inspect electrical connections

### Troubleshooting Guide

See `QUICK_REFERENCE.md` for command reference and common issues.

## Documentation

- `README.md`: This file
- `CAN_INTEGRATION_GUIDE.md`: Detailed CAN bus setup
- `MOTOR_CONTROL_README.md`: Motor driver configuration
- `WIRING.md`: Hardware wiring diagrams
- `QUICK_REFERENCE.md`: Command reference
- `DISPENSING_INTEGRATION_PROCESS.md`: Integration steps

## Dependencies

- **mcp_can**: CAN bus library for MCP2515
- **PubSubClient**: MQTT client for ESP32
- **ArduinoJson**: JSON parsing

## Future Enhancements

- Web-based calibration interface
- Machine learning for predictive maintenance
- Multi-language dispensing profiles
- Integration with scales for weight-based dispensing
- Remote firmware updates (OTA)
- Advanced diagnostics and telemetry
