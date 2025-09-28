# Arduino Mega Motor Control System

## Overview

This enhanced system adds motor control capabilities to your existing Arduino Mega scale monitoring setup. The system maintains all original functionality (I2C scale reading, Modbus communication) while adding the ability to control multiple motors via Modbus commands.

## System Architecture

```
[Arduino Micro] ←→ [RS485] ←→ [Arduino Mega] ←→ [I2C] ←→ [Nano Scales]
     (Master)                    (Slave + Motor Controller)
```

- **Arduino Micro**: Modbus master, sends motor control commands
- **Arduino Mega**: Modbus slave, controls motors and reads scales
- **Nano Every**: I2C scale controllers (unchanged)

## Hardware Connections

### Arduino Mega Pin Assignments

**Existing connections (unchanged):**
- A2, A3: RS485 control (RE_PIN, DE_PIN)
- Serial3 (pins 14, 15): Modbus communication
- I2C (pins 20, 21): Communication with Nano scales

**New motor control pins:**
```cpp
// Default configuration - adjust to match your PCB
Motor 0: PWM=D2,  DIR=D22, EN=D24
Motor 1: PWM=D3,  DIR=D26, EN=D28
Motor 2: PWM=D4,  DIR=D30, EN=D32
Motor 3: PWM=D5,  DIR=D34, EN=D36
Motor 4: PWM=D6,  DIR=D38, EN=D40
Motor 5: PWM=D7,  DIR=D42, EN=D44
```

### Motor Driver Connections

For each motor (using TB6612, L298N, or similar):
- **PWM pin** → Driver's PWM/Speed input
- **DIR pin** → Driver's Direction input
- **EN pin** → Driver's Enable input (optional)
- **Motor power** → External power supply (share GND with Arduino)

## Software Components

### 1. Enhanced Mega Firmware (`main.cpp`)

**Key features:**
- Maintains original scale reading and Modbus functionality
- Adds 6 motor control channels
- Extended Modbus register map
- Supports both read (0x03) and write (0x06) Modbus functions

**Register Map:**
```
Register 0-1: Scale A (weight, status)
Register 2-3: Scale B (weight, status)  
Register 4-9: Motor control values (6 motors)
Register 10:  Motor status bits
```

**Motor Control Format:**
- Bit 15: Direction (0=forward, 1=reverse)
- Bits 0-7: Speed (0-255)
- Example: 0x0064 = forward at speed 100
- Example: 0x8064 = reverse at speed 100

### 2. Enhanced Micro Firmware (`micro.cpp`)

**Key features:**
- Maintains original scale monitoring
- Adds motor control via serial commands
- Implements Modbus write operations
- Real-time motor status display

**Serial Commands:**
```
M <motor_idx> <speed>  - Set motor speed (-255 to +255)
STOP                   - Stop all motors
HELP                   - Show command help
```

**Examples:**
```
M 0 100     - Motor 0 forward at speed 100
M 1 -150    - Motor 1 reverse at speed 150
M 2 0       - Stop motor 2
STOP        - Stop all motors
```

### 3. Motor Test Program (`motor_test.cpp`)

A standalone test program for verifying motor connections before deploying the full system.

**Commands:**
```
T <motor> <speed> <dir>  - Test motor (speed 0-255, dir 0/1)
S <motor>                - Stop specific motor
A                        - Stop all motors
L                        - List motor configuration
H                        - Show help
```

## Setup Instructions

### Step 1: Configure Pin Mapping

1. Open `main.cpp` and locate the `MOTORS[]` array
2. Adjust pin assignments to match your PCB:
```cpp
static MotorChannel MOTORS[] = {
  // {PWM_PIN, DIR_PIN, EN_PIN, active, speed}
  {2,  22, 24, false, 0},  // Motor 0
  {3,  26, 28, false, 0},  // Motor 1
  // ... add more motors as needed
};
```

### Step 2: Test Hardware Connections

1. Upload `motor_test.cpp` to your Mega
2. Open Serial Monitor at 115200 baud
3. Use test commands to verify each motor:
```
T 0 100 0    - Test motor 0, speed 100, forward
T 0 100 1    - Test motor 0, speed 100, reverse
S 0          - Stop motor 0
```

### Step 3: Deploy Full System

1. Upload enhanced `main.cpp` to Mega
2. Upload enhanced `micro.cpp` to Micro
3. Verify Modbus communication works
4. Test motor control via Micro serial commands

## Usage Examples

### Basic Motor Control from Micro

1. Connect to Micro's USB serial at 115200 baud
2. Send commands:
```
HELP         - Show available commands
M 0 150      - Start motor 0 forward at speed 150
M 1 -100     - Start motor 1 reverse at speed 100
STOP         - Stop all motors
```

### Modbus Register Access

**Reading motor status:**
```
Function: 0x03 (Read Holding Registers)
Address:  10 (motor status register)
Quantity: 1
Response: 16-bit value with motor status bits
```

**Controlling motor:**
```
Function: 0x06 (Write Single Register)  
Address:  4 (for motor 0, 5 for motor 1, etc.)
Value:    0x0064 (forward speed 100) or 0x8064 (reverse speed 100)
```

## Safety Features

1. **Motor initialization**: All motors start disabled/stopped
2. **Speed limiting**: Motor speeds constrained to valid range (-255 to +255)
3. **Enable pin control**: Motors can be completely disabled via enable pins
4. **Status monitoring**: Real-time motor status available via Modbus
5. **Emergency stop**: `STOP` command immediately stops all motors

## Troubleshooting

### Motor doesn't move
1. Check power supply connections
2. Verify pin assignments match PCB
3. Test with `motor_test.cpp` program
4. Check enable pin wiring (if used)

### Modbus communication issues
1. Verify RS485 wiring (A+, B-, GND)
2. Check baud rate settings (9600)
3. Confirm slave ID matches (default: 3)
4. Test with original scale reading first

### Serial commands not working
1. Check USB connection to Micro
2. Verify baud rate (115200)
3. Ensure proper line endings in serial terminal

## Customization

### Adding More Motors

1. Add entries to `MOTORS[]` array in `main.cpp`
2. Update `NUM_MOTORS` constant in `micro.cpp`
3. Ensure PWM pins are used for speed control
4. Test with `motor_test.cpp` first

### Changing Pin Assignments

1. Update `MOTORS[]` array with your pin numbers
2. Avoid pins already used by existing system:
   - A2, A3 (RS485)
   - 14, 15 (Serial3)
   - 20, 21 (I2C)

### Motor Driver Compatibility

The system works with most common motor drivers:
- **TB6612FNG**: PWM + DIR + STBY pins
- **L298N**: PWM + IN1/IN2 (use DIR for IN1, PWM for IN2)  
- **DRV8833**: PWM + DIR + nSLEEP pins
- **Single MOSFET**: PWM only (set EN_PIN to 255)

## Performance Notes

- **PWM frequency**: Default Arduino PWM (~490Hz on most pins)
- **Modbus polling**: 200ms interval for scales, 500ms for motor status
- **Motor response**: Near-instantaneous via hardware PWM
- **Maximum motors**: Limited by available PWM pins (up to 12 on Mega)

## Future Enhancements

1. **Encoder feedback**: Add position/speed feedback
2. **PID control**: Implement closed-loop speed control  
3. **Acceleration profiles**: Smooth motor start/stop
4. **Web interface**: Control motors via WiFi module
5. **Data logging**: Log motor usage and scale data

---

*This system maintains full backward compatibility with your existing scale monitoring setup while adding powerful motor control capabilities.* 