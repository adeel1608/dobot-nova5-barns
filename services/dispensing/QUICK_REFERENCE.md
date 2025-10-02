# Motor Control Quick Reference

## Pin Configuration (Default)
```
Motor 0: PWM=D2,  DIR=D22, EN=D24
Motor 1: PWM=D3,  DIR=D26, EN=D28  
Motor 2: PWM=D4,  DIR=D30, EN=D32
Motor 3: PWM=D5,  DIR=D34, EN=D36
Motor 4: PWM=D6,  DIR=D38, EN=D40
Motor 5: PWM=D7,  DIR=D42, EN=D44
```
**⚠️ ADJUST THESE TO MATCH YOUR PCB!**

## Micro Serial Commands
```bash
M 0 150      # Motor 0 forward at speed 150
M 1 -100     # Motor 1 reverse at speed 100  
M 2 0        # Stop motor 2
STOP         # Stop all motors
HELP         # Show help
```

## Motor Test Commands
```bash
T 0 100 0    # Test motor 0, speed 100, forward
T 1 150 1    # Test motor 1, speed 150, reverse
S 0          # Stop motor 0
A            # Stop all motors
L            # List configuration
```

## Modbus Registers
```
0-1:  Scale A (weight, status)
2-3:  Scale B (weight, status)
4-9:  Motor control (6 motors)
10:   Motor status bits
```

## Motor Control Format
- **Forward**: 0x0064 (speed 100)
- **Reverse**: 0x8064 (speed 100)  
- **Stop**: 0x0000

## Testing Procedure
1. Upload `motor_test.cpp` to Mega
2. Test each motor individually
3. Verify pin assignments match PCB
4. Upload full system when confirmed

## Safety Notes
- All motors start STOPPED
- Emergency STOP available
- Speed limited to ±255
- Enable pins for complete shutdown 