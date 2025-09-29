# BARNS Dispensing System Wiring

This document lists the physical wiring between the controller boards and PCBs used in the dispensing stack (Arduino Mega 2560 + Arduino Micro bridge + MCP2515 CAN + W5500 Ethernet + leak sensors + scales + motor control PCB).

- Boards: Arduino Mega 2560 (main controller), Arduino Micro (MQTT+CAN bridge)
- Buses: UART bridge (Mega Serial2 ↔ Micro Serial1), CAN bus (Micro MCP2515), I²C scales (Mega ↔ two scale slaves)
- Power: Ensure common ground across all boards and modules. Provide stable 5V where required.

## 1) Arduino Mega 2560 (Main Dispenser Controller)

- UART2 bridge to Micro:
  - Mega TX2 (D16) → Micro RX1 (Serial1 RX)
  - Mega RX2 (D17) ← Micro TX1 (Serial1 TX)
  - Mega GND ↔ Micro GND
  - Baud: 115200
- I²C to scales (two slaves):
  - Mega SDA (D20) ↔ I²C bus SDA
  - Mega SCL (D21) ↔ I²C bus SCL
  - 5V and GND to scales
  - I²C slave addresses: 0x28 (milk scale), 0x29 (sauce scale)
- Leak sensors:
  - Sensor 1 signal → A5
  - Sensor 1 power → A6 (driven HIGH to power the sensor)
  - Sensor 2 signal → A8
  - Sensor 2 power → A9 (driven HIGH to power the sensor)
  - Common ground to sensors
- Global speed control:
  - Speed enable line → D23 (shared with all motors)
- Motor control outputs (to motor driver PCB inputs):
  - milk1 → D2
  - milk2 → D3
  - milk3 → D4
  - milk4 → D5
  - milk5 → D6
  - milk6 → D7
  - milk7 → D8
  - milk8 → D9
  - sauce1 → D10
  - sauce2 → D11
  - sauce3 → D12
  - sauce4 → D25
  - sauce5 → D27
  - sauce6 → D29
  - sauce7 → D31
  - sauce8 → D33
  - sauce9 → D35
  - sauce10 → D37
  - sauce11 → D39
  - sauce12 → D41
  - sauce13 → D43
  - sauce14 → D45
  - sauce15 → D47
  - rinser → D49

Notes:
- Each motor control output switches a corresponding motor/valve channel on the motor driver PCB. Keep grounds common between Mega and the driver board.
- Leak detection threshold is currently disabled in code (LEAK_THRESHOLD=0). Sensors are still wired as above.

## 2) Arduino Micro (MQTT + CAN Bridge)

- UART to Mega:
  - Micro TX1 (Serial1 TX) → Mega RX2 (D17)
  - Micro RX1 (Serial1 RX) ← Mega TX2 (D16)
  - Micro GND ↔ Mega GND
  - Baud: 115200
- Ethernet (W5500):
  - CS → D10
  - RST → D9
  - SCK/MOSI/MISO → Micro hardware SPI pins (ICSP header)
  - 3.3V/5V and GND per your W5500 module (many breakout boards accept 5V via onboard regulator; verify your module)
  - Static IP configured in firmware: 192.168.200.211 (/24), GW 192.168.200.1
- CAN (MCP2515 + transceiver):
  - CS → D4
  - INT → D2 (optional, library can poll)
  - SCK/MOSI/MISO → Micro hardware SPI pins (ICSP header)
  - VCC and GND per your MCP2515 board (most common modules are 5V with onboard 5V↔3.3V level shifting; verify yours)
  - CANH/CANL → CAN bus twisted pair

Notes:
- The bridge subscribes to MQTT topics (e.g., `automation_dispensing`, `automation_dispensing_can`) and forwards commands to the Mega over UART2, and to CAN (ID 0x110). Ensure the Micro is on the same network as your broker (192.168.200.233:1883 by default in firmware).

## 3) CAN Bus (between Micro bridge and any CAN nodes)

- Bitrate: 500 kbit/s
- MCP2515 crystal: 8 MHz (configured in firmware)
- Bus wiring:
  - CANH ↔ CANH, CANL ↔ CANL across all CAN devices
  - Exactly two 120 Ω terminators at both ends of the CAN trunk
  - Common ground between CAN transceivers and controller grounds
- Dispenser command frames use ID 0x110 (see `CAN_COMMANDS.md` for payload layout and examples). ACKs on 0x111.

## 4) Scales (two I²C slaves)

- Addresses:
  - Milk scale: 0x28
  - Sauce scale: 0x29
- Wiring to Mega:
  - SDA ↔ Mega SDA (D20)
  - SCL ↔ Mega SCL (D21)
  - 5V and GND
- Place scales and load cells per your mechanical setup. Keep I²C wiring short and clean. Add pull-ups if required (often integrated on breakout boards).

## 5) Power & Grounding

- Provide stable 5V to Mega, Micro, motor driver PCB logic, sensors, and modules. Some modules (W5500) may require 3.3V—verify your exact boards.
- Always share a common ground between Mega, Micro, motor drivers, leak sensors, scales, and CAN transceivers.
- For motors/valves, use a separate power rail sized for peak current. Ensure proper flyback protection and that control signals reference the same ground.

## 6) Quick Verification Checklist

- Mega ↔ Micro UART: D16↔RX1, D17↔TX1, GND common
- Micro W5500: CS=D10, RST=D9, SPI on ICSP; link up, IP=192.168.200.211
- Micro MCP2515: CS=D4, INT=D2, SPI on ICSP; CANH/CANL wired, two 120 Ω terminators
- Mega I²C: SDA=D20, SCL=D21; devices respond at 0x28 and 0x29
- Leak sensors: A5/A8 signal, A6/A9 power HIGH; readings change when wet
- Motor lines: milk1..8 on D2..D9; sauce1..15 on D10,11,12,25,27,29,31,33,35,37,39,41,43,45,47; rinser on D49; speed line D23

## 7) Reference Files

- `services/dispensing/src/main.cpp` (motor pins, leak sensors, UART2 config, I²C addresses)
- `services/dispensing/src/mqtt_can_bridge.cpp` (W5500, MCP2515, MQTT topics, CAN IDs, Serial1 to Mega)
- `services/dispensing/CAN_COMMANDS.md` (CAN frame format and examples) 