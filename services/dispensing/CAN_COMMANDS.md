# Dispenser CAN Command Cheat‑Sheet (USB‑CAN → Arduino Micro → Mega)

## Interface setup (other laptop)
```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

Monitor (keep running in another terminal):
```bash
candump can0 -td
```

## Frame format (ID 0x110)
- ID: `0x110`
- DLC: 8
- Payload bytes: `[cmd] [motor_id] [weight_dg_low] [weight_dg_high] [liquid_type] [ext0] [ext1] [ext2]`
  - `cmd`: `0x01` = dispense, `0x02` = stop, `0x03` = set_lag
  - `motor_id`: 1..24 (see map below)
  - `weight_dg`: target weight in decigrams (for `dispense`) or lag in decigrams (for `set_lag`), little‑endian
  - `liquid_type`: `1` water, `2` milk, `3` sauce, `4` caramel, `5` syrup, `6` honey (ignored for `set_lag`)
  - `ext0`: for `set_lag`, speed profile: `0` (SPEED 0/faster), `1` (SPEED 1/slower)

Send (no spaces in data):
```bash
cansend can0 110#01MMLLHHTT000000
# dotted equivalent:
cansend can0 110#01.MM.LL.HH.TT.00.00.00
```

ACK: Comes back on ID `0x111` with byte0 = `0x01`.

## Motor ID map
- Milk:
  - `milk1..milk8` → `0x01..0x08`
- Sauces:
  - `sauce1..sauce15` → `0x09..0x17`
- Rinser:
  - `rinser` → `0x18`

| Line      | motor_id (hex) |
|-----------|-----------------|
| milk1     | 01 |
| milk2     | 02 |
| milk3     | 03 |
| milk4     | 04 |
| milk5     | 05 |
| milk6     | 06 |
| milk7     | 07 |
| milk8     | 08 |
| sauce1    | 09 |
| sauce2    | 0A |
| sauce3    | 0B |
| sauce4    | 0C |
| sauce5    | 0D |
| sauce6    | 0E |
| sauce7    | 0F |
| sauce8    | 10 |
| sauce9    | 11 |
| sauce10   | 12 |
| sauce11   | 13 |
| sauce12   | 14 |
| sauce13   | 15 |
| sauce14   | 16 |
| sauce15   | 17 |
| rinser    | 18 |

### Motor name → CAN ID → Mega pin
| Motor | CAN motor_id (hex) | Mega pin |
|-------|---------------------|----------|
| milk1 | 01 | D2 |
| milk2 | 02 | D3 |
| milk3 | 03 | D4 |
| milk4 | 04 | D5 |
| milk5 | 05 | D6 |
| milk6 | 06 | D7 |
| milk7 | 07 | D8 |
| milk8 | 08 | D9 |
| sauce1 | 09 | D10 |
| sauce2 | 0A | D11 |
| sauce3 | 0B | D12 |
| sauce4 | 0C | D25 |
| sauce5 | 0D | D27 |
| sauce6 | 0E | D29 |
| sauce7 | 0F | D31 |
| sauce8 | 10 | D33 |
| sauce9 | 11 | D35 |
| sauce10 | 12 | D37 |
| sauce11 | 13 | D39 |
| sauce12 | 14 | D41 |
| sauce13 | 15 | D43 |
| sauce14 | 16 | D45 |
| sauce15 | 17 | D47 |
| rinser | 18 | D49 |

## Ready examples

### Milk 50 g (milk1..milk8)
- 50 g → 500 dg → `0x01F4` → `LL=F4`, `HH=01`, liquid=`02`
```bash
cansend can0 110#0101F40102000000  # milk1
cansend can0 110#0102F40102000000  # milk2
cansend can0 110#0103F40102000000  # milk3
cansend can0 110#0104F40102000000  # milk4
cansend can0 110#0105F40102000000  # milk5
cansend can0 110#0106F40102000000  # milk6
cansend can0 110#0107F40102000000  # milk7
cansend can0 110#0108F40102000000  # milk8
```

### Caramel 10 g (sauce1..sauce15)
- 10 g → 100 dg → `0x0064` → `LL=64`, `HH=00`, liquid=`04`
```bash
cansend can0 110#0109640004000000  # sauce1
cansend can0 110#010A640004000000  # sauce2
cansend can0 110#010B640004000000  # sauce3
cansend can0 110#010C640004000000  # sauce4
cansend can0 110#010D640004000000  # sauce5
cansend can0 110#010E640004000000  # sauce6
cansend can0 110#010F640004000000  # sauce7
cansend can0 110#0110640004000000  # sauce8
cansend can0 110#0111640004000000  # sauce9
cansend can0 110#0112640004000000  # sauce10
cansend can0 110#0113640004000000  # sauce11
cansend can0 110#0114640004000000  # sauce12
cansend can0 110#0115640004000000  # sauce13
cansend can0 110#0116640004000000  # sauce14
cansend can0 110#0117640004000000  # sauce15
```

### Set per‑motor lag (new)
- Set `sauce9` lag to 22.0 g on SPEED 1 (slower profile)
  - motor_id: `sauce9` → 0x11
  - lag 22.0 g → 220 dg → `0x00DC` → `LL=DC`, `HH=00`
  - speed=1 → `ext0=01`
```bash
cansend can0 110#0311DC0000010000
# bytes: cmd=03 motor=11 LL=DC HH=00 type=00 ext0=01 ext1=00 ext2=00
```
- Clear override by setting lag to 0 (falls back to liquid lag):
```bash
cansend can0 110#0311000000000000
```

## Parametric sender (bash helper)
```bash
# usage: send_can <motor_id_dec> <grams> <liquid_dec>
send_can() {
  mid_hex=$(printf "%02X" "$1")
  dg=$(( ${2%.*} * 10 ))
  ll=$(printf "%02X" $(( dg & 0xFF )))
  hh=$(printf "%02X" $(( (dg >> 8) & 0xFF )))
  lt_hex=$(printf "%02X" "$3")
  cansend can0 110#01${mid_hex}${ll}${hh}${lt_hex}000000
}
# examples:
# send_can 9 10 4     # sauce1, 10g, caramel
# send_can 1 50 2     # milk1, 50g, milk

# usage: set_lag_can <motor_id_dec> <lag_g> <speed01>
set_lag_can() {
  mid_hex=$(printf "%02X" "$1")
  dg=$(( ${2%.*} * 10 ))
  ll=$(printf "%02X" $(( dg & 0xFF )))
  hh=$(printf "%02X" $(( (dg >> 8) & 0xFF )))
  sp=$(printf "%02X" "$3")
  # cmd=03 (set_lag), liquid=00, ext0=speed, ext1=00, ext2=00
  cansend can0 110#03${mid_hex}${ll}${hh}00${sp}0000
}
# examples:
# set_lag_can 17 22 1   # sauce15 lag=22g on SPEED 1
# set_lag_can 11 0 0    # sauce9 clear override (lag=0) on SPEED 0
```

## MQTT usage (via Arduino Micro bridge)
- Dispense by name (bridge converts to CAN and forwards to Mega):
```bash
mosquitto_pub -h 192.168.200.233 -t automation_dispensing -m '{"ingredient":"caramel_syrup","weight":10,"motor":"sauce15"}'
```
- Set per‑motor lag override (affects stop‑early compensation):
  - `speed`: 0 = faster profile, 1 = slower profile
  - `lag`: grams (float). Use 0 to clear override and fall back to liquid lag.
```bash
mosquitto_pub -h 192.168.200.233 -t automation_dispensing_lag -m '{"motor":"sauce15","speed":1,"lag":22.0}'
mosquitto_pub -h 192.168.200.233 -t automation_dispensing_lag -m '{"motor":"milk3","speed":0,"lag":9.5}'
```
Notes:
- Overrides live in RAM; they reset on power cycle.
- The device’s active speed profile (SPEED 0/1) is selected on the Mega (USB serial `SPEED 0` or `SPEED 1`). The override you set for that profile is used automatically.

## CAN usage (raw SocketCAN)
- Dispense: `cmd=0x01`, payload carries weight in decigrams and liquid type.
- Set lag: `cmd=0x03`, payload carries lag in decigrams; `ext0` = speed (0|1).

Examples
- Sauce15 (id=0x17) 10 g caramel (type=0x04):
```bash
cansend can0 110#0117640004000000
```
- Set lag 22.0 g for sauce15 on speed=1:
```bash
cansend can0 110#0317DC0000010000
```
- Clear lag override (fallback to liquid lag):
```bash
cansend can0 110#0317000000000000
```

Helper functions
```bash
# usage: send_can <motor_id_dec> <grams> <liquid_dec>
send_can() {
  mid_hex=$(printf "%02X" "$1")
  dg=$(( ${2%.*} * 10 ))
  ll=$(printf "%02X" $(( dg & 0xFF )))
  hh=$(printf "%02X" $(( (dg >> 8) & 0xFF )))
  lt_hex=$(printf "%02X" "$3")
  cansend can0 110#01${mid_hex}${ll}${hh}${lt_hex}000000
}
# usage: set_lag_can <motor_id_dec> <lag_g> <speed01>
set_lag_can() {
  mid_hex=$(printf "%02X" "$1")
  dg=$(( ${2%.*} * 10 ))
  ll=$(printf "%02X" $(( dg & 0xFF )))
  hh=$(printf "%02X" $(( (dg >> 8) & 0xFF )))
  sp=$(printf "%02X" "$3")
  cansend can0 110#03${mid_hex}${ll}${hh}00${sp}0000
}
# examples:
# send_can 23 10 4     # sauce15 (0x17), 10 g, caramel
# set_lag_can 23 22 1  # sauce15 lag=22 g, speed=1
```

## Notes / Troubleshooting
- Bitrate: 500 kbit/s.
- MCP2515 crystal on Micro side: 8 MHz (firmware configured accordingly).
- Exactly two 120 Ω terminators on the CAN bus. Common ground required.
- ACK appears on ID `0x111` (byte0=`0x01`). If no ACK, check wiring/termination/bitrate/crystal. 