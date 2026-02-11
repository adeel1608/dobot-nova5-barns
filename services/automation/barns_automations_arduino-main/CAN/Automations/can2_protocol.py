#!/usr/bin/env python3
# requirements: paho-mqtt, python-can
# pip install paho-mqtt python-can

import json
import logging
import sys
import time
import asyncio
from typing import Dict, Any
from concurrent.futures import ThreadPoolExecutor

import can
from paho.mqtt import client as mqtt

# ==== CONFIG (edit these as needed) ===========================================
MQTT_HOST = "192.168.200.254"
MQTT_PORT = 1883
MQTT_USERNAME = "admin"
MQTT_PASSWORD = "admin123"

TOPIC_MILK = "automation_milk"
TOPIC_SLUSH = "automation_slush"
TOPIC_GRINDER = "automation_grinding"
TOPIC_TAMPER = "automation_tampering"
TOPIC_ICE = "automation_ice_can_old"
TOPIC_SYRUP = "automation_syrup"
TOPIC_RINSER = "automation_rinser"
TOPIC_STATUS_MILK = "automation_milk_status"
TOPIC_STATUS_SYRUP = "automation_syrup_status"
TOPIC_CLEAR_MILK = "automation_milk_clear"
TOPIC_CLEAR_SYRUP = "automation_syrup_clear"
TOPIC_FROTHER_INIT = "automation_frother_init"
TOPIC_FROTHER = "automation_frother"
TOPIC_CLEAN_FROTHER = "automation_clean_frother"
TOPIC_RESPONSE = "automation/response"
TOPIC_HEALTH_CHECK = "automation_health"

# CAN interface (Linux socketcan). Ensure `can0` is up: e.g.
# sudo ip link set can0 up type can bitrate 500000
CAN_CHANNEL = "can0"
CAN_INTERFACE = "socketcan"

# Fixed CAN IDs
# NOTE: Modbus Master (sketch_jan24a) listens on 0x120 and responds on 0x121
CAN_ID_DISPENSER = 0x120       # Modbus Master command ID (for milk/syrup dispense)
CAN_ID_DISPENSER_ACK = 0x121   # Modbus Master response ID
CAN_ID_MILK_1 = 0x105  # For milk types 1-4 (also used by ICE below) - OLD SYSTEM
CAN_ID_MILK_2 = 0x104  # For milk types 5-8 - OLD SYSTEM
CAN_ID_SLUSH = 0x102
CAN_ID_GRINDER = 0x101
CAN_ID_TAMPER = 0x103
CAN_ID_ICE = 0x106      #106
CAN_ID_FROTHER_CMD = 0x320     # Frother command ID
CAN_ID_FROTHER_EVENT = 0x321   # Frother event/response ID
CAN_ID_FROTHER_DISPENSER_HEARTBEAT = 0x3FF # Frother dispenser heartbeat ID

# Payload layout (8 bytes total):
# [0]   : cmd/indicator/etc. (0x01 for ACK, 0xFF with next 0xFF for heartbeat)
# [1]   : type (water_type or syrup_type or shots_number or tampering or ice)
# [2-3] : amount (unsigned 16-bit, little-endian, e.g., milliliters)
# [4-7] : reserved (0x00)
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("mqtt-can-bridge")

# ----------------------------- CAN RX infra -----------------------------------
# We capture all CAN frames in a background listener thread via can.Notifier.
# - Heartbeats: log "<device-name> is alive"
# - ACKs (b0==0x01): routed into per-ID asyncio queues so waits are race-free.

# Per-ID ACK queues (filled by CAN listener; awaited by wait_for_can_response)
ACK_QUEUES: Dict[int, asyncio.Queue] = {}

# Health monitoring for devices
HEALTH_DEVICES = {
    CAN_ID_SLUSH: "slush",
    CAN_ID_GRINDER: "grinder", 
    CAN_ID_TAMPER: "tamper",
    CAN_ID_ICE: "ice",
    CAN_ID_FROTHER_DISPENSER_HEARTBEAT: "frother_dispenser"
}

# Health tracking data structure
# Each device tracks: {"heartbeats": [timestamps], "status": "ok"/"fail", "last_check": timestamp}
DEVICE_HEALTH: Dict[int, Dict[str, Any]] = {}
HEALTH_CHECK_INTERVAL = 72.0  # 1.2 minutes in seconds
HEARTBEAT_WINDOW = 72.0  # 1.2 minutes window for heartbeats
MIN_HEARTBEATS_REQUIRED = 2  # Minimum heartbeats required in window

def build_id_name_map() -> Dict[int, str]:
    # Combine names if multiple features share the same ID.
    pairs = [
        (CAN_ID_GRINDER, "grinder"),
        (CAN_ID_SLUSH,   "slush"),
        (CAN_ID_TAMPER,  "tamper"),
        (CAN_ID_DISPENSER, "modbus_master_cmd"),  # 0x120 - Modbus Master commands
        (CAN_ID_DISPENSER_ACK, "modbus_master_rsp"),  # 0x121 - Modbus Master responses
        (CAN_ID_MILK_1,  "milk_1-4"),
        (CAN_ID_MILK_2,  "milk_5-8"),
        (CAN_ID_ICE,     "ice"),
        (CAN_ID_FROTHER_CMD, "frother_cmd"),
        (CAN_ID_FROTHER_EVENT, "frother_event"),
        (CAN_ID_FROTHER_DISPENSER_HEARTBEAT, "frother_dispenser_heartbeat"),
    ]
    m: Dict[int, str] = {}
    for cid, name in pairs:
        if cid in m:
            m[cid] = f"{m[cid]}/{name}"
        else:
            m[cid] = name
    return m

class HeartbeatAndAckListener(can.Listener):
    def __init__(self, id_to_name: Dict[int, str], loop: asyncio.AbstractEventLoop):
        super().__init__()
        self.id_to_name = id_to_name
        self.loop = loop

    def on_message_received(self, msg: can.Message) -> None:
        try:
            if msg.is_extended_id:
                return
            arb = msg.arbitration_id
            data = bytes(msg.data) if msg.data is not None else b""

            # Heartbeat: FF FF ...
            if len(data) >= 2 and data[0] == 0xFF and data[1] == 0xFF:
                name = self.id_to_name.get(arb, f"0x{arb:03X}")
                log.info("%s is alive", name)
                
                # Record heartbeat for health monitoring
                record_heartbeat(arb)

            # ACK/Response detection:
            # Format 1: [0x01, ...] - Simple ACK (legacy devices)
            # Format 2: [cmd, result, error, status, ...] - Modbus Master response (0x121)
            #   where cmd = echo of command (e.g., 0x0A for DISPENSE)
            #   and result = 0 (RC_OK) or 1 (RC_FAIL)
            is_ack = False
            if len(data) >= 1 and data[0] == 0x01:
                is_ack = True  # Legacy ACK format
            elif arb == CAN_ID_DISPENSER_ACK and len(data) >= 2:
                is_ack = True  # Modbus Master response format
            
            if is_ack:
                q = ACK_QUEUES.get(arb)
                if q is not None:
                    # Deliver to asyncio queue from this thread
                    self.loop.call_soon_threadsafe(q.put_nowait, data)
        except Exception as e:
            log.error("CAN listener error: %s", e)

def clear_ack_queue(can_id: int) -> None:
    """Best-effort drain to avoid stale ACKs."""
    q = ACK_QUEUES.get(can_id)
    if not q:
        return
    try:
        while True:
            q.get_nowait()
    except asyncio.QueueEmpty:
        pass

def initialize_device_health() -> None:
    """Initialize health tracking for all monitored devices."""
    global DEVICE_HEALTH
    current_time = time.time()
    for device_id in HEALTH_DEVICES.keys():
        DEVICE_HEALTH[device_id] = {
            "heartbeats": [],
            "status": "fail",  # Start as fail until we get heartbeats
            "last_check": current_time
        }

def record_heartbeat(device_id: int) -> None:
    """Record a heartbeat for a device."""
    if device_id not in HEALTH_DEVICES:
        return
    
    current_time = time.time()
    
    # Initialize if not exists
    if device_id not in DEVICE_HEALTH:
        DEVICE_HEALTH[device_id] = {
            "heartbeats": [],
            "status": "fail",
            "last_check": current_time
        }
    
    # Add heartbeat timestamp
    DEVICE_HEALTH[device_id]["heartbeats"].append(current_time)
    
    # Clean old heartbeats (older than HEARTBEAT_WINDOW)
    cutoff_time = current_time - HEARTBEAT_WINDOW
    DEVICE_HEALTH[device_id]["heartbeats"] = [
        hb for hb in DEVICE_HEALTH[device_id]["heartbeats"] 
        if hb > cutoff_time
    ]

def evaluate_device_health(device_id: int) -> str:
    """Evaluate if a device is healthy based on recent heartbeats."""
    if device_id not in DEVICE_HEALTH:
        return "fail"
    
    current_time = time.time()
    heartbeats = DEVICE_HEALTH[device_id]["heartbeats"]
    
    # Count heartbeats in the last HEARTBEAT_WINDOW
    cutoff_time = current_time - HEARTBEAT_WINDOW
    recent_heartbeats = [hb for hb in heartbeats if hb > cutoff_time]
    
    if len(recent_heartbeats) >= MIN_HEARTBEATS_REQUIRED:
        return "ok"
    else:
        return "fail"

def get_health_status_json() -> str:
    """Get health status as JSON string for MQTT publishing."""
    health_data = {}
    current_time = time.time()
    
    for device_id, device_name in HEALTH_DEVICES.items():
        # Update health status
        DEVICE_HEALTH[device_id]["status"] = evaluate_device_health(device_id)
        DEVICE_HEALTH[device_id]["last_check"] = current_time
        
        health_data[device_name] = DEVICE_HEALTH[device_id]["status"]
    
    return json.dumps(health_data)

def send_health_check_mqtt(client: mqtt.Client) -> None:
    """Send health check status to MQTT."""
    health_json = get_health_status_json()
    client.publish(TOPIC_HEALTH_CHECK, health_json)
    log.info("Sent health check: %s", health_json)

async def wait_for_can_response(expected_id: int, timeout: float = 60.0) -> bool:
    """Wait for ACK/response from expected_id via per-ID queue (filled by listener)."""
    q = ACK_QUEUES.get(expected_id)
    if q is None:
        # Should not happen if initialized properly
        log.warning("No ACK queue for id=0x%03X", expected_id)
        await asyncio.sleep(timeout)
        return False
    try:
        data = await asyncio.wait_for(q.get(), timeout=timeout)
        log.info("Received CAN response: id=0x%03X data=%s", expected_id, data.hex())
        
        # For Modbus Master responses (0x121), check result byte
        if expected_id == CAN_ID_DISPENSER_ACK and len(data) >= 2:
            result = data[1]  # 0=RC_OK, 1=RC_FAIL
            if result == 0:
                log.info("Modbus Master: RC_OK (success)")
                return True
            else:
                error = data[2] if len(data) > 2 else 0
                status = data[3] if len(data) > 3 else 0
                log.warning("Modbus Master: RC_FAIL (error=%d, status=%d)", error, status)
                return False
        
        # Legacy ACK format (0x01) or other devices
        return True
    except asyncio.TimeoutError:
        log.warning("Timeout waiting for CAN response from id=0x%03X", expected_id)
        return False

async def auto_clear_on_error(bus: can.Bus) -> bool:
    """Automatically send CLEAR command to reset error states.
    Returns: True if clear succeeded, False otherwise
    """
    log.info("Auto-clearing error state...")
    
    # Build CAN message: D_OP_CLEAR = 2
    data = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)
    
    # Wait for ACK
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=3.0)
    if success:
        log.info("Auto-clear successful")
    else:
        log.warning("Auto-clear failed (no ACK)")
    return success

async def poll_for_completion(bus: can.Bus, timeout_s: int, is_cleaner: bool = False, mqtt_client=None, status_topic: str = None) -> str:
    """Poll Modbus Master STATUS until operation completes.
    Args:
        bus: CAN bus
        timeout_s: Operation timeout in seconds
        is_cleaner: True to poll CLEANER status (C_OP_STATUS), False for DISPENSER (D_OP_STATUS)
        mqtt_client: Optional MQTT client for publishing status updates
        status_topic: Optional MQTT topic for status updates
    Returns: 'success', 'error', or 'timeout'
    """
    poll_interval = 0.5  # Poll every 500ms
    # Use max of timeout_s or 70s to ensure we don't give up before automation_functions.py (75s timeout)
    effective_timeout = max(timeout_s, 70)
    max_polls = int((effective_timeout + 10) / poll_interval)  # timeout + 10s buffer
    
    # Choose correct status command based on device
    status_cmd_byte = 0x81 if is_cleaner else 0x01  # C_OP_STATUS or D_OP_STATUS
    device_name = "Cleaner" if is_cleaner else "Dispenser"
    
    for poll_num in range(max_polls):
        await asyncio.sleep(poll_interval)
        
        # Send STATUS command
        status_cmd = bytes([status_cmd_byte, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        clear_ack_queue(CAN_ID_DISPENSER_ACK)
        send_can(bus, CAN_ID_DISPENSER, status_cmd)
        
        # Wait for status response
        q = ACK_QUEUES.get(CAN_ID_DISPENSER_ACK)
        if not q:
            continue
        
        try:
            status_data = await asyncio.wait_for(q.get(), timeout=2.0)
            if len(status_data) >= 8:
                sys_status = status_data[3]  # byte[3] = system status
                
                # Extract temperature and elapsed time for cleaner (frother)
                if is_cleaner and len(status_data) >= 8:
                    temp_x10 = (status_data[4] << 8) | status_data[5]  # bytes 4-5: big-endian temp
                    elapsed_x10 = (status_data[6] << 8) | status_data[7]  # bytes 6-7: big-endian elapsed
                    temp_c = temp_x10 / 10.0
                    elapsed_s = elapsed_x10 / 10.0
                    
                    # Publish status update if MQTT client provided
                    if mqtt_client and status_topic and sys_status == 1:  # SYS_ACTIVE
                        status_update = {
                            "status": "active",
                            "temp_c": round(temp_c, 1),
                            "elapsed_s": round(elapsed_s, 1),
                            "poll": poll_num + 1
                        }
                        mqtt_client.publish(status_topic, json.dumps(status_update))
                        log.info(f"Frother: temp={temp_c:.1f}°C, elapsed={elapsed_s:.1f}s (poll #{poll_num + 1})")
                
                # SYS_IDLE=0, SYS_ACTIVE=1, SYS_OFF=7, faults=2-6
                if sys_status == 0 or sys_status == 7:  # SYS_IDLE or SYS_OFF - done!
                    log.info("%s completed (poll #%d, status=%d)", device_name, poll_num + 1, sys_status)
                    return 'success'
                elif sys_status in [2, 3, 4, 5, 6]:  # Fault states
                    log.warning("%s ended with fault (status=%d)", device_name, sys_status)
                    return 'error'
                # else: still ACTIVE, continue polling
        except asyncio.TimeoutError:
            continue  # No response, retry
    
    log.warning("Timeout waiting for %s completion after %d polls", device_name, max_polls)
    return 'timeout'

# ---- CAN helpers -------------------------------------------------------------
def make_payload(cmd: int, type_val: int, amount: int) -> bytes:
    """Make payload for old CAN protocol (slush, grinder, tamper, ice)"""
    if not (0 <= type_val <= 255):
        raise ValueError("type must fit in one byte (0..255)")
    if not (0 <= amount <= 65535):
        raise ValueError("amount must fit in uint16 (0..65535)")
    b0 = cmd & 0xFF
    b1 = type_val & 0xFF
    b2 = amount & 0xFF        # LE low byte
    b3 = (amount >> 8) & 0xFF # LE high byte
    return bytes([b0, b1, b2, b3, 0x00, 0x00, 0x00, 0x00])

def make_tamper_payload(cmd: int, tamping_level_timer: int) -> bytes:
    """
    Make payload for tamper CAN protocol (big-endian format).
    Format: [cmd] [timer_high] [timer_low] [padding...]
    
    Args:
        cmd: Command byte (0x01 for tamper)
        tamping_level_timer: Timer value in milliseconds (0-65535)
    
    Example: tamping_level_timer=1700 -> 0x0106A4 (cmd=0x01, high=0x06, low=0xA4)
    """
    if not (0 <= tamping_level_timer <= 65535):
        raise ValueError("tamping_level_timer must fit in uint16 (0..65535)")
    
    b0 = cmd & 0xFF
    b1 = (tamping_level_timer >> 8) & 0xFF  # Big-endian high byte
    b2 = tamping_level_timer & 0xFF         # Big-endian low byte
    return bytes([b0, b1, b2, 0x00, 0x00, 0x00, 0x00, 0x00])

def make_dispenser_payload(cmd: int, motor_id: int, weight_dg: int, liquid_type: int, ext0: int = 0x00) -> bytes:
    """
    Make payload for dispenser CAN protocol (ID 0x110).
    Format: [cmd] [motor_id] [weight_dg_low] [weight_dg_high] [liquid_type] [ext0] [ext1] [ext2]
    
    Args:
        cmd: Command byte (0x01 = dispense, 0x02 = stop, 0x03 = set_lag)
        motor_id: Motor ID (1-24, see CAN_COMMANDS.md)
        weight_dg: Weight in decigrams (little-endian, 0-65535)
        liquid_type: Liquid type (1=water, 2=milk, 3=sauce, 4=caramel, 5=syrup, 6=honey)
        ext0: Extended parameter (for SET_LAG: speed profile 0 or 1)
    """
    if not (1 <= motor_id <= 24):
        raise ValueError("motor_id must be 1-24")
    if not (0 <= weight_dg <= 65535):
        raise ValueError("weight_dg must fit in uint16 (0..65535)")
    if not (0 <= liquid_type <= 255):
        raise ValueError("liquid_type must fit in one byte")
    
    return bytes([
        cmd & 0xFF,
        motor_id & 0xFF,
        weight_dg & 0xFF,           # weight low byte
        (weight_dg >> 8) & 0xFF,    # weight high byte
        liquid_type & 0xFF,
        ext0 & 0xFF,  # ext0 (speed for SET_LAG)
        0x00,  # ext1
        0x00   # ext2
    ])

def send_can(bus: can.Bus, arb_id: int, data: bytes) -> None:
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)
    log.info("Sent CAN id=0x%03X data=%s", arb_id, data.hex())

def send_mqtt_response(client: mqtt.Client, automation_type: str, status: str) -> None:
    """Send response to automation-specific response topic"""
    # Map automation types to their corresponding response topics
    response_topic_map = {
        "milk": "automation_milk/response",
        "slush": "automation_slush/response",
        "grinder": "automation_grinding/response",
        "tamper": "automation_tampering/response",
        "ice": "automation_ice/response",
        "syrup": "automation_syrup/response",
        "rinser": "automation_rinser/response",
        "frother_init": "automation_frother_init/response",
        "frother": "automation_frother/response",
        "clean_frother": "automation_clean_frother/response",
    }
    
    # Get the response topic for this automation type, fallback to generic if not found
    response_topic = response_topic_map.get(automation_type, TOPIC_RESPONSE)
    
    response = {"automation": [automation_type], "status": status}
    response_json = json.dumps(response)
    client.publish(response_topic, response_json)
    log.info("Sent MQTT response to %s: %s", response_topic, response_json)

# =================================================================================================
# ---- Message handlers --------------------------------------------------------

# Milk type to motor_id mapping (for CAN dispenser system)
# Maps milk type name to motor_id (0x01 to 0x08)
MILK_TYPE_MAP = {
    "whole": 1,        # milk1 -> motor_id 0x01
    "low": 2,          # milk2 -> motor_id 0x02
    "lactoze": 3,      # milk3 -> motor_id 0x03
    "lactose_free": 4, # milk4 -> motor_id 0x04
    "almond": 5,       # milk5 -> motor_id 0x05
    "soy": 6,          # milk6 -> motor_id 0x06
    "oat": 7,          # milk7 -> motor_id 0x07
    "coconut": 8       # milk8 -> motor_id 0x08
}

SLUSH_TYPE_MAP = {
    "slush_1": 1,
    "slush_2": 2,
}

# Syrup/Sauce type to motor_id mapping (for CAN dispenser system)
# Maps syrup type name to motor_id (0x09 to 0x17 = 9 to 23 in decimal)
SYRUP_TYPE_MAP = {
    "vanilla": 9,        # sauce1 -> motor_id 0x09
    "hazelnut": 10,      # sauce2 -> motor_id 0x0A
    "caramel": 11,       # sauce3 -> motor_id 0x0B
    "chocolate": 12,     # sauce4 -> motor_id 0x0C
    "white_chocolate": 13,  # sauce5 -> motor_id 0x0D
    "peppermint": 14,    # sauce6 -> motor_id 0x0E
    "toffee": 15,        # sauce7 -> motor_id 0x0F
    "maple": 16,         # sauce8 -> motor_id 0x10
    "cinnamon": 17,      # sauce9 -> motor_id 0x11
    "irish_cream": 18,   # sauce10 -> motor_id 0x12
    "coconut": 19,       # sauce11 -> motor_id 0x13
    "almond": 20,        # sauce12 -> motor_id 0x14
    "amaretto": 21,      # sauce13 -> motor_id 0x15
    "pumpkin_spice": 22, # sauce14 -> motor_id 0x16
    "lavender": 23,      # sauce15 -> motor_id 0x17
}

# Frother temperature mapping (in Celsius)
FROTHER_TEMP_MAP = {
    "kids": 54.0,           # Kids temperature: 54°C
    "standard": 71.5,       # Standard temperature: 71.5°C
    "extra_hot": 83.5,      # Extra hot temperature: 83.5°C
}

# Lag settings for dispensers (in grams, with speed profile)
# Based on can_commands_lag_dispense.md
DISPENSER_LAG_MAP = {
    # Milk motors (1-8): 18g lag, speed 0
    1: {"lag_g": 18.0, "speed": 0},
    2: {"lag_g": 18.0, "speed": 0},
    3: {"lag_g": 18.0, "speed": 0},
    4: {"lag_g": 18.0, "speed": 0},
    5: {"lag_g": 18.0, "speed": 0},
    6: {"lag_g": 18.0, "speed": 0},
    7: {"lag_g": 18.0, "speed": 0},
    8: {"lag_g": 18.0, "speed": 0},
    # Sauce motors (9-21): 15g lag, speed 0
    9: {"lag_g": 15.0, "speed": 0},
    10: {"lag_g": 15.0, "speed": 0},
    11: {"lag_g": 15.0, "speed": 0},
    12: {"lag_g": 15.0, "speed": 0},
    13: {"lag_g": 15.0, "speed": 0},
    14: {"lag_g": 15.0, "speed": 0},
    15: {"lag_g": 15.0, "speed": 0},
    16: {"lag_g": 15.0, "speed": 0},
    17: {"lag_g": 15.0, "speed": 0},
    18: {"lag_g": 15.0, "speed": 0},
    19: {"lag_g": 15.0, "speed": 0},
    20: {"lag_g": 15.0, "speed": 0},
    21: {"lag_g": 15.0, "speed": 0},
    # Sauce motors (22-23): 1g lag, speed 0
    22: {"lag_g": 1.0, "speed": 0},
    23: {"lag_g": 1.0, "speed": 0},
}

# Motor mapping (from sketch_jan24b.ino line 71)
MILK_MOTORS = {8, 12, 15, 16, 17, 18, 19, 20}
SYRUP_MOTORS = {1, 7, 9, 11, 13, 14, 23}

async def handle_dispense(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any], requested_type: str) -> None:
    """
    Handle milk/syrup dispensing via Modbus Master CAN protocol (sketch_jan24a).
    Auto-routes based on motor ID regardless of topic.
    
    Expected payload: {"pump_number": 8, "amount": 50, "slow_pct": 0, "viscous_pct": 0, "timeout_s": 60}
    
    CAN Message Format (8 bytes):
    [0] = cmd (D_OP_DISPENSE = 10)
    [1-2] = motorId (big-endian u16)
    [3-4] = target_g * 10 in decigrams (big-endian u16)
    [5] = slowPct (0-100)
    [6] = viscPct (0-100)  
    [7] = timeout_s
    """
    try:
        motor_id = int(payload["pump_number"])
        amount_grams = float(payload["amount"])
        visc_pct = int(payload.get("viscous_pct", payload.get("visc", 0)))
        timeout_s = int(payload.get("timeout_s", payload.get("timeout", 60)))
        
        # Auto-route based on motor ID (smart routing)
        if motor_id in MILK_MOTORS:
            actual_type = "milk"
        elif motor_id in SYRUP_MOTORS:
            actual_type = "syrup"
        else:
            # Unknown motor, use requested type
            actual_type = requested_type
            log.warning(f"Motor {motor_id} not in known milk/syrup list; using requested type '{requested_type}'")
        
        # Log if there's a mismatch (but still proceed)
        if actual_type != requested_type:
            log.info(f"Auto-routing: motor {motor_id} is {actual_type.upper()} (requested '{requested_type}' topic)")
        
        # Auto slow-down logic:
        # For motors 8, 12, 16: never auto-slow (use provided value or 0)
        # For other motors: only auto-slow if amount > 15g
        NO_AUTO_SLOW_MOTORS = {8, 12, 16}
        
        # Get user-provided value if any
        user_slow_pct = payload.get("slow_pct", payload.get("slow", None))
        
        if motor_id in NO_AUTO_SLOW_MOTORS:
            # Use provided value or default to 0
            slow_pct = int(user_slow_pct) if user_slow_pct is not None else 0
        else:
            # Auto-slow only if amount > 15g
            if amount_grams > 15.0:
                # If user didn't specify, use auto 70%
                if user_slow_pct is None or user_slow_pct == 0:
                    slow_pct = 70
                    log.info(f"Auto slow-down: motor {motor_id} will slow at {slow_pct}% of target (amount={amount_grams}g > 15g)")
                else:
                    slow_pct = int(user_slow_pct)
            else:
                # Small amount (<= 15g), don't auto-slow
                slow_pct = int(user_slow_pct) if user_slow_pct is not None else 0
                if slow_pct == 0:
                    log.info(f"No slow-down: motor {motor_id} amount={amount_grams}g <= 15g (full speed)")
        
        weight_dg = int(amount_grams * 10)  # Convert grams to decigrams
        
        log.info(f"{actual_type.capitalize()} motor {motor_id}: dispensing {amount_grams}g (slow={slow_pct}%, visc={visc_pct}%, timeout={timeout_s}s)")
        
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, requested_type, "error")
        raise ValueError(f"Invalid dispense payload: {payload} ({e})")

    # Build CAN message for Modbus Master (sketch_jan24a)
    # D_OP_DISPENSE = 10 (0x0A)
    data = bytes([
        0x0A,  # cmd: D_OP_DISPENSE
        (motor_id >> 8) & 0xFF,  # motorId high byte
        motor_id & 0xFF,         # motorId low byte
        (weight_dg >> 8) & 0xFF, # target_g (dg) high byte
        weight_dg & 0xFF,        # target_g (dg) low byte
        slow_pct & 0xFF,         # slowPct
        visc_pct & 0xFF,         # viscPct
        timeout_s & 0xFF         # timeout_s
    ])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)

    # Wait for initial ACK from Modbus Master (responds on 0x121)
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=10.0)
    if not success:
        # Auto-clear error before returning
        log.warning("Initial ACK failed, sending auto-clear...")
        await auto_clear_on_error(bus)
        send_mqtt_response(client, actual_type, "timeout")
        return
    
    # Poll for completion (Modbus Master will show status change from ACTIVE to IDLE)
    log.info("Dispense started, polling for completion...")
    result = await poll_for_completion(bus, timeout_s)
    
    # If error occurred, auto-clear before responding
    if result in ['error', 'timeout']:
        log.warning(f"Dispense ended with {result}, sending auto-clear...")
        await auto_clear_on_error(bus)
    
    send_mqtt_response(client, actual_type, result)

# ---- Message MILK (wrapper) --------------------------------------------------------
async def handle_milk(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """Handle milk dispense request - delegates to common handler with auto-routing."""
    await handle_dispense(bus, client, payload, "milk")

# ---- Message SLUSH --------------------------------------------------------
async def handle_slush(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        stype_str = payload["slush_type"].lower().strip()
        # Accept either "weight" or "timer" field
        amount = int(payload.get("weight", payload.get("timer", 0)))
        difference = int(payload.get("difference", 100))  # Optional field
        
        if stype_str not in SLUSH_TYPE_MAP:
            raise ValueError(f"Unknown slush type: {stype_str}")
        stype = SLUSH_TYPE_MAP[stype_str]
        
        log.info(f"Slush type={stype_str} amount={amount} difference={difference}")
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "slush", "error")
        raise ValueError(f"Invalid slush payload: {payload} ({e})")

    clear_ack_queue(CAN_ID_SLUSH)
    data = make_payload(cmd=0x02, type_val=stype, amount=amount)
    send_can(bus, CAN_ID_SLUSH, data)

    success = await wait_for_can_response(CAN_ID_SLUSH)
    send_mqtt_response(client, "slush", "success" if success else "timeout")

async def handle_grinder(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        shots = int(payload["shots_number"])
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "grinder", "error")
        raise ValueError(f"Invalid grinder payload: {payload} ({e})")

    clear_ack_queue(CAN_ID_GRINDER)
    data = make_payload(cmd=0x03, type_val=shots, amount=0)
    send_can(bus, CAN_ID_GRINDER, data)

    success = await wait_for_can_response(CAN_ID_GRINDER)
    send_mqtt_response(client, "grinder", "success" if success else "timeout")


# ---- Message TAMPER --------------------------------------------------------
async def handle_tamper(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        tampering = int(payload["tampering"])
        calibration = int(payload["calibration"])
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "tamper", "error")
        raise ValueError(f"Invalid tamper payload: {payload} ({e})")


    clear_ack_queue(CAN_ID_TAMPER)
    data = make_tamper_payload(cmd=0x01, tamping_level_timer=calibration)
    send_can(bus, CAN_ID_TAMPER, data)

    success = await wait_for_can_response(CAN_ID_TAMPER)
    send_mqtt_response(client, "tamper", "success" if success else "timeout")


async def handle_ice(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        ice = int(payload["ice"])
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "ice", "error")
        raise ValueError(f"Invalid ice payload: {payload} ({e})")

    clear_ack_queue(CAN_ID_ICE)
    data = make_payload(cmd=0x01, type_val=0, amount=ice)
    send_can(bus, CAN_ID_ICE, data)

    success = await wait_for_can_response(CAN_ID_ICE)
    send_mqtt_response(client, "ice", "success" if success else "timeout")

# ---- Message SYRUP (wrapper) --------------------------------------------------------
async def handle_syrup(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """Handle syrup dispense request - delegates to common handler with auto-routing."""
    await handle_dispense(bus, client, payload, "syrup")

# ---- Message STATUS --------------------------------------------------------
async def handle_status(bus: can.Bus, client: mqtt.Client, automation_type: str) -> None:
    """
    Send STATUS command to Modbus Master to get current state.
    CAN Message Format (8 bytes):
    [0] = 0x01 (D_OP_STATUS)
    [1-7] = 0x00 (unused)
    
    Response: [cmd, result, error, status, weight_x10_lo, weight_x10_hi, scale_id, ...]
    """
    log.info(f"Sending STATUS command for {automation_type}")
    
    # Build CAN message: D_OP_STATUS = 1
    data = bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)
    
    # Wait for response with status details
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=5.0)
    send_mqtt_response(client, automation_type, "success" if success else "timeout")

# ---- Message CLEAR (Error Reset) --------------------------------------------------------
async def handle_clear(bus: can.Bus, client: mqtt.Client, automation_type: str) -> None:
    """
    Send CLEAR command to Modbus Master to reset error states.
    CAN Message Format (8 bytes):
    [0] = 0x02 (D_OP_CLEAR)
    [1-7] = 0x00 (unused)
    """
    log.info(f"Sending CLEAR command for {automation_type}")
    
    # Build CAN message: D_OP_CLEAR = 2
    data = bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)
    
    # Wait for ACK
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=5.0)
    send_mqtt_response(client, automation_type, "success" if success else "timeout")

# ---- Message RINSER --------------------------------------------------------
async def handle_rinser(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """
    Handle rinse operation via Modbus Master CAN protocol.
    Expected payload: {"rinser": 1, "timer": 5} (from automation_functions.py)
    
    CAN Message Format (8 bytes):
    [0] = 0x0B (D_OP_RINSE)
    [1-2] = seconds_x10 (big-endian u16)
    [3-7] = 0x00 (unused)
    """
    try:
        rinser_state = int(payload.get("rinser", 1))
        timer = float(payload.get("timer", 5.0))  # Get timer from payload
        
        # If timer is 0, use default 5 seconds
        if timer <= 0:
            timer = 5.0
        
        seconds_x10 = int(timer * 10.0)
        
        log.info(f"Rinse: {timer}s (rinser_state={rinser_state})")
        
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "rinser", "error")
        raise ValueError(f"Invalid rinse payload: {payload} ({e})")
    
    # Build CAN message: D_OP_RINSE = 11 (0x0B)
    data = bytes([
        0x0B,  # cmd: D_OP_RINSE
        (seconds_x10 >> 8) & 0xFF,  # seconds_x10 high byte
        seconds_x10 & 0xFF,         # seconds_x10 low byte
        0x00, 0x00, 0x00, 0x00, 0x00  # unused
    ])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)
    
    # Wait for initial ACK
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=5.0)
    if not success:
        log.warning("Rinse ACK failed, sending auto-clear...")
        await auto_clear_on_error(bus)
        send_mqtt_response(client, "rinser", "timeout")
        return
    
    # Poll for completion
    log.info("Rinse started, polling for completion...")
    result = await poll_for_completion(bus, int(timer + 10))
    
    # If error occurred, auto-clear before responding
    if result in ['error', 'timeout']:
        log.warning(f"Rinse ended with {result}, sending auto-clear...")
        await auto_clear_on_error(bus)
    
    send_mqtt_response(client, "rinser", result)

# ---- Message FROTHER INIT --------------------------------------------------------
async def handle_frother_init(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """
    Handle frother initialization via Modbus Master CAN protocol.
    Expected payload: {"frother_init": 1}
    
    CAN Message to Modbus Master (0x120):
    [0] = 0x8A (C_OP_INIT with 0x80 flag for CLEANER)
    [1-2] = seconds_x10 (big-endian u16, default 50 = 5.0s)
    [3-7] = 0x00
    
    Response on 0x121 from Modbus Master
    """
    try:
        frother_init = int(payload.get("frother_init", 1))
        seconds = float(payload.get("seconds", 1.5))  # Default 1.5 seconds
        seconds_x10 = int(seconds * 10.0)
        
        log.info(f"Initializing frother via Modbus Master (seconds={seconds})")
        
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "frother_init", "error")
        raise ValueError(f"Invalid frother_init payload: {payload} ({e})")

    # Build CAN message: C_OP_INIT = 10, with 0x80 flag = 0x8A
    data = bytes([
        0x8A,  # cmd: C_OP_INIT (10 | 0x80)
        (seconds_x10 >> 8) & 0xFF,  # seconds_x10 high byte
        seconds_x10 & 0xFF,         # seconds_x10 low byte
        0x00, 0x00, 0x00, 0x00, 0x00
    ])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)

    # Wait for ACK from Modbus Master
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=10.0)
    if not success:
        log.warning("Frother init ACK failed, sending auto-clear...")
        await auto_clear_on_error(bus)
        send_mqtt_response(client, "frother_init", "timeout")
        return
    
    # Poll for completion (CLEANER status, not DISPENSER)
    log.info("Frother init started, polling for completion...")
    result = await poll_for_completion(bus, int(seconds + 10), is_cleaner=True)
    
    if result in ['error', 'timeout']:
        log.warning(f"Frother init ended with {result}, sending auto-clear...")
        await auto_clear_on_error(bus)
    
    send_mqtt_response(client, "frother_init", result)

# ---- Message FROTHER --------------------------------------------------------
async def handle_frother(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """
    Handle frother operation via Modbus Master CAN protocol.
    Expected payload: {"temperature": "standard"}
    - temperature: one of FROTHER_TEMP_MAP keys (kids, standard, extra_hot)
    
    CAN Message to Modbus Master (0x120):
    [0] = 0x8B (C_OP_FROTH with 0x80 flag for CLEANER)
    [1-2] = targetC_x10 (big-endian u16, e.g., 71.5°C = 715)
    [3-4] = timeout_s_x10 (big-endian u16, e.g., 180s = 1800)
    [5-7] = 0x00
    
    Response on 0x121 from Modbus Master
    """
    try:
        temp_str = payload["temperature"].lower().strip()
        
        if temp_str not in FROTHER_TEMP_MAP:
            raise ValueError(f"Unknown temperature: {temp_str}")
        
        target_temp_c = FROTHER_TEMP_MAP[temp_str]
        timeout_s = 180.0  # Default timeout 180 seconds
        
        # Convert to CAN format (x10 scaling, big-endian)
        target_temp_x10 = int(target_temp_c * 10)  # e.g., 71.5°C -> 715
        timeout_x10 = int(timeout_s * 10)  # 180s -> 1800
        
        log.info(f"Frothing at {target_temp_c}°C (timeout={timeout_s}s)")
        
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "frother", "error")
        raise ValueError(f"Invalid frother payload: {payload} ({e})")

    # Build CAN message: C_OP_FROTH = 11, with 0x80 flag = 0x8B
    data = bytes([
        0x8B,  # cmd: C_OP_FROTH (11 | 0x80)
        (target_temp_x10 >> 8) & 0xFF,  # targetC_x10 high byte
        target_temp_x10 & 0xFF,         # targetC_x10 low byte
        (timeout_x10 >> 8) & 0xFF,      # timeout_x10 high byte
        timeout_x10 & 0xFF,             # timeout_x10 low byte
        0x00, 0x00, 0x00
    ])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)

    # Wait for ACK from Modbus Master
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=10.0)
    if not success:
        log.warning("Frother ACK failed, sending auto-clear...")
        await auto_clear_on_error(bus)
        send_mqtt_response(client, "frother", "timeout")
        return
    
    # Poll for completion (CLEANER status, not DISPENSER) with temperature monitoring
    log.info("Frothing started, polling for completion...")
    result = await poll_for_completion(bus, int(timeout_s + 20), is_cleaner=True, 
                                       mqtt_client=client, status_topic="automation_frother/status")
    
    if result in ['error', 'timeout']:
        log.warning(f"Frothing ended with {result}, sending auto-clear...")
        await auto_clear_on_error(bus)
    
    send_mqtt_response(client, "frother", result)

# ---- Message CLEAN FROTHER --------------------------------------------------------
async def handle_clean_frother(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    """
    Handle frother cleaning via Modbus Master CAN protocol.
    Expected payload: {"clean_frother": 1}
    
    CAN Message to Modbus Master (0x120):
    [0] = 0x8C (C_OP_CLEAN with 0x80 flag for CLEANER)
    [1-2] = tValve_x10 (big-endian u16, default 10s = 100)
    [3-4] = tSteam_x10 (big-endian u16, default 15s = 150)
    [5-6] = tStandby_x10 (big-endian u16, default 10s = 100)
    [7] = 0x00
    
    Response on 0x121 from Modbus Master
    """
    try:
        clean_frother = int(payload.get("clean_frother", 1))
        # Default clean cycle times (in seconds)
        tValve = float(payload.get("tValve", 5.0))
        tSteam = float(payload.get("tSteam", 5.0))
        tStandby = float(payload.get("tStandby", 5.0))
        
        tValve_x10 = int(tValve * 10.0)
        tSteam_x10 = int(tSteam * 10.0)
        tStandby_x10 = int(tStandby * 10.0)
        
        log.info(f"Cleaning frother (tValve={tValve}s, tSteam={tSteam}s, tStandby={tStandby}s)")
        
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "clean_frother", "error")
        raise ValueError(f"Invalid clean_frother payload: {payload} ({e})")

    # Build CAN message: C_OP_CLEAN = 12, with 0x80 flag = 0x8C
    data = bytes([
        0x8C,  # cmd: C_OP_CLEAN (12 | 0x80)
        (tValve_x10 >> 8) & 0xFF,
        tValve_x10 & 0xFF,
        (tSteam_x10 >> 8) & 0xFF,
        tSteam_x10 & 0xFF,
        (tStandby_x10 >> 8) & 0xFF,
        tStandby_x10 & 0xFF,
        0x00
    ])
    
    clear_ack_queue(CAN_ID_DISPENSER_ACK)
    send_can(bus, CAN_ID_DISPENSER, data)

    # Wait for ACK from Modbus Master
    success = await wait_for_can_response(CAN_ID_DISPENSER_ACK, timeout=10.0)
    if not success:
        log.warning("Clean frother ACK failed, sending auto-clear...")
        await auto_clear_on_error(bus)
        send_mqtt_response(client, "clean_frother", "timeout")
        return
    
    # Poll for completion (CLEANER status, not DISPENSER)
    total_time = tValve + tSteam + tStandby
    log.info(f"Clean frother started, polling for completion (total={total_time}s)...")
    result = await poll_for_completion(bus, int(total_time + 20), is_cleaner=True)
    
    if result in ['error', 'timeout']:
        log.warning(f"Clean frother ended with {result}, sending auto-clear...")
        await auto_clear_on_error(bus)
    
    send_mqtt_response(client, "clean_frother", result)


# ---- MQTT setup --------------------------------------------------------------
class Bridge:
    def __init__(self):
        # CAN bus
        self.bus = can.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE)

        # MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mqtt-can-bridge")
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        # Async event loop and executor
        self.loop = None
        self.executor = ThreadPoolExecutor(max_workers=5)

        # CAN notifier (set in run_async)
        self.notifier = None
        self.id_to_name = build_id_name_map()

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            log.info("MQTT connected")
            client.subscribe([
                (TOPIC_MILK, 1),
                (TOPIC_SLUSH, 1),
                (TOPIC_GRINDER, 1),
                (TOPIC_TAMPER, 1),
                (TOPIC_ICE, 1),
                (TOPIC_SYRUP, 1),
                (TOPIC_RINSER, 1),
                (TOPIC_STATUS_MILK, 1),
                (TOPIC_STATUS_SYRUP, 1),
                (TOPIC_CLEAR_MILK, 1),
                (TOPIC_CLEAR_SYRUP, 1),
                (TOPIC_FROTHER_INIT, 1),
                (TOPIC_FROTHER, 1),
                (TOPIC_CLEAN_FROTHER, 1)
            ])
            log.info("Subscribed to all automation topics")
        else:
            log.error("MQTT connect failed with code: %s", reason_code)

    def on_disconnect(self, client, userdata, reason_code, properties=None):
        log.warning("MQTT disconnected: %s", reason_code)

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="replace").strip()
        log.info("MQTT msg on '%s': %s", topic, payload_raw)

        try:
            payload = json.loads(payload_raw) if payload_raw else {}
        except json.JSONDecodeError as e:
            log.error("Bad JSON on '%s': %s", topic, e)
            return

        # Handle messages asynchronously
        if self.loop and not self.loop.is_closed():
            if topic == TOPIC_MILK:
                asyncio.run_coroutine_threadsafe(
                    handle_milk(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_SLUSH:
                asyncio.run_coroutine_threadsafe(
                    handle_slush(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_GRINDER:
                asyncio.run_coroutine_threadsafe(
                    handle_grinder(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_TAMPER:
                asyncio.run_coroutine_threadsafe(
                    handle_tamper(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_ICE:
                asyncio.run_coroutine_threadsafe(
                    handle_ice(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_SYRUP:
                asyncio.run_coroutine_threadsafe(
                    handle_syrup(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_RINSER:
                asyncio.run_coroutine_threadsafe(
                    handle_rinser(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_STATUS_MILK:
                asyncio.run_coroutine_threadsafe(
                    handle_status(self.bus, self.client, "milk"), self.loop
                )
            elif topic == TOPIC_STATUS_SYRUP:
                asyncio.run_coroutine_threadsafe(
                    handle_status(self.bus, self.client, "syrup"), self.loop
                )
            elif topic == TOPIC_CLEAR_MILK:
                asyncio.run_coroutine_threadsafe(
                    handle_clear(self.bus, self.client, "milk"), self.loop
                )
            elif topic == TOPIC_CLEAR_SYRUP:
                asyncio.run_coroutine_threadsafe(
                    handle_clear(self.bus, self.client, "syrup"), self.loop
                )
            elif topic == TOPIC_FROTHER_INIT:
                asyncio.run_coroutine_threadsafe(
                    handle_frother_init(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_FROTHER:
                asyncio.run_coroutine_threadsafe(
                    handle_frother(self.bus, self.client, payload), self.loop
                )
            elif topic == TOPIC_CLEAN_FROTHER:
                asyncio.run_coroutine_threadsafe(
                    handle_clean_frother(self.bus, self.client, payload), self.loop
                )
            else:
                log.warning("Unhandled topic: %s", topic)

    async def mqtt_loop(self):
        """Run MQTT client in async loop"""
        while True:
            try:
                self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
                # Run MQTT loop in executor to avoid blocking
                await self.loop.run_in_executor(
                    self.executor,
                    lambda: self.client.loop_forever(retry_first_connection=True)
                )
            except Exception as e:
                log.error("MQTT connection error: %s", e)
                await asyncio.sleep(2)

    async def health_check_loop(self):
        """Periodic health check task - sends health status every 1.2 minutes"""
        while True:
            try:
                await asyncio.sleep(HEALTH_CHECK_INTERVAL)
                send_health_check_mqtt(self.client)
            except Exception as e:
                log.error("Health check error: %s", e)
                await asyncio.sleep(5)  # Wait before retrying

    async def run_async(self):
        """Main async run method"""
        self.loop = asyncio.get_event_loop()

        # Initialize per-ID ACK queues
        global ACK_QUEUES
        ACK_QUEUES = {cid: asyncio.Queue() for cid in self.id_to_name.keys()}

        # Initialize device health monitoring
        initialize_device_health()

        # Start CAN notifier with our listener (logs heartbeats, routes ACKs)
        self.notifier = can.Notifier(
            self.bus,
            [HeartbeatAndAckListener(self.id_to_name, self.loop)],
            timeout=0.01
        )

        # Start MQTT loop and health check loop
        mqtt_task = asyncio.create_task(self.mqtt_loop())
        health_task = asyncio.create_task(self.health_check_loop())

        try:
            # Wait for either task to complete (they run forever)
            await asyncio.gather(mqtt_task, health_task)
        except KeyboardInterrupt:
            log.info("Shutting down...")
            mqtt_task.cancel()
            health_task.cancel()
        except Exception as e:
            log.error("Error in async loop: %s", e)

    def run(self):
        """Entry point - start async loop"""
        asyncio.run(self.run_async())

    def close(self):
        try:
            if self.notifier:
                self.notifier.stop()
        except Exception:
            pass
        try:
            self.client.disconnect()
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
        if self.executor:
            self.executor.shutdown(wait=True)

# ---- Main --------------------------------------------------------------------
if __name__ == "__main__":
    bridge = Bridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        log.info("Shutting down...")
    finally:
        bridge.close()
        log.info("Bye.")
