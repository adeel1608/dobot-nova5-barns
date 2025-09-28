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
MQTT_HOST = "192.168.100.48"
MQTT_PORT = 1883
MQTT_USERNAME = "admin"
MQTT_PASSWORD = "admin123"

TOPIC_MILK = "automation_milk"
TOPIC_SLUSH = "automation_slush"
TOPIC_GRINDER = "automation_grinding"
TOPIC_TAMPER = "automation_tampering"
TOPIC_ICE = "automation_ice"
TOPIC_RESPONSE = "automation/response"

# CAN interface (Linux socketcan). Ensure `can0` is up: e.g.
# sudo ip link set can0 up type can bitrate 500000
CAN_CHANNEL = "can0"
CAN_INTERFACE = "socketcan"

# Fixed CAN IDs
CAN_ID_MILK_1 = 0x105  # For milk types 1-4 (also used by ICE below)
CAN_ID_MILK_2 = 0x104  # For milk types 5-8
CAN_ID_SLUSH = 0x106
CAN_ID_GRINDER = 0x101
CAN_ID_TAMPER = 0x103
CAN_ID_ICE = 0x105      # same as MILK_1 on purpose

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

def build_id_name_map() -> Dict[int, str]:
    # Combine names if multiple features share the same ID.
    pairs = [
        (CAN_ID_GRINDER, "grinder"),
        (CAN_ID_SLUSH,   "slush"),
        (CAN_ID_TAMPER,  "tamper"),
        (CAN_ID_MILK_1,  "milk_1-4"),
        (CAN_ID_MILK_2,  "milk_5-8"),
        (CAN_ID_ICE,     "ice"),
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

            # ACK: b0 == 0x01
            if len(data) >= 1 and data[0] == 0x01:
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

async def wait_for_can_response(expected_id: int, timeout: float = 5.0) -> bool:
    """Wait for ACK (b0=1) from expected_id via per-ID queue (filled by listener)."""
    q = ACK_QUEUES.get(expected_id)
    if q is None:
        # Should not happen if initialized properly
        log.warning("No ACK queue for id=0x%03X", expected_id)
        await asyncio.sleep(timeout)
        return False
    try:
        data = await asyncio.wait_for(q.get(), timeout=timeout)
        log.info("Received CAN response: id=0x%03X data=%s", expected_id, data.hex())
        return True
    except asyncio.TimeoutError:
        log.warning("Timeout waiting for CAN response from id=0x%03X", expected_id)
        return False

# ---- CAN helpers -------------------------------------------------------------
def make_payload(cmd: int, type_val: int, amount: int) -> bytes:
    if not (0 <= type_val <= 255):
        raise ValueError("type must fit in one byte (0..255)")
    if not (0 <= amount <= 65535):
        raise ValueError("amount must fit in uint16 (0..65535)")
    b0 = cmd & 0xFF
    b1 = type_val & 0xFF
    b2 = amount & 0xFF        # LE low byte
    b3 = (amount >> 8) & 0xFF # LE high byte
    return bytes([b0, b1, b2, b3, 0x00, 0x00, 0x00, 0x00])

def send_can(bus: can.Bus, arb_id: int, data: bytes) -> None:
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)
    log.info("Sent CAN id=0x%03X data=%s", arb_id, data.hex())

def send_mqtt_response(client: mqtt.Client, automation_type: str, status: str) -> None:
    """Send response to automation_response topic"""
    response = {"automation": [automation_type], "status": status}
    response_json = json.dumps(response)
    client.publish(TOPIC_RESPONSE, response_json)
    log.info("Sent MQTT response: %s", response_json)

# =================================================================================================
# ---- Message handlers --------------------------------------------------------

MILK_TYPE_MAP = {
    "whole": 1,
    "low": 2, 
    "lactoze": 3,
    "lactose_free": 4,
    "almond": 5,
    "soy": 6,
    "oat": 7,
    "coconut": 8
}

SLUSH_TYPE_MAP = {
    "slush_1": 1,
    "slush_2": 2,
}

# ---- Message MILK --------------------------------------------------------
async def handle_milk(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        milk_type_str = payload["milk_type"].lower().strip()
        amount = int(payload["amount"])
        if milk_type_str not in MILK_TYPE_MAP:
            raise ValueError(f"Unknown milk type: {milk_type_str}")
        mtype = MILK_TYPE_MAP[milk_type_str]
        can_id = CAN_ID_MILK_1 if mtype <= 4 else CAN_ID_MILK_2
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "milk", "error")
        raise ValueError(f"Invalid milk payload: {payload} ({e})")

    clear_ack_queue(can_id)
    data = make_payload(cmd=0x01, type_val=mtype, amount=amount)
    send_can(bus, can_id, data)

    success = await wait_for_can_response(can_id)
    send_mqtt_response(client, "milk", "success" if success else "timeout")

# ---- Message SLUSH --------------------------------------------------------
async def handle_slush(bus: can.Bus, client: mqtt.Client, payload: Dict[str, Any]) -> None:
    try:
        stype_str = payload["slush_type"].lower().strip()
        amount = int(payload["timer"])
        if stype_str not in SLUSH_TYPE_MAP:
            raise ValueError(f"Unknown slush type: {stype_str}")
        stype = SLUSH_TYPE_MAP[stype_str]
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "slush", "error")
        raise ValueError(f"Invalid syrup payload: {payload} ({e})")

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
    except (KeyError, ValueError, TypeError) as e:
        send_mqtt_response(client, "tamper", "error")
        raise ValueError(f"Invalid tamper payload: {payload} ({e})")

    clear_ack_queue(CAN_ID_TAMPER)
    data = make_payload(cmd=0x01, type_val=tampering, amount=0)
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
    data = make_payload(cmd=0x05, type_val=ice, amount=0)
    send_can(bus, CAN_ID_ICE, data)

    success = await wait_for_can_response(CAN_ID_ICE)
    send_mqtt_response(client, "ice", "success" if success else "timeout")

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
                (TOPIC_ICE, 1)
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

    async def run_async(self):
        """Main async run method"""
        self.loop = asyncio.get_event_loop()

        # Initialize per-ID ACK queues
        global ACK_QUEUES
        ACK_QUEUES = {cid: asyncio.Queue() for cid in self.id_to_name.keys()}

        # Start CAN notifier with our listener (logs heartbeats, routes ACKs)
        self.notifier = can.Notifier(
            self.bus,
            [HeartbeatAndAckListener(self.id_to_name, self.loop)],
            timeout=0.01
        )

        # Start MQTT loop
        mqtt_task = asyncio.create_task(self.mqtt_loop())

        try:
            await mqtt_task
        except KeyboardInterrupt:
            log.info("Shutting down...")
            mqtt_task.cancel()
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
