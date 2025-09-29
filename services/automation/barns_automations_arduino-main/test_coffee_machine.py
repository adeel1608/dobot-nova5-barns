#!/usr/bin/env python3
import json
import time
from test_communction import LaSpazialeCoffeeMachine
from paho.mqtt.client import Client, CallbackAPIVersion

# --- Configuration ---
MQTT_BROKER         = "192.168.200.254"
# MQTT_BROKER         = "192.168.200.104" #Bring it back to 254 when done testin
MQTT_PORT           = 1883
MQTT_USER           = "admin"
MQTT_PASS           = "admin123"
MQTT_TOPIC_COMMAND  = "automation_coffee_machine"
MQTT_TOPIC_RESPONSE = "automation/response"

# --- Instantiate MQTT client ---
mqtt_client = Client(callback_api_version=CallbackAPIVersion.VERSION1)
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)

# --- Connect to the coffee machine ---
coffee_machine = LaSpazialeCoffeeMachine(port="/dev/ttyUSB0")
if not coffee_machine.connect():
    raise RuntimeError("Could not connect to coffee machine")
if coffee_machine.is_machine_blocked():
    raise RuntimeError("Machine is blocked")
else:
    print("→ Coffee machine is ready for commands")
# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, rc):
    print(f"→ MQTT connected (rc={rc}), clearing retained and subscribing…")
    client.publish(MQTT_TOPIC_COMMAND, payload="", retain=True)
    client.subscribe(MQTT_TOPIC_COMMAND)
    print(f"→ Subscribed to '{MQTT_TOPIC_COMMAND}'")

def on_message(client, userdata, msg):
    if msg.retain:
        print("→ Ignored retained message")
        return

    try:
        data = json.loads(msg.payload.decode("utf-8").strip())
    except Exception as e:
        print(f"→ JSON parse error: {e}")
        return

    slot        = data.get("slot_number")
    coffee_type = data.get("coffee_t")
    print(f"→ Received command: slot={slot}, coffee_type={coffee_type}")
    if slot is None or coffee_type is None:
        print(f"→ Invalid payload, missing keys: coffee_t={coffee_type}, slot={slot}")
        return

    # clear any ongoing cycle before starting
    coffee_machine.stop_delivery(slot)
    coffee_machine.wait_for_idle(slot)

    print(f"→ Dispatching: coffee_type={coffee_type}, slot={slot}")
    if coffee_type == 1:
        # coffee_machine.deliver_single_long(slot)
        # coffee_machine.deliver_single_long(slot)
        coffee_machine.deliver_double_short(slot)
        # time.sleep(2)
    elif coffee_type == 2:
        coffee_machine.deliver_double_long(slot)

    elif coffee_type == 3:
        coffee_machine.deliver_single_short(slot)
        # time.sleep(2)

    else:
        print(f"→ Unknown coffee_type: {coffee_type}")
        return

    # final sync: ensure everything's idle before status/check
    coffee_machine.wait_for_idle(slot)

    # optional status check
    status = coffee_machine.get_group_selection(slot)
    print(f"→ Group {slot} status: {status}")

    # publish back a success status
    resp = {
        "status":      "success",
        "group":       slot,
        "coffee_type": coffee_type
    }
    client.publish(MQTT_TOPIC_RESPONSE, json.dumps(resp))
    print(f"→ Published to '{MQTT_TOPIC_RESPONSE}': {resp}")

# --- Wire up callbacks and start loop ---
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_forever()