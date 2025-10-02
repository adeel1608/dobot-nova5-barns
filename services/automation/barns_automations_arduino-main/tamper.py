import paho.mqtt.client as mqtt
import json
import time

received_response = None

def on_connect(client, userdata, flags, rc, properties=None):
    print("Dispenser [MQTT] Connected with result code", rc)

def on_message(client, userdata, msg):
    global received_response
    try:
        payload = json.loads(msg.payload.decode())
        print("Response received:")
        print(json.dumps(payload, indent=2))
        received_response = payload
    except json.JSONDecodeError:
        print("Invalid JSON:", msg.payload.decode())

def send_and_wait(slush_type):
    global received_response
    received_response = None  # reset

    message = {
        "tampering": 1
    }

    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set("admin", "admin123")
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("192.168.200.254", 1883, 60)
    client.loop_start()

    # Subscribe before publishing to ensure we catch the response
    client.subscribe("automation/response", qos=1)

    # Publish the request
    client.publish("automation_tampering", json.dumps(message), qos=1)
    print("Sent message:", json.dumps(message))

    # Wait for response or timeout
    start_time = time.time()
    timeout = 220  # seconds

    while received_response is None and (time.time() - start_time) < timeout:
        time.sleep(0.1)

    client.loop_stop()
    client.disconnect()

    if received_response:
        return received_response
    else:
        print("No response received within timeout.")
        return None

# Example use
if __name__ == "__main__":
    resp = send_and_wait(1)
    if resp:
        print("Final response:", resp)
