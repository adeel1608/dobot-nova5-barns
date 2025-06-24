# services/automation/automation_functions.py
"""Automation functions for BARNS coffee brewing system."""

import asyncio
import time

#Ibrahim (Disoenser)
import paho.mqtt.client as mqtt
import json

async def heat_water(params: dict):
    """Heat water to specified temperature."""
    target_temp = params.get("target_temp_c", 93)
    volume_ml = params.get("volume_ml", 250)
    
    # Simulate heating process
    await asyncio.sleep(3)
    
    return {
        "success": True,
        "message": f"Heated {volume_ml}ml water to {target_temp}°C",
        "details": {
            "target_temperature": target_temp,
            "volume": volume_ml,
            "actual_temperature": target_temp,
            "duration_sec": 3
        }
    }

# Milk Dispenser
# This function uses MQTT to communicate with the milk dispenser service.
# It sends a request to dispense a specific type and amount of milk, and waits for a response.
# params should contain "milk_type" ("whole", "oat", "almond", etc.) and "amount" (integer)
async def dispense_milk(params: dict):
    """Dispense milk using MQTT communication."""
    milk_type = params.get("milk_type", "whole")
    amount = params.get("amount", 50)
    print("Calling dispense_milk function")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        print("Connected with code", rc)
        client.subscribe("automation/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            print("Response:", json.dumps(payload, indent=2))
            response["data"] = payload
        except json.JSONDecodeError:
            print("Invalid JSON:", msg.payload.decode())

    payload = json.dumps({"milk_type": milk_type, "amount": amount})
    
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    print(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    client.publish("automation", payload, qos=1)
    print("Sent:", payload)

    timeout = params.get("timeout", 10)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        print("Timeout: No response from dispenser")
        return {
            "success": False,
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    print("[Dispenser] Final response:", json.dumps(response["data"], indent=2))
    return response["data"]


async def automation_test1(params: dict):
    """Test function 1 for automation service."""
    await asyncio.sleep(5)
    print("automation_test1 passed successfully")
    return {
        "success": True,
        "message": "automation_test1 passed successfully",
        "details": {
            "test_name": "automation_test1",
            "params_received": params,
            "duration_sec": 0.5,
            "service": "automation"
        }
    }

async def automation_test2(params: dict):
    """Test function 2 for automation service."""
    await asyncio.sleep(5)
    print("automation_test2 passed successfully")
    return {
        "success": True,
        "message": "automation_test2 passed successfully",
        "details": {
            "test_name": "automation_test2",
            "params_received": params,
            "duration_sec": 0.7,
            "service": "automation"
        }
    }


# Map function names to implementations
AUTOMATION_FUNCTIONS = {
    "heat_water": heat_water,
    "dispense_milk": dispense_milk,
    "automation_test1": automation_test1,
    "automation_test2": automation_test2,
    # Add more automation functions as needed
} 
