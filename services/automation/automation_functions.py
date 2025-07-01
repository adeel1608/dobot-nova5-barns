# services/automation/automation_functions.py
"""Automation functions for BARNS coffee brewing system."""
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
    time.sleep(10)
    milk_type = params.get("milk_type", "whole")
    amount = params.get("amount", 50)
    logger.info("Calling dispense_milk function")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        logger.info(f"Connected with code {rc}")
        client.subscribe("automation/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            logger.info(f"Response: {json.dumps(payload, indent=2)}")
            response["data"] = payload
        except json.JSONDecodeError:
            logger.info(f"Invalid JSON: {msg.payload.decode()}")

    payload = json.dumps({"milk_type": milk_type, "amount": amount})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    # client.publish("automation.dispense", payload, qos=1)
    client.publish("automation", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 100)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Dispenser] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully dispensed {amount}ml of {milk_type} milk",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "message": f"Failed to dispense milk: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }


# Slush Machoine
# This function uses MQTT to communicate with the milk dispenser service.
# It sends a request to dispense a specific type and amount of milk, and waits for a response.
# params should contain "milk_type" ("whole", "oat", "almond", etc.) and "amount" (integer)

async def slush_machine(params: dict):
    """Slush machine using MQTT communication."""
    slush_type = params.get("slush_type", "slush_1")
    cup_size = params.get("cup", "C12")
    
    print("Calling Slush mach function")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        logger.info(f"Connected with code {rc}")
        client.subscribe("automation/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            logger.info(f"Response: {json.dumps(payload, indent=2)}")
            response["data"] = payload
        except json.JSONDecodeError:
            logger.info(f"Invalid JSON: {msg.payload.decode()}")

    payload = json.dumps({"slush_type": slush_type, "cup": cup_size})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    client.publish("automation_slush", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 100)
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

    print(f"[Slush Machine] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully prepared {slush_type} slush in {cup_size} cup",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "message": f"Failed to prepare slush: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

async def coffee_machine(params: dict):
    """coffee machine using MQTT communication."""
    coffee_type = params.get("coffee_amount", "Slush_1")
    cup_size = params.get("cup", "C12")
    print("Calling Slush mach function")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        print(f"Connected with code {rc}")
        client.subscribe("automation/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            print(f"Response: {json.dumps(payload, indent=2)}")
            response["data"] = payload
        except json.JSONDecodeError:
            print(f"Invalid JSON: {msg.payload.decode()}")

    payload = json.dumps({"coffee_type": coffee_type, "cup": cup_size})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    client.publish("automation_coffee", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 100)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Coffee Machine] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully prepared {coffee_type} coffee in {cup_size} cup",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "message": f"Failed to prepare coffee: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }


# Map function names to implementations
AUTOMATION_FUNCTIONS = {
    "heat_water": heat_water,
    "dispense_milk": dispense_milk,
    "slush_machine": slush_machine,
    "coffee_machine": coffee_machine,
    # Add more automation functions as needed
} 

