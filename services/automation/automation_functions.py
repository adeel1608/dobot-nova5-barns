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
# This function uses MQTT to communicate with the syrup dispenser service.
# It sends a request to dispense a specific type and amount of syrup, and waits for a response.
# params should contain "syrup_type" ("whole", "oat", "almond", etc.) and "amount" (integer)
async def dispense_syrup(params: dict):
    """Dispense syrup using MQTT communication."""
    # example params: {"syrup_type": "whole", "amount": 150, "timeout": 300}
    syrup_type = params.get("syrup_type", "whole")
    amount = params.get("amount", 20)
    logger.info("Calling dispense_syrup function")
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

    #
    payload = json.dumps({"syrup_type": syrup_type, "amount": amount})
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
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message
    client.publish("automation_syrup", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 90)  # Reduced to allow buffer for routine service
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
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
            "message": f"Successfully dispensed {amount}ml of {syrup_type} syrup",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to dispense syrup: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }



async def dispense_ice(params: dict):
    """Dispense syrup using MQTT communication."""
    # example params: {"syrup_type": "whole", "amount": 150, "timeout": 300}
    ice = params.get("ice", 1)

    logger.info("Calling dispense_syrup function")
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

    #
    payload = json.dumps({"ice": ice})
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
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message
    client.publish("automation_ice", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 90)  # Reduced to allow buffer for routine service
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
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
            "message": f"Successfully dispensed ice",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to dispense syrup: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }


# EX:example params: {"milk_type": "whole", "amount": 150, "timeout": 300}
async def dispense_milk(params: dict):
    """Dispense milk using MQTT communication."""
    # example params: {"milk_type": "whole", "amount": 150, "timeout": 300}
    logger.info(f"Calling dispense_milk function with params:{params}")
    milk_type = params.get("milk_type", "whole")
    amount = params.get("amount", 150)
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

    #
    payload = json.dumps({"milk_type": milk_type, "amount": amount})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for dispensing (your Arduino setup)
    mqtt_host = params.get("mqtt_host", "192.168.200.233")  # Use external MQTT broker
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message
    client.publish("automation_milk", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 90)  # Reduced to allow buffer for routine service
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
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
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to dispense milk: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }


# Slush Machoine
# This function uses MQTT to communicate with the milk dispenser service.
# It sends a request to dispense a specific type and amount of milk, and waits for a response.
# params should contain "milk_type" ("whole", "oat", "almond", etc.) and "amount" (integer)

# EX: example params: {"slush_type": "slush_1", "cup": "C12", "timeout": 300}
async def slush_machine(params: dict):
    """Slush machine using MQTT communication."""
    # example params: {"slush_type": "slush_1", "cup": "C12", "timeout": 300}
    slush_type = params.get("slush_type", "slush_2")
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
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_slush", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        print("Timeout: No response from dispenser")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
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
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to prepare slush: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }
# cooffee machine commented until Ammar finshes the issue with Registers
# EX: example params: {"cup": "C12", "shots_number": 1, "timeout": 300}
async def coffee_machine(params: dict):
    """coffee machine using MQTT communication."""
    # coffee_type is the number of the shots 1,2
    coffee_t = params.get("coffee_t", 1)
    slot_number = params.get("slot_number", 1)
    print("Calling Coffee machine function")
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

    payload = json.dumps({"slot_number": slot_number, "coffee_t": coffee_t})
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
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_coffee_machine", payload, qos=1)
    logger.info(f"Sent: {payload}")

    # timeout = params.get("timeout", 120)
    # start_time = time.time()
    return {
            "success": True,
            "message": f"Successfully prepared {coffee_t} coffee in {slot_number} slot",
            "details": "Processing coffee"
        }
    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from dispenser")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
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
            "message": f"Successfully prepared {coffee_t} coffee in {slot_number} slot",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to prepare coffee: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

# EX: example params: {"shots_number": 1, "timeout": 300}
async def grinding_machine(params: dict):
    """Grinding machine using MQTT communication."""
    # example params: {"shots_number": 1, "timeout": 300}
    shots_number = params.get("shots_number", 1)
    logger.info("Calling grinding machine function")
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

    payload = json.dumps({"shots_number": shots_number})
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
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_grinding", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from grinder")
        return {
            "success": False,
            "error": "Timeout: No response from grinder",
            "message": "Timeout: No response from grinder"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Grinding Machine] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully ground coffee for {shots_number} shot(s)",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to grind coffee: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

# EX: example params: {"tampering": 1}
async def tampering_machine(params: dict):
    """tampering machine using MQTT communication."""
    # example params: {"tampering": 1} tampering is 1,2,3 for coffee shots number
    tampering = params.get("tampering", 1)
    logger.info("Calling tampering machine function")
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

    payload = json.dumps({"tampering": tampering})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"),
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message

    mqtt_host = params.get("mqtt_host", "rabbitmq")
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)

    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_tampering", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from tampering machine")
        return {
            "success": False,
            "error": "Timeout: No response from tampering machine",
            "message": "Timeout: No response from tampering machine"
        }

    client.loop_stop()
    client.disconnect()

    logger.info(f"[tampering Machine] Final response: {json.dumps(response['data'], indent=2)}")

    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": "Successfully completed tampering operation",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to complete tampering: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

async def automation_test(params: dict):
    """Automation test using MQTT communication."""
    # example params: {"automation_test": 1}
    logger.info("Starting automation test function")
    time.sleep(100)
    logger.info("Ending automation test function")
    return {
        "success": True,
        "message": "Successfully completed automation test",
        "details": "Automation test completed"
    }

async def dispense_ingredient(params: dict):
    """Dispense ingredient using MQTT communication."""
    ingredient = params.get("ingredient", "sauce")
    weight = params.get("weight", 10)
    motor = params.get("motor", "sauce1")
    
    logger.info("Calling dispense_ingredient function")
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

    # Format command for dispensing system: ingredient_weight
    command = f"{ingredient}_{weight}"
    payload = json.dumps({
        "ingredient": ingredient, 
        "weight": weight, 
        "motor": motor,
        "command": command
    })
    
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for dispensing (your Arduino setup)
    mqtt_host = params.get("mqtt_host", "192.168.200.233")  # Use external MQTT broker
    logger.info(f"Connecting to MQTT broker at {mqtt_host}:1883")
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        logger.error("Failed to connect to MQTT broker")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message to dispensing topic
    client.publish("automation_dispensing", payload, qos=1)
    logger.info(f"Sent: {payload}")

    # Give a moment for the message to be sent
    await asyncio.sleep(0.5)
    
    client.loop_stop()
    client.disconnect()

    logger.info(f"✅ Dispensed {weight}g of {ingredient}")
    return {
        "success": True,
        "message": f"Dispensed {weight}g of {ingredient} via {motor}",
        "details": {
            "ingredient": ingredient,
            "weight": weight,
            "motor": motor,
            "command": command
        }
    }

# Map function names to implementations
AUTOMATION_FUNCTIONS = {
    "heat_water": heat_water,
    "dispense_syrup": dispense_syrup,
    "dispense_milk": dispense_milk,
    "dispense_ingredient": dispense_ingredient,
    "slush_machine": slush_machine,
    "coffee_machine": coffee_machine,
    "grinding_machine": grinding_machine,
    "tampering_machine" : tampering_machine,
    "dispense_ice": dispense_ice,
    "automation_test": automation_test,
    # Add more automation functions as needed
}