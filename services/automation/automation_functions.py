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

async def dispnese_hot_water(params: dict):
    """Heat water to specified temperature."""
    target_temp = params.get("target_temp_c", 93)
    volume_ml = params.get("volume_ml", 250)
    ## Parameter: {'water': {'hot_water': 160.0}, 'cups': {'cup_H9': 1.0}, 'temperature': {'regular_temperature': 73.0}, 'espresso': {'espresso_shot_single': 1.0}}
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
# EX: example params: {"pump_number": 9, "amount": 15, "timeout": 300}
# OR: {"syrups": {3: 45.0}, "timeout": 300}
async def dispense_syrup(params: dict):
    """Dispense syrup using MQTT communication."""
    # example params: {"pump_number": 9, "amount": 15, "timeout": 300}
    # OR nested format: {"syrups": {3: 45.0}, "timeout": 300} where 3 is pump number (9-23)
    logger.info(f"Calling dispense_syrup function with params:{params}")
    
    # Handle nested syrups dictionary format
    if "syrups" in params and isinstance(params["syrups"], dict):
        syrups_dict = params["syrups"]
        # Extract the first key-value pair (pump_number: amount)
        pump_key = list(syrups_dict.keys())[0]
        amount = syrups_dict[pump_key]
        # Convert pump_key to integer (handle both int and string keys)
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 9
    else:
        # Fallback to flat parameter format
        pump_number = params.get("pump_number", 9)
        amount = params.get("amount", 20)
    
    logger.info(f"Dispensing {amount}g from syrup pump {pump_number}")
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
    payload = json.dumps({"pump_number": pump_number, "amount": amount})
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
            "message": f"Successfully dispensed {amount}g from syrup pump {pump_number}",
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
    """Dispense ice using MQTT communication."""
    # example params: {"ice": 8, "timeout": 300}
    # OR nested format: {"ice": {"ice_cubes_16oz": 11.0}, "timeout": 300}
    logger.info(f"Calling dispense_ice function with params:{params}")
    
    # Handle nested ice dictionary format
    if "ice" in params and isinstance(params["ice"], dict):
        ice_dict = params["ice"]
        # Extract amount from first value (ignore the key name like "ice_cubes_16oz")
        ice = int(list(ice_dict.values())[0])  # Get first value, convert to int
    else:
        # Fallback to flat parameter format (direct integer value)
        ice = params.get("ice", 1)
    
    logger.info(f"Dispensing {ice} ice cubes")
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


# EX:example params: {"pump_number": 1, "amount": 150, "timeout": 300}
# OR: {"milk": {'213411': 200.0}, "timeout": 300}
async def dispense_milk(params: dict):
    """Dispense milk using MQTT communication."""
    # example params: {"pump_number": 1, "amount": 150, "timeout": 300}
    # OR nested format: {"milk": {'213411': 200.0}, "timeout": 300} where key is pump number (1-8)
    logger.info(f"Calling dispense_milk function with params:{params}")
    
    # Handle nested milk dictionary format
    if "milk" in params and isinstance(params["milk"], dict):
        milk_dict = params["milk"]
        # Extract the first key-value pair (pump_number: amount)
        pump_key = list(milk_dict.keys())[0]
        amount = milk_dict[pump_key]
        # Convert pump_key to integer (handle both int and string keys)
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 1
    else:
        # Fallback to flat parameter format
        pump_number = params.get("pump_number", 1)
        amount = params.get("amount", 150)
    
    logger.info(f"Dispensing {amount}g from milk pump {pump_number}")
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
    payload = json.dumps({"pump_number": pump_number, "amount": amount})
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
            "message": f"Successfully dispensed {amount}g from milk pump {pump_number}",
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
# EX: example params: {"coffee_t": 1, "slot_number": 1, "timeout": 300}
# OR: {"espresso": {"espresso_shot_double": 2.0}, "slot_number": 1, "timeout": 300}
async def coffee_machine(params: dict):
    """coffee machine using MQTT communication."""
    # coffee_t is the number of the shots 1,2
    # Handle nested espresso dictionary format
    if "espresso" in params and isinstance(params["espresso"], dict):
        espresso_dict = params["espresso"]
        # Extract amount from first value (ignore the key name like "espresso_shot_double")
        coffee_t = int(list(espresso_dict.values())[0])  # Get first value, convert to int
    else:
        # Fallback to flat parameter format
        coffee_t = params.get("coffee_t", 1)
    
    if coffee_t == 1:
        slot_number = 3
    elif coffee_t == 2:
        slot_number = 1
    else:
         raise ValueError("Invalid triple shot not supported: {coffee_t}")
        
    # slot_number = params.get("slot_number", 1)
    logger.info(f"Calling Coffee machine function with coffee_t: {coffee_t}, slot_number: {slot_number}")
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

    payload = json.dumps({"coffee_t": coffee_t, "slot_number": slot_number})
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
# OR: {"espresso": {"espresso_shot_double": 2.0}, "timeout": 300}
async def grinding_machine(params: dict):
    """Grinding machine using MQTT communication."""
    # example params: {"shots_number": 1, "timeout": 300}
    # OR nested format: {"espresso": {"espresso_shot_double": 2.0}, "timeout": 300}
    
    # Handle nested espresso dictionary format
    if "espresso" in params and isinstance(params["espresso"], dict):
        espresso_dict = params["espresso"]
        # Extract amount from first value (ignore the key name like "espresso_shot_double")
        shots_number = int(list(espresso_dict.values())[0])  # Get first value, convert to int
    else:
        # Fallback to flat parameter format
        shots_number = params.get("shots_number", 1)
    
    logger.info(f"Calling grinding machine function with shots_number: {shots_number}")
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

async def froth_milk(params: dict):
    """Froth milk using MQTT communication."""
    # example params: {"temperature": "standard", "timeout": 300}
    # OR nested format: {"temperature": {"regular_temperature": 73.0}, "timeout": 300}
    logger.info(f"Calling froth_milk function with params:{params}")
    
    # Handle nested temperature dictionary format
    if "temperature" in params and isinstance(params["temperature"], dict):
        temp_dict = params["temperature"]
        # Extract temperature value (could be numeric or string)
        temp_value = list(temp_dict.values())[0]
        
        # Map temperature ranges to frother settings
        if isinstance(temp_value, (int, float)):
            # Numeric temperature in Celsius
            if temp_value <= 60:
                temperature = "kids"
            elif temp_value <= 75:
                temperature = "standard"
            else:
                temperature = "extra_hot"
        else:
            # String temperature
            temperature = str(temp_value).lower()
    else:
        # Fallback to flat parameter format
        temperature = params.get("temperature", "standard")
    
    logger.info(f"Frothing milk at temperature: {temperature}")
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

    # Send temperature to frother
    payload = json.dumps({"temperature": temperature})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for frother (your Arduino setup)
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
    
    # Now send the message to frother topic
    client.publish("automation_frother", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 210)  # Extended timeout for frothing (200s + buffer)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from frother")
        return {
            "success": False,
            "error": "Timeout: No response from frother",
            "message": "Timeout: No response from frother"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Frother] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully frothed milk at {temperature} temperature",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to froth milk: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

async def initialize_frother(params: dict):
    """Initialize frother using MQTT communication."""
    # This function doesn't use any parameters from params dict
    # It just sends a frother_init command to the MQTT broker
    logger.info("Calling frother_init function")
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

    # Fixed payload for frother initialization
    payload = json.dumps({"frother_init": 1})
    logger.info("Calling MQTT")
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for frother (your Arduino setup)
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
    
    # Now send the message to frother init topic
    client.publish("automation_frother_init", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 90)  # Timeout for frother init
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from frother")
        return {
            "success": False,
            "error": "Timeout: No response from frother",
            "message": "Timeout: No response from frother"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Frother] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": "Successfully initialized frother",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to initialize frother: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

async def clean_frother(params: dict):
    """Clean frother using MQTT communication."""
    # This function ignores params and sends a fixed payload
    logger.info("Calling clean_frother function")
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

    # Fixed payload for cleaning frother
    payload = json.dumps({"clean_frother": 1})
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
    
    # Now send the message to clean frother topic
    client.publish("automation_clean_frother", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 120)  # Timeout for clean operation
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from frother cleaning")
        return {
            "success": False,
            "error": "Timeout: No response from frother cleaning",
            "message": "Timeout: No response from frother cleaning"
        }
    client.loop_stop()
    client.disconnect()

    logger.info(f"[Clean Frother] Final response: {json.dumps(response['data'], indent=2)}")
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": "Successfully cleaned frother",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to clean frother: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

async def rinser_machine(params: dict):
    """Rinser machine using MQTT communication."""
    # example params: {"rinser": 1, "timer": 0, "timeout": 300}
    rinser_state = params.get("rinser", 1)
    timer = params.get("timer", 0)
    
    logger.info(f"Calling rinser machine function with rinser_state: {rinser_state}, timer: {timer}")
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

    payload = json.dumps({"rinser": rinser_state, "timer": timer})
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
    
    client.publish("automation_rinser", payload, qos=1)
    logger.info(f"Sent: {payload}")

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        logger.info("Timeout: No response from rinser machine")
        return {
            "success": False,
            "error": "Timeout: No response from rinser machine",
            "message": "Timeout: No response from rinser machine"
        }

    client.loop_stop()
    client.disconnect()

    logger.info(f"[Rinser Machine] Final response: {json.dumps(response['data'], indent=2)}")

    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": "Successfully completed rinser operation",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to complete rinser operation: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
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
    "dispnese_hot_water": dispnese_hot_water,
    "dispense_syrup": dispense_syrup,
    "dispense_milk": dispense_milk,
    "dispense_ingredient": dispense_ingredient,
    "slush_machine": slush_machine,
    "coffee_machine": coffee_machine,
    "grinding_machine": grinding_machine,
    "tampering_machine" : tampering_machine,
    "dispense_ice": dispense_ice,
    "froth_milk": froth_milk,
    "initialize_frother": initialize_frother,
    "clean_frother": clean_frother,
    "rinser_machine": rinser_machine,
    "automation_test": automation_test,
    # Add more automation functions as needed
}