# services/automation/automation_functions.py
"""Automation functions for BARNS coffee brewing system."""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from shared.logger import log

import asyncio
import time

#Ibrahim (Disoenser)
import paho.mqtt.client as mqtt
import json

async def dispense_hot_water(params: dict):
    """Dispense hot water using MQTT communication."""

    cups_dict = params["cups"]
    # Handle nested cups dictionary format
    if "cups" in params and isinstance(params["cups"], dict):
        
        # Extract cup type from first key (e.g., "cup_H9" or "cup_H12")
        cup_type = list(cups_dict.keys())[0]
        
        # Map cup type to calibration value
        if "cup_h9" in cup_type.lower():
            calibration = 2
        elif "cup_h12" in cup_type.lower():
            calibration = 1
        else:
            # Default to calibration 2 if unknown cup type
            log("ERROR", f"Unknown cup type: {cups_dict}, defaulting to calibration 2", service="automation")
            calibration = 2
    else:
        # Fallback to flat parameter format
        calibration = params.get("calibration", 2)
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_coffee_machine_hot_water/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"calibration": calibration})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_coffee_machine_hot_water", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from coffee machine", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from coffee machine",
            "message": "Timeout: No response from coffee machine"
        }
    client.loop_stop()
    client.disconnect()
    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully dispensed hot water (calibration={calibration})",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to dispense hot water: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }

# Milk Dispenser
# This function uses MQTT to communicate with the syrup dispenser service.
# It sends a request to dispense a specific type and amount of syrup, and waits for a response.
# params should contain "syrup_type" ("whole", "oat", "almond", etc.) and "amount" (integer)
# EX: example params: {"pump_number": 9, "amount": 15, "timeout": 300}
# OR: {"syrups": {3: 45.0}, "timeout": 300}
async def dispense_sauce(params: dict):
    """Dispense multiple syrups using MQTT communication."""
    # example params: {"syrups": {2: 5.0, 5: 16.0}, ...}
    # Loops through all pumps in the syrups dictionary
    
    # Extract syrups dictionary
    if "syrups" not in params or not isinstance(params["syrups"], dict):
        log("ERROR", "No syrups dictionary found in params", service="automation")
        return {
            "success": False,
            "error": "No syrups dictionary found in params",
            "message": "Invalid parameters: syrups dictionary required"
        }
    
    syrups_dict = params["syrups"]
    
    if not syrups_dict:
        return {
            "success": True,
            "message": "No syrups to dispense",
            "details": []
        }
    
    # Prepare for loop through all syrups
    all_results = []
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker (same as milk)
    username = params.get("username", "admin")
    password = params.get("password", "admin123")
    
    # Loop through each syrup pump
    for pump_key, amount in syrups_dict.items():
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 9
        
        response = {"data": None}

        def on_connect(client, userdata, flags, rc, props=None):
            client.subscribe("automation_syrup/response", qos=1)

        def on_message(client, userdata, msg):
            try:
                payload = json.loads(msg.payload.decode())
                response["data"] = payload
            except json.JSONDecodeError:
                pass

        # Create MQTT payload
        payload = json.dumps({"pump_number": pump_number, "amount": amount})
        client = mqtt.Client(protocol=mqtt.MQTTv311)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.on_message = on_message
        
        # Connect to external MQTT broker for syrup dispensing (same as milk)
        client.connect(mqtt_host, 1883, 60)
        
        client.loop_start()
        
        # Wait for connection and subscription to be established
        connection_timeout = 10
        connection_start = time.time()
        while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
            time.sleep(0.1)
        
        if not client.is_connected():
            log("ERROR", "Failed to connect to MQTT broker", service="automation")
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": "Failed to connect to MQTT broker"
            })
            continue
        
        # Give a moment for subscription to be processed
        time.sleep(0.5)
        
        # Send the message
        client.publish("automation_syrup", payload, qos=1)

        # Wait indefinitely for response (no timeout)
        while response["data"] is None:
            await asyncio.sleep(0.1)

        client.loop_stop()
        client.disconnect()

        
        # Store result for this pump
        mqtt_response = response["data"]
        if mqtt_response.get("status") == "success":
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": True,
                "message": f"Successfully dispensed {amount}g from syrup pump {pump_number}",
                "details": mqtt_response
            })
        else:
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": mqtt_response.get('error', 'Unknown error'),
                "details": mqtt_response
            })
    
    # Return combined results
    all_success = all(result["success"] for result in all_results)
    
    if all_success:
        return {
            "success": True,
            "message": f"Successfully dispensed {len(all_results)} syrups",
            "details": all_results
        }
    else:
        failed_count = sum(1 for result in all_results if not result["success"])
        return {
            "success": False,
            "error": f"{failed_count} syrup(s) failed to dispense",
            "message": f"Completed with {failed_count} failure(s) out of {len(all_results)} syrups",
            "details": all_results
        }



async def dispense_ice(params: dict):
    """Dispense ice using MQTT communication."""
    # example params: {"cups": {"cup_c16": 1.0}, "timeout": 300}
    # OR flat format: {"timer": 4, "timeout": 300}

    cups_dict = params["cups"]
    # Handle nested cups dictionary format
    if "cups" in params and isinstance(params["cups"], dict):
        
        # Extract cup type from first key (e.g., "cup_c7", "cup_c9", "cup_c12", "cup_c16")
        cup_type = list(cups_dict.keys())[0]
        
        # Map cup type to timer value
        cup_type_lower = cup_type.lower()
        if "cup_c7" in cup_type_lower:
            timer = 1
        elif "cup_c9" in cup_type_lower:
            timer = 1
        elif "cup_c12" in cup_type_lower:
            timer = 2
        elif "cup_c16" in cup_type_lower:
            timer = 2
        else:
            # Default to timer 0 if unknown cup type
            log("ERROR", f"Unknown cup type: {cups_dict}, defaulting to timer 0", service="automation")
            timer = 0
    else:
        # Fallback to flat parameter format
        timer = params.get("timer", 0)
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_ice/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"timer": timer})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message
    client.publish("automation_ice", payload, qos=1)

    timeout = params.get("timeout", 90)  # Reduced to allow buffer for routine service
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from dispenser", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    
    # Standardize the response format
    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully dispensed ice (timer={timer})",
            "details": mqtt_response
        }
    else:
        return {
            "success": False,
            "error": mqtt_response.get('error', 'Unknown error'),
            "message": f"Failed to dispense ice: {mqtt_response.get('error', 'Unknown error')}",
            "details": mqtt_response
        }


# EX:example params: {"pump_number": 1, "amount": 150, "timeout": 300}
# OR: {"milk": {'213411': 200.0}, "timeout": 300}
# OR: {"water": {5: 100.0}, "timeout": 300}  # Water uses pump 5
async def dispense_milk(params: dict):
    """Dispense multiple milk types or water using MQTT communication."""
    # example params: {"milk": {1: 260.0}, ...} or {"water": {5: 100.0}, ...}
    # Loops through all pumps in the milk/water dictionary
    # Both use the same milk dispenser hardware
    
    # Extract milk or water dictionary (both use milk dispenser hardware)
    milk_dict = None
    if "milk" in params and isinstance(params["milk"], dict):
        milk_dict = params["milk"]
    elif "water" in params and isinstance(params["water"], dict):
        milk_dict = params["water"]
    
    if not milk_dict:
        log("ERROR", "No milk or water dictionary found in params", service="automation")
        return {
            "success": False,
            "error": "No milk or water dictionary found in params",
            "message": "Invalid parameters: milk or water dictionary required"
        }
    
    if not milk_dict:
        return {
            "success": True,
            "message": "No milk or water to dispense",
            "details": []
        }
    
    # Prepare for loop through all milk/water pumps
    all_results = []
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    username = params.get("username", "admin")
    password = params.get("password", "admin123")
    
    # Loop through each pump (milk or water)
    for pump_key, amount in milk_dict.items():
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 1
        
        response = {"data": None}

        def on_connect(client, userdata, flags, rc, props=None):
            client.subscribe("automation_milk/response", qos=1)

        def on_message(client, userdata, msg):
            try:
                payload = json.loads(msg.payload.decode())
                response["data"] = payload
            except json.JSONDecodeError:
                pass

        # Create MQTT payload
        payload = json.dumps({"pump_number": pump_number, "amount": amount})
        client = mqtt.Client(protocol=mqtt.MQTTv311)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.on_message = on_message
        
        # Connect to external MQTT broker for dispensing
        client.connect(mqtt_host, 1883, 60)
        
        client.loop_start()
        
        # Wait for connection and subscription to be established
        connection_timeout = 10
        connection_start = time.time()
        while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
            time.sleep(0.1)
        
        if not client.is_connected():
            log("ERROR", "Failed to connect to MQTT broker", service="automation")
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": "Failed to connect to MQTT broker"
            })
            continue
        
        # Give a moment for subscription to be processed
        time.sleep(0.5)
        
        # Send the message
        client.publish("automation_milk", payload, qos=1)

        # Wait indefinitely for response (no timeout)
        while response["data"] is None:
            await asyncio.sleep(0.1)

        client.loop_stop()
        client.disconnect()

        
        # Store result for this pump
        mqtt_response = response["data"]
        if mqtt_response.get("status") == "success":
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": True,
                "message": f"Successfully dispensed {amount}g from pump {pump_number}",
                "details": mqtt_response
            })
        else:
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": mqtt_response.get('error', 'Unknown error'),
                "details": mqtt_response
            })
    
    # Return combined results
    all_success = all(result["success"] for result in all_results)
    
    if all_success:
        return {
            "success": True,
            "message": f"Successfully dispensed {len(all_results)} item(s)",
            "details": all_results
        }
    else:
        failed_count = sum(1 for result in all_results if not result["success"])
        return {
            "success": False,
            "error": f"{failed_count} dispense(s) failed",
            "message": f"Completed with {failed_count} failure(s) out of {len(all_results)} items",
            "details": all_results
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
    
    log("DEBUG", "Calling Slush mach function", service="automation")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_slush/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"slush_type": slush_type, "cup": cup_size})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_slush", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("DEBUG", "Timeout: No response from dispenser", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    log("DEBUG", f"[Slush Machine] Final response: {json.dumps(response['data'], indent=2)}", service="automation")
    
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
        coffee_t = params.get("coffee_t", 3)
    
    if coffee_t == 1:
        slot_number = 3
    elif coffee_t == 2:
        slot_number = 1
    elif coffee_t == 3:
        coffee_t = 1
        slot_number = 2
    else:
        raise ValueError(f"Invalid triple shot not supported: {coffee_t}")
        
    # slot_number = params.get("slot_number", 1)
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        log("DEBUG", f"Connected with code {rc}", service="automation")
        client.subscribe("automation_coffee_machine/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            log("DEBUG", f"Response: {json.dumps(payload, indent=2)}", service="automation")
            response["data"] = payload
        except json.JSONDecodeError:
            log("DEBUG", f"Invalid JSON: {msg.payload.decode()}", service="automation")

    payload = json.dumps({"coffee_t": coffee_t, "slot_number": slot_number})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_coffee_machine", payload, qos=1)

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
        log("ERROR", "Timeout: No response from dispenser", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    
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

async def coffee_machine_wait(params: dict):
    """coffee machine using MQTT communication."""
    # coffee_t is the number of the shots 1,2
    
    # Handle nested espresso dictionary format
    if "espresso" in params and isinstance(params["espresso"], dict):
        espresso_dict = params["espresso"]
        # Extract amount from first value (ignore the key name like "espresso_shot_double")
        coffee_t = int(list(espresso_dict.values())[0])  # Get first value, convert to int
    else:
        # Fallback to flat parameter format
        coffee_t = params.get("coffee_t", 3)
    
    if coffee_t == 1:
        slot_number = 3
    elif coffee_t == 2:
        slot_number = 1
    elif coffee_t == 3:
        coffee_t = 1
        slot_number = 2
    else:
        raise ValueError(f"Invalid triple shot not supported: {coffee_t}")
        
    # slot_number = params.get("slot_number", 1)
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        log("DEBUG", f"Connected with code {rc}", service="automation")
        client.subscribe("automation_coffee_machine/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            log("DEBUG", f"Response: {json.dumps(payload, indent=2)}", service="automation")
            response["data"] = payload
        except json.JSONDecodeError:
            log("DEBUG", f"Invalid JSON: {msg.payload.decode()}", service="automation")

    payload = json.dumps({"coffee_t": coffee_t, "slot_number": slot_number})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_coffee_machine", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()
    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from dispenser", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from dispenser",
            "message": "Timeout: No response from dispenser"
        }
    client.loop_stop()
    client.disconnect()

    
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
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_grinding/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"shots_number": shots_number})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_grinding", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from grinder", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from grinder",
            "message": "Timeout: No response from grinder"
        }
    client.loop_stop()
    client.disconnect()

    
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
    # example params: {"espresso": {"espresso_shot_single": 1.0}} or {"espresso": {"espresso_shot_double": 2.0}}
    
    # Handle nested espresso dictionary format
    if "espresso" in params and isinstance(params["espresso"], dict):
        espresso_dict = params["espresso"]
        # Extract amount from first value (ignore the key name like "espresso_shot_single")
        espresso_shots = int(list(espresso_dict.values())[0])  # Get first value, convert to int
    else:
        # Fallback to flat parameter format
        espresso_shots = params.get("tampering", 1)
    
    # Map espresso shots to calibration
    # If espresso = 1, send tampering: 1, calibration: 1
    # If espresso = 2, send tampering: 1, calibration: 2
    tampering = 1  # Always 1
    if espresso_shots == 1:
        calibration = 2000        ##set values here in ms
    elif espresso_shots == 2:
        calibration = 1900        ##set values here in ms
    else:
        # Default to calibration 1 if unknown shot count
        log("ERROR", f"Unknown espresso shot count: {espresso_shots}, defaulting to calibration 1", service="automation")
        calibration = 1
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_tampering/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"tampering": tampering, "calibration": calibration})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"),
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message

    mqtt_host = params.get("mqtt_host", "rabbitmq")
    client.connect(mqtt_host, 1883, 60)

    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_tampering", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from tampering machine", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from tampering machine",
            "message": "Timeout: No response from tampering machine"
        }

    client.loop_stop()
    client.disconnect()


    mqtt_response = response["data"]
    if mqtt_response.get("status") == "success":
        return {
            "success": True,
            "message": f"Successfully completed tampering operation (calibration={calibration})",
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
    time.sleep(60)
    return {
        "success": True,
        "message": "Successfully completed automation test",
        "details": "Automation test completed"
    }

async def froth_milk(params: dict):
    """Froth milk using MQTT communication."""
    # example params: {"temperature": "standard", "timeout": 300}
    # OR nested format: {"temperature": {"regular_temperature": 73.0}, "timeout": 300}
    
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
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_frother/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    # Send temperature to frother
    payload = json.dumps({"temperature": temperature})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for frother (your Arduino setup)
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message to frother topic
    client.publish("automation_frother", payload, qos=1)

    timeout = params.get("timeout", 210)  # Extended timeout for frothing (200s + buffer)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from frother", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from frother",
            "message": "Timeout: No response from frother"
        }
    client.loop_stop()
    client.disconnect()

    
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
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_frother_init/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    # Fixed payload for frother initialization
    payload = json.dumps({"frother_init": 1})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for frother (your Arduino setup)
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message to frother init topic
    client.publish("automation_frother_init", payload, qos=1)

    timeout = params.get("timeout", 90)  # Timeout for frother init
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from frother", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from frother",
            "message": "Timeout: No response from frother"
        }
    client.loop_stop()
    client.disconnect()

    
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
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_clean_frother/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    # Fixed payload for cleaning frother
    payload = json.dumps({"clean_frother": 1})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message to clean frother topic
    client.publish("automation_clean_frother", payload, qos=1)
    return {
            "success": True,
            "message": "Successfully cleaned frother",
            "details": "froth command sent"
        }
    timeout = params.get("timeout", 120)  # Timeout for clean operation
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from frother cleaning", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from frother cleaning",
            "message": "Timeout: No response from frother cleaning"
        }
    client.loop_stop()
    client.disconnect()

    
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
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_rinser/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"rinser": rinser_state, "timer": timer})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")  # Use 'rabbitmq' service name
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    client.publish("automation_rinser", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", "Timeout: No response from rinser machine", service="automation")
        return {
            "success": False,
            "error": "Timeout: No response from rinser machine",
            "message": "Timeout: No response from rinser machine"
        }

    client.loop_stop()
    client.disconnect()


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
    
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_dispensing/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    # Format command for dispensing system: ingredient_weight
    command = f"{ingredient}_{weight}"
    payload = json.dumps({
        "ingredient": ingredient, 
        "weight": weight, 
        "motor": motor,
        "command": command
    })
    
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to external MQTT broker for dispensing (your Arduino setup)
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    client.connect(mqtt_host, 1883, 60)
    
    client.loop_start()
    
    # Wait for connection and subscription to be established
    connection_timeout = 10
    connection_start = time.time()
    while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
        time.sleep(0.1)
    
    if not client.is_connected():
        log("ERROR", "Failed to connect to MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message to dispensing topic
    client.publish("automation_dispensing", payload, qos=1)

    # Give a moment for the message to be sent
    await asyncio.sleep(0.5)
    
    client.loop_stop()
    client.disconnect()

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
    "dispense_sauce": dispense_sauce,
    "dispense_milk": dispense_milk,
    "dispense_ingredient": dispense_ingredient,
    "slush_machine": slush_machine,
    "coffee_machine": coffee_machine,
    "coffee_machine_wait": coffee_machine_wait,
    "grinding_machine": grinding_machine,   
    "tampering_machine" : tampering_machine,
    "dispense_ice": dispense_ice,
    "froth_milk": froth_milk,
    "initialize_frother": initialize_frother,
    "clean_frother": clean_frother,
    "rinser_machine": rinser_machine,
    "automation_test": automation_test,
    "dispense_hot_water": dispense_hot_water,
    # Add more automation functions as needed
}