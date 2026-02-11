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
import numpy as np

# MQTT Broker Fallback Configuration
MQTT_BROKER_PRIMARY = "rabbitmq"  # Docker/K8s service name
MQTT_PORT_PRIMARY = 1883
MQTT_BROKER_FALLBACK = "192.168.200.109"  # External Kubernetes NodePort
MQTT_PORT_FALLBACK = 30673

def connect_mqtt_with_fallback(client, preferred_host="rabbitmq", preferred_port=1883, timeout=10):
    """
    Connect to MQTT broker with automatic fallback support.
    
    Tries in order:
    1. Preferred host/port (usually 'rabbitmq' for Docker, or custom from params)
    2. Fallback to Kubernetes external broker if preferred fails
    
    Args:
        client: MQTT client instance
        preferred_host: First broker to try
        preferred_port: Port for first broker
        timeout: Connection timeout per broker
    
    Returns:
        tuple: (success: bool, connected_host: str, connected_port: int)
    """
    brokers = [
        (preferred_host, preferred_port, "preferred"),
        (MQTT_BROKER_FALLBACK, MQTT_PORT_FALLBACK, "fallback")
    ]
    
    for broker, port, label in brokers:
        connected = {"status": False}
        
        def on_connect_check(client, userdata, flags, rc):
            if rc == 0:
                connected["status"] = True
        
        try:
            log("INFO", f"Attempting {label} MQTT broker ({broker}:{port})", service="automation")
            
            original_on_connect = client.on_connect
            client.on_connect = on_connect_check
            
            client.connect(broker, port, 60)
            client.loop_start()
            
            start = time.time()
            while not connected["status"] and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if connected["status"]:
                client.on_connect = original_on_connect
                log("INFO", f"Connected to {label} broker: {broker}:{port}", service="automation")
                return True, broker, port
            else:
                log("WARNING", f"Connection timeout for {label} broker", service="automation")
                client.loop_stop()
                client.on_connect = original_on_connect
                
        except Exception as e:
            log("WARNING", f"Failed to connect to {label} broker: {e}", service="automation")
            try:
                client.loop_stop()
            except:
                pass
    
    log("ERROR", "Failed to connect to any MQTT broker", service="automation")
    return False, None, None

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
    
    # Connect with fallback support
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to dispense hot water: {error_msg}",
            "details": mqtt_response
        }

# Milk Dispenser
# This function uses MQTT to communicate with the syrup dispenser service.
# It sends a request to dispense a specific type and amount of syrup, and waits for a response.
# params should contain "syrup_type" ("whole", "oat", "almond", etc.) and "amount" (integer)
# EX: example params: {"pump_number": 9, "amount": 15, "timeout": 75}
# OR: {"syrups": {3: 45.0}, "timeout": 75}
# Note: Timeout defaults to 75s. After timeout, assumes success to avoid crashing recipe.
async def dispense_sauce(params: dict):
    """Dispense multiple sauces using MQTT communication."""
    # example params: {"sauce": {12: 5.0, 16: 16.0}, ...}
    # Loops through all pumps in the sauces dictionary
    
    log("INFO", f"[DISPENSE-SAUCE] Called with params: {params}", service="automation")
    
    # Extract sauces dictionary
    if "sauce" not in params or not isinstance(params["sauce"], dict):
        log("ERROR", "No sauces dictionary found in params", service="automation")
        return {
            "success": False,
            "error": "No sauces dictionary found in params",
            "message": "Invalid parameters: sauces dictionary required"
        }
    
    sauces_dict = params["sauce"]
    log("INFO", f"[DISPENSE-SAUCE] Extracted sauces dict: {sauces_dict}", service="automation")
    
    if not sauces_dict:
        return {
            "success": True,
            "message": "No sauces to dispense",
            "details": []
        }
    
    # Prepare for loop through all sauces
    all_results = []
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker (same as milk)
    username = params.get("username", "admin")
    password = params.get("password", "admin123")
    
    # Loop through each sauce pump
    for pump_key, amount in sauces_dict.items():
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 9
        
        log("INFO", f"[DISPENSE-SAUCE] Starting dispense: pump_key={pump_key}, pump_number={pump_number}, amount={amount}", service="automation")
        
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
        
        # Connect with fallback support
        mqtt_port = params.get("mqtt_port", 1883)
        success_conn, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
        
        if not success_conn:
            log("ERROR", "Failed to connect to any MQTT broker", service="automation")
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": "Failed to connect to MQTT broker"
            })
            continue
        
        # Give a moment for subscription to be processed
        time.sleep(0.5)
        
        # Send the message (sauce uses milk topic - same CAN dispenser handles both)
        client.publish("automation_milk", payload, qos=1)

        # Wait for response with timeout (default 75 seconds)
        response_timeout = params.get("timeout", 75)
        response_start = time.time()
        log("INFO", f"[DISPENSE-SAUCE] Waiting for MQTT response from pump {pump_number} (timeout: {response_timeout}s)", service="automation")
        
        while response["data"] is None:
            if (time.time() - response_start) > response_timeout:
                log("WARNING", f"[DISPENSE-SAUCE] Timeout waiting for response from pump {pump_number} after {response_timeout}s - Assuming success to continue recipe", service="automation")
                client.loop_stop()
                client.disconnect()
                all_results.append({
                    "pump_number": pump_number,
                    "amount": amount,
                    "success": True,
                    "message": f"Assumed success after {response_timeout}s timeout (no hardware response)",
                    "timeout": True
                })
                break  # Skip to next pump
            await asyncio.sleep(0.1)
        
        # Check if we got a response or timed out
        if response["data"] is None:
            continue  # Already added timeout result as success, move to next pump

        client.loop_stop()
        client.disconnect()

        
        # Store result for this pump
        mqtt_response = response["data"]
        log("INFO", f"[DISPENSE-SAUCE] MQTT response for pump {pump_number}: {mqtt_response}", service="automation")
        
        if mqtt_response.get("status") == "success":
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": True,
                "message": f"Successfully dispensed {amount}g from sauce pump {pump_number}",
                "details": mqtt_response
            })
            log("INFO", f"[DISPENSE-SAUCE] SUCCESS: Pump {pump_number} dispensed {amount}g", service="automation")
        else:
            # Use error field if present, otherwise use status field
            error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": error_msg,
                "details": mqtt_response
            })
            log("ERROR", f"[DISPENSE-SAUCE] FAILED: Pump {pump_number} failed to dispense {amount}g - Error: {error_msg}", service="automation")
    
    # Return combined results
    all_success = all(result["success"] for result in all_results)
    
    if all_success:
        log("INFO", f"[DISPENSE-SAUCE] All {len(all_results)} dispenses successful", service="automation")
        return {
            "success": True,
            "message": f"Successfully dispensed {len(all_results)} sauces",
            "details": all_results
        }
    else:
        failed_count = sum(1 for result in all_results if not result["success"])
        failed_pumps = [r["pump_number"] for r in all_results if not r["success"]]
        log("ERROR", f"[DISPENSE-SAUCE] {failed_count} of {len(all_results)} dispenses failed. Failed pumps: {failed_pumps}", service="automation")
        log("ERROR", f"[DISPENSE-SAUCE] Full results: {all_results}", service="automation")
        return {
            "success": False,
            "error": f"{failed_count} sauce(s) failed to dispense",
            "message": f"Completed with {failed_count} failure(s) out of {len(all_results)} sauces",
            "details": all_results
        }

async def dispense_syrup(params: dict):
    """Dispense multiple syrups and/or water using MQTT communication."""
    # example params: {"syrups": {2: 5.0, 5: 16.0}, "water": {1: 100.0}, ...}
    # Loops through all pumps in the syrups/water dictionaries
    # Water uses milk dispenser hardware but is managed here
    
    log("INFO", f"[DISPENSE-SYRUP] Called with params: {json.dumps(params, default=str)}", service="automation")
    
    # Collect all items to dispense (both syrups and water)
    items_to_dispense = []
    
    # Check for syrups
    if "syrups" in params and isinstance(params["syrups"], dict) and params["syrups"]:
        for pump_key, amount in params["syrups"].items():
            items_to_dispense.append({
                "pump_key": pump_key,
                "amount": amount,
                "item_type": "syrup",
                "mqtt_topic": "automation_syrup",
                "mqtt_response_topic": "automation_syrup/response"
            })
        log("INFO", f"[DISPENSE-SYRUP] Extracted {len(params['syrups'])} syrups: {params['syrups']}", service="automation")
    
    # Check for water
    if "water" in params and isinstance(params["water"], dict) and params["water"]:
        for pump_key, amount in params["water"].items():
            items_to_dispense.append({
                "pump_key": pump_key,
                "amount": amount,
                "item_type": "water",
                "mqtt_topic": "automation_syrup",  # Water pump 1 is a syrup motor
                "mqtt_response_topic": "automation_syrup/response"
            })
        log("INFO", f"[DISPENSE-SYRUP] Extracted {len(params['water'])} water pumps: {params['water']}", service="automation")
    
    if not items_to_dispense:
        log("ERROR", "No syrups or water dictionary found in params", service="automation")
        return {
            "success": False,
            "error": "No syrups or water dictionary found in params",
            "message": "Invalid parameters: syrups or water dictionary required"
        }
    
    log("INFO", f"[DISPENSE-SYRUP] Processing {len(items_to_dispense)} total items (syrups + water)", service="automation")
    
    # Prepare for loop through all items (syrups and water)
    all_results = []
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    username = params.get("username", "admin")
    password = params.get("password", "admin123")
    
    # Loop through each item (syrup or water)
    for item in items_to_dispense:
        pump_key = item["pump_key"]
        amount = item["amount"]
        item_type = item["item_type"]
        mqtt_topic = item["mqtt_topic"]
        mqtt_response_topic = item["mqtt_response_topic"]
        
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else (9 if item_type == "syrup" else 1)
        
        log("INFO", f"[DISPENSE-{item_type.upper()}] Starting dispense: pump_key={pump_key}, pump_number={pump_number}, amount={amount}", service="automation")
        
        response = {"data": None}

        def on_connect(client, userdata, flags, rc, props=None):
            client.subscribe(mqtt_response_topic, qos=1)

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
        
        # Connect with fallback support
        mqtt_port = params.get("mqtt_port", 1883)
        success_conn, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
        
        if not success_conn:
            log("ERROR", "Failed to connect to any MQTT broker", service="automation")
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "item_type": item_type,
                "success": False,
                "error": "Failed to connect to MQTT broker"
            })
            continue
        
        # Give a moment for subscription to be processed
        time.sleep(0.5)
        
        # Send the message
        client.publish(mqtt_topic, payload, qos=1)

        # Wait for response with timeout (default 75 seconds)
        response_timeout = params.get("timeout", 75)
        response_start = time.time()
        log("INFO", f"[DISPENSE-{item_type.upper()}] Waiting for MQTT response from pump {pump_number} (timeout: {response_timeout}s)", service="automation")
        
        while response["data"] is None:
            if (time.time() - response_start) > response_timeout:
                log("WARNING", f"[DISPENSE-{item_type.upper()}] Timeout waiting for response from pump {pump_number} after {response_timeout}s - Assuming success to continue recipe", service="automation")
                client.loop_stop()
                client.disconnect()
                all_results.append({
                    "pump_number": pump_number,
                    "amount": amount,
                    "item_type": item_type,
                    "success": True,
                    "message": f"Assumed success after {response_timeout}s timeout (no hardware response)",
                    "timeout": True
                })
                break  # Skip to next pump
            await asyncio.sleep(0.1)
        
        # Check if we got a response or timed out
        if response["data"] is None:
            continue  # Already added timeout result as success, move to next pump

        client.loop_stop()
        client.disconnect()

        
        # Store result for this pump
        mqtt_response = response["data"]
        log("INFO", f"[DISPENSE-{item_type.upper()}] MQTT response for pump {pump_number}: {mqtt_response}", service="automation")
        
        if mqtt_response.get("status") == "success":
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "item_type": item_type,
                "success": True,
                "message": f"Successfully dispensed {amount}g from {item_type} pump {pump_number}",
                "details": mqtt_response
            })
            log("INFO", f"[DISPENSE-{item_type.upper()}] SUCCESS: Pump {pump_number} dispensed {amount}g", service="automation")
        else:
            # Use error field if present, otherwise use status field
            error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "item_type": item_type,
                "success": False,
                "error": error_msg,
                "details": mqtt_response
            })
            log("ERROR", f"[DISPENSE-{item_type.upper()}] FAILED: Pump {pump_number} failed to dispense {amount}g - Error: {error_msg}", service="automation")
    
    # Return combined results
    all_success = all(result["success"] for result in all_results)
    
    # Count syrups vs water for summary
    syrup_count = sum(1 for r in all_results if r.get("item_type") == "syrup")
    water_count = sum(1 for r in all_results if r.get("item_type") == "water")
    
    if all_success:
        summary_parts = []
        if syrup_count > 0:
            summary_parts.append(f"{syrup_count} syrup(s)")
        if water_count > 0:
            summary_parts.append(f"{water_count} water pump(s)")
        summary = " and ".join(summary_parts)
        
        log("INFO", f"[DISPENSE-SYRUP/WATER] All {len(all_results)} dispenses successful ({summary})", service="automation")
        return {
            "success": True,
            "message": f"Successfully dispensed {summary}",
            "details": all_results
        }
    else:
        failed_count = sum(1 for result in all_results if not result["success"])
        failed_pumps = [r["pump_number"] for r in all_results if not r["success"]]
        log("ERROR", f"[DISPENSE-SYRUP/WATER] {failed_count} of {len(all_results)} dispenses failed. Failed pumps: {failed_pumps}", service="automation")
        log("ERROR", f"[DISPENSE-SYRUP/WATER] Full results: {all_results}", service="automation")
        return {
            "success": False,
            "error": f"{failed_count} item(s) failed to dispense",
            "message": f"Completed with {failed_count} failure(s) out of {len(all_results)} total dispenses",
            "details": all_results
        }

# Ice dispenser calibration data: input sent -> actual output received
ICE_CALIBRATION_DATA = {
    'input': [10, 20, 30, 40, 50, 60, 70, 75, 80, 176.8],
    'output': [30, 37.1, 47.5, 58, 65.1, 66.6, 70, 97.6, 98.7, 184.3]
}

def calculate_ice_input(desired_output):
    """
    Calculate the input value needed to get the desired ice output amount.
    Uses linear interpolation/extrapolation based on calibration data.
    Works for ANY value, not just within calibration range.
    
    Args:
        desired_output: The actual weight you want to receive (in grams)
    
    Returns:
        The input value to send to the ice machine
    """
    inputs = np.array(ICE_CALIBRATION_DATA['input'])
    outputs = np.array(ICE_CALIBRATION_DATA['output'])
    
    min_output, max_output = min(outputs), max(outputs)
    
    # For values outside calibration range, use linear extrapolation from last two points
    if desired_output > max_output:
        # Extrapolate using the last two calibration points
        last_two_outputs = outputs[-2:]
        last_two_inputs = inputs[-2:]
        
        # Calculate slope
        slope = (last_two_inputs[1] - last_two_inputs[0]) / (last_two_outputs[1] - last_two_outputs[0])
        
        # Extrapolate
        required_input = last_two_inputs[1] + slope * (desired_output - last_two_outputs[1])
        
        log("INFO", f"[ICE-CALIBRATION] Extrapolating above range: {desired_output}g desired → {required_input:.1f}g input (max calibrated: {max_output}g)", service="automation")
        
    elif desired_output < min_output:
        # Extrapolate using the first two calibration points
        first_two_outputs = outputs[:2]
        first_two_inputs = inputs[:2]
        
        # Calculate slope
        slope = (first_two_inputs[1] - first_two_inputs[0]) / (first_two_outputs[1] - first_two_outputs[0])
        
        # Extrapolate
        required_input = first_two_inputs[0] + slope * (desired_output - first_two_outputs[0])
        
        log("INFO", f"[ICE-CALIBRATION] Extrapolating below range: {desired_output}g desired → {required_input:.1f}g input (min calibrated: {min_output}g)", service="automation")
        
    else:
        # Within calibration range - use normal interpolation
        required_input = np.interp(desired_output, outputs, inputs)
        log("INFO", f"[ICE-CALIBRATION] Within range: {desired_output}g desired → {required_input:.1f}g input", service="automation")
    
    # Ensure we don't send negative values
    required_input = max(0, required_input)
    
    return round(required_input, 1)

async def dispense_ice(params: dict):
    """Dispense ice using MQTT communication with automatic calibration."""
    # example params: 
    # NEW FORMAT: {"ice": {"ice_cubes_12oz": 40}, "cups": {"cup_C12": 1.0}, ...}
    # OLD FORMAT: {"cups": {"cup_c16": 1.0}, "timeout": 300}
    # FLAT FORMAT: {"weight": 4, "timeout": 300}
    # Optional: "use_calibration": False to disable auto-calibration

    log("INFO", f"[DISPENSE-ICE] ========== FUNCTION CALLED ==========", service="automation")
    log("INFO", f"[DISPENSE-ICE] Full params: {params}", service="automation")
    log("INFO", f"[DISPENSE-ICE] Params keys: {list(params.keys())}", service="automation")
    
    use_calibration = params.get("use_calibration", True)  # Enable calibration by default
    
    # Priority 1: Check for new 'ice' dictionary format
    if "ice" in params and isinstance(params["ice"], dict):
        ice_dict = params["ice"]
        log("INFO", f"[DISPENSE-ICE] Found ice dictionary: {ice_dict}", service="automation")
        
        if not ice_dict:
            log("ERROR", f"[DISPENSE-ICE] Ice dictionary is empty!", service="automation")
            desired_weight = 0
        else:
            # Extract ice type and amount from first key
            # e.g., {"ice_cubes_12oz": 40} means 40g of ice
            ice_type = list(ice_dict.keys())[0]
            desired_weight = float(ice_dict[ice_type])
            log("INFO", f"[DISPENSE-ICE] Extracted from ice dict: type={ice_type}, amount={desired_weight}g", service="automation")
    
    # Priority 2: Handle nested cups dictionary format (old format)
    elif "cups" in params and isinstance(params["cups"], dict):
        cups_dict = params["cups"]
        # Extract cup type from first key (e.g., "cup_c7", "cup_c9", "cup_c12", "cup_c16")
        cup_type = list(cups_dict.keys())[0]
        
        # Map cup type to desired weight value (what we want to receive)
        cup_type_lower = cup_type.lower()
        if "cup_c7" in cup_type_lower:
            desired_weight = 64.0  #grams of ice
        elif "cup_c9" in cup_type_lower:
            desired_weight = 84.0  #grams of ice
        elif "cup_c12" in cup_type_lower:
            desired_weight = 120.0  #grams of ice
        elif "cup_c16" in cup_type_lower:
            desired_weight = 180.0  #grams of ice
        else:
            # Default to weight 0 if unknown cup type
            log("ERROR", f"Unknown cup type: {cups_dict}, defaulting to weight 0", service="automation")
            desired_weight = 0
        log("INFO", f"[DISPENSE-ICE] Extracted from cups dict: type={cup_type}, amount={desired_weight}g", service="automation")
    
    # Priority 3: Fallback to flat parameter format
    else:
        desired_weight = params.get("weight", 0)
        log("INFO", f"[DISPENSE-ICE] Using flat weight parameter: {desired_weight}g", service="automation")
    
    # Apply calibration if enabled
    if use_calibration and desired_weight > 0:
        weight = calculate_ice_input(desired_weight)
        log("INFO", f"[DISPENSE-ICE] Desired output: {desired_weight}g → Sending calibrated input: {weight}g", service="automation")
    else:
        weight = desired_weight
        if desired_weight > 0:
            log("INFO", f"[DISPENSE-ICE] Sending raw input: {weight}g (calibration disabled)", service="automation")
        else:
            log("ERROR", f"[DISPENSE-ICE] Desired weight is 0! Cannot dispense ice.", service="automation")
            return {
                "success": False,
                "error": "No ice amount specified",
                "message": "Cannot dispense ice - weight is 0"
            }
    
    log("INFO", f"[DISPENSE-ICE] About to connect to MQTT broker...", service="automation")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_ice/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"weight": weight})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect with fallback support
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "[DISPENSE-ICE] Failed to connect to any MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    log("INFO", f"[DISPENSE-ICE] Connected to MQTT broker successfully at {connected_host}:{connected_port}", service="automation")
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    # Now send the message
    log("INFO", f"[DISPENSE-ICE] Publishing to MQTT topic 'automation_ice': {payload}", service="automation")
    publish_result = client.publish("automation_ice", payload, qos=1)
    log("INFO", f"[DISPENSE-ICE] Publish result: {publish_result.rc} (0=success)", service="automation")

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
    log("INFO", f"[DISPENSE-ICE] Received MQTT response: {mqtt_response}", service="automation")
    
    if mqtt_response.get("status") == "success":
        if use_calibration and weight != desired_weight:
            message = f"Successfully dispensed ice (desired={desired_weight}g, sent={weight}g calibrated)"
        else:
            message = f"Successfully dispensed ice (weight={weight}g)"
        log("INFO", f"[DISPENSE-ICE] ✓ SUCCESS: {message}", service="automation")
        return {
            "success": True,
            "message": message,
            "details": mqtt_response
        }
    else:
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        log("ERROR", f"[DISPENSE-ICE] ✗ FAILED: {error_msg}", service="automation")
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to dispense ice: {error_msg}",
            "details": mqtt_response
        }

async def dispense_milk(params: dict):
    """Dispense multiple milk types using MQTT communication."""
    # example params: {"milk": {1: 260.0}, ...}
    # Loops through all pumps in the milk dictionary
    
    log("INFO", f"[DISPENSE-MILK] Called with params: {params}", service="automation")
    
    # Extract milk dictionary
    if "milk" not in params or not isinstance(params["milk"], dict):
        log("ERROR", "No milk dictionary found in params", service="automation")
        return {
            "success": False,
            "error": "No milk dictionary found in params",
            "message": "Invalid parameters: milk dictionary required"
        }
    
    milk_dict = params["milk"]
    log("INFO", f"[DISPENSE-MILK] Extracted milk dict: {milk_dict}", service="automation")
    
    if not milk_dict:
        return {
            "success": True,
            "message": "No milk to dispense",
            "details": []
        }
    
    # Prepare for loop through all milk pumps
    all_results = []
    mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
    username = params.get("username", "admin")
    password = params.get("password", "admin123")
    
    # Loop through each milk pump
    for pump_key, amount in milk_dict.items():
        pump_number = int(pump_key) if isinstance(pump_key, (int, str)) else 1
        
        log("INFO", f"[DISPENSE-MILK] Starting dispense: pump_key={pump_key}, pump_number={pump_number}, amount={amount}", service="automation")
        
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
        
        # Connect with fallback support
        mqtt_port = params.get("mqtt_port", 1883)
        success_conn, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
        
        if not success_conn:
            log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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

        # Wait for response with timeout (default 75 seconds)
        response_timeout = params.get("timeout", 75)
        response_start = time.time()
        log("INFO", f"[DISPENSE-MILK] Waiting for MQTT response from pump {pump_number} (timeout: {response_timeout}s)", service="automation")
        
        while response["data"] is None:
            if (time.time() - response_start) > response_timeout:
                log("WARNING", f"[DISPENSE-MILK] Timeout waiting for response from pump {pump_number} after {response_timeout}s - Assuming success to continue recipe", service="automation")
                client.loop_stop()
                client.disconnect()
                all_results.append({
                    "pump_number": pump_number,
                    "amount": amount,
                    "success": True,
                    "message": f"Assumed success after {response_timeout}s timeout (no hardware response)",
                    "timeout": True
                })
                break  # Skip to next pump
            await asyncio.sleep(0.1)
        
        # Check if we got a response or timed out
        if response["data"] is None:
            continue  # Already added timeout result as success, move to next pump

        client.loop_stop()
        client.disconnect()

        
        # Store result for this pump
        mqtt_response = response["data"]
        log("INFO", f"[DISPENSE-MILK] MQTT response for pump {pump_number}: {mqtt_response}", service="automation")
        
        if mqtt_response.get("status") == "success":
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": True,
                "message": f"Successfully dispensed {amount}g from pump {pump_number}",
                "details": mqtt_response
            })
            log("INFO", f"[DISPENSE-MILK] SUCCESS: Pump {pump_number} dispensed {amount}g", service="automation")
        else:
            # Use error field if present, otherwise use status field
            error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
            all_results.append({
                "pump_number": pump_number,
                "amount": amount,
                "success": False,
                "error": error_msg,
                "details": mqtt_response
            })
            log("ERROR", f"[DISPENSE-MILK] FAILED: Pump {pump_number} failed to dispense {amount}g - Error: {error_msg}", service="automation")
    
    # Return combined results
    all_success = all(result["success"] for result in all_results)
    
    if all_success:
        log("INFO", f"[DISPENSE-MILK] All {len(all_results)} dispenses successful", service="automation")
        return {
            "success": True,
            "message": f"Successfully dispensed {len(all_results)} item(s)",
            "details": all_results
        }
    else:
        failed_count = sum(1 for result in all_results if not result["success"])
        failed_pumps = [r["pump_number"] for r in all_results if not r["success"]]
        log("ERROR", f"[DISPENSE-MILK] {failed_count} of {len(all_results)} dispenses failed. Failed pumps: {failed_pumps}", service="automation")
        log("ERROR", f"[DISPENSE-MILK] Full results: {all_results}", service="automation")
        return {
            "success": False,
            "error": f"{failed_count} dispense(s) failed",
            "message": f"Completed with {failed_count} failure(s) out of {len(all_results)} items",
            "details": all_results
        }

async def slush_machine(params: dict):
    """Slush machine using MQTT communication."""
    
    slush_type = params.get("slush_type", "slush_2")
    
    # Handle nested cups dictionary format
    if "cups" in params and isinstance(params["cups"], dict):
        cups_dict = params["cups"]
        cup_type = list(cups_dict.keys())[0]
        
        # Map cup type to weight (in grams) and difference
        cup_type_lower = cup_type.lower()
        if "cup_c9" in cup_type_lower:
            weight = 500 #500g
            difference = 100 #100g margin
        elif "cup_c12" in cup_type_lower:
            weight = 400 #400g
            difference = 100 #100g margin
        elif "cup_c16" in cup_type_lower:
            weight = 300 #300g
            difference = 100 #100g margin
        else:
            log("ERROR", f"Unknown cup type: {cups_dict}, defaulting to weight 150", service="automation")
            weight = 150 #150g
            difference = 100 #100g margin
    else:
        # Fallback to flat parameter format
        weight = params.get("weight", 150)
        difference = params.get("difference", 100)
    
    log("DEBUG", f"Calling Slush machine with slush_type={slush_type}, weight={weight}, difference={difference}", service="automation")
    response = {"data": None}

    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_slush/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    payload = json.dumps({"slush_type": slush_type, "weight": weight, "difference": difference})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect to RabbitMQ MQTT broker using service name in Docker network
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
            "message": f"Successfully prepared {slush_type} slush (weight={weight}g, difference={difference}g)",
            "details": mqtt_response
        }
    else:
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to prepare slush: {error_msg}",
            "details": mqtt_response
        }

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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to prepare coffee: {error_msg}",
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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to prepare coffee: {error_msg}",
            "details": mqtt_response
        }

async def coffee_machine_purge(params: dict):

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

    coffee_t=3    
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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to prepare coffee: {error_msg}",
            "details": mqtt_response
        }

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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    log("INFO", f"[GRINDER] Attempting MQTT connection to {mqtt_host}:{mqtt_port}", service="automation")
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "[GRINDER] Failed to connect to any MQTT broker", service="automation")
        return {
            "success": False,
            "error": "Failed to connect to MQTT broker",
            "message": "Failed to connect to MQTT broker"
        }
    
    log("INFO", f"[GRINDER] Connected to MQTT broker at {connected_host}:{connected_port}", service="automation")
    
    # Give a moment for subscription to be processed
    time.sleep(0.5)
    
    log("INFO", f"[GRINDER] Publishing message: {payload}", service="automation")
    client.publish("automation_grinding", payload, qos=1)

    timeout = params.get("timeout", 120)
    start_time = time.time()
    log("INFO", f"[GRINDER] Waiting for response (timeout={timeout}s)", service="automation")

    while response["data"] is None and (time.time() - start_time) < timeout:
        await asyncio.sleep(0.1)

    if response["data"] is None:
        log("ERROR", f"[GRINDER] Timeout: No response from grinder after {timeout}s", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to grind coffee: {error_msg}",
            "details": mqtt_response
        }

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
        calibration = 4000        ##set values here in ms
    elif espresso_shots == 2:
        calibration = 3750        ##set values here in ms
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

    # Connect with fallback support
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to complete tampering: {error_msg}",
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
    
    # Connect with fallback support
    mqtt_host = params.get("mqtt_host", "192.168.200.254")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to froth milk: {error_msg}",
            "details": mqtt_response
        }

async def initialize_frother(params: dict):
    """Initialize frother using MQTT communication."""
    response = {"data": None}
    
    # Get seconds parameter, default to 1.5
    seconds = params.get("seconds", 1.5)
    
    def on_connect(client, userdata, flags, rc, props=None):
        client.subscribe("automation_frother_init/response", qos=1)

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            response["data"] = payload
        except json.JSONDecodeError:
            pass

    # Payload for frother initialization with seconds parameter
    payload = json.dumps({"frother_init": 1, "seconds": seconds})
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.username_pw_set(
        params.get("username", "admin"), 
        params.get("password", "admin123")
    )
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect with fallback support
    mqtt_host = params.get("mqtt_host", "192.168.200.254")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to initialize frother: {error_msg}",
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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to clean frother: {error_msg}",
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
    mqtt_host = params.get("mqtt_host", "rabbitmq")
    mqtt_port = params.get("mqtt_port", 1883)
    success, connected_host, connected_port = connect_mqtt_with_fallback(client, mqtt_host, mqtt_port)
    
    if not success:
        log("ERROR", "Failed to connect to any MQTT broker", service="automation")
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
        # Use error field if present, otherwise use status field
        error_msg = mqtt_response.get('error', mqtt_response.get('status', 'Unknown error'))
        return {
            "success": False,
            "error": error_msg,
            "message": f"Failed to complete rinser operation: {error_msg}",
            "details": mqtt_response
        }

# async def dispense_ingredient(params: dict):
#     """Dispense ingredient using MQTT communication."""
#     ingredient = params.get("ingredient", "sauce")
#     weight = params.get("weight", 10)
#     motor = params.get("motor", "sauce1")
    
#     response = {"data": None}

#     def on_connect(client, userdata, flags, rc, props=None):
#         client.subscribe("automation_dispensing/response", qos=1)

#     def on_message(client, userdata, msg):
#         try:
#             payload = json.loads(msg.payload.decode())
#             response["data"] = payload
#         except json.JSONDecodeError:
#             pass

#     # Format command for dispensing system: ingredient_weight
#     command = f"{ingredient}_{weight}"
#     payload = json.dumps({
#         "ingredient": ingredient, 
#         "weight": weight, 
#         "motor": motor,
#         "command": command
#     })
    
#     client = mqtt.Client(protocol=mqtt.MQTTv311)
#     client.username_pw_set(
#         params.get("username", "admin"), 
#         params.get("password", "admin123")
#     )
#     client.on_connect = on_connect
#     client.on_message = on_message
    
#     # Connect to external MQTT broker for dispensing (your Arduino setup)
#     mqtt_host = params.get("mqtt_host", "192.168.200.254")  # Use external MQTT broker
#     client.connect(mqtt_host, 1883, 60)
    
#     client.loop_start()
    
#     # Wait for connection and subscription to be established
#     connection_timeout = 10
#     connection_start = time.time()
#     while not client.is_connected() and (time.time() - connection_start) < connection_timeout:
#         time.sleep(0.1)
    
#     if not client.is_connected():
#         log("ERROR", "Failed to connect to MQTT broker", service="automation")
#         return {
#             "success": False,
#             "error": "Failed to connect to MQTT broker",
#             "message": "Failed to connect to MQTT broker"
#         }
    
#     # Give a moment for subscription to be processed
#     time.sleep(0.5)
    
#     # Now send the message to dispensing topic
#     client.publish("automation_dispensing", payload, qos=1)

#     # Give a moment for the message to be sent
#     await asyncio.sleep(0.5)
    
#     client.loop_stop()
#     client.disconnect()

#     return {
#         "success": True,
#         "message": f"Dispensed {weight}g of {ingredient} via {motor}",
#         "details": {
#             "ingredient": ingredient,
#             "weight": weight,
#             "motor": motor,
#             "command": command
#         }
#     }

###=========== Test Functions ============

async def automation_test(params: dict):
    """Automation test using MQTT communication."""
    # example params: {"automation_test": 1}
    time.sleep(1)
    return {
        "success": True,
        "message": "Successfully completed automation test",
        "details": "Automation test completed"
    }

# Map function names to implementations
AUTOMATION_FUNCTIONS = {
    "dispense_syrup": dispense_syrup,
    "dispense_sauce": dispense_sauce,
    "dispense_milk": dispense_milk,
    # "dispense_ingredient": dispense_ingredient,
    "slush_machine": slush_machine,
    "coffee_machine": coffee_machine,
    "coffee_machine_purge": coffee_machine_purge,
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