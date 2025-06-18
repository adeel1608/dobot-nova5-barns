# services/automation/automation_functions.py
"""Automation functions for BARNS coffee brewing system."""

import asyncio


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

async def dispense_milk(params: dict):
    """Dispense milk from automated milk system."""
    milk_type = params.get("milk_type", "regular")
    amount = params.get("amount", 120)
    temperature = params.get("temperature", "cold")
    
    # Simulate milk dispensing
    await asyncio.sleep(1.5)
    
    return {
        "success": True,
        "message": f"Dispensed {amount}ml of {milk_type} milk",
        "details": {
            "milk_type": milk_type,
            "amount_ml": amount,
            "temperature": temperature,
            "duration_sec": 1.5
        }
    }

async def automation_test1(params: dict):
    """Test function 1 for automation service."""
    await asyncio.sleep(0.5)
    
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
    await asyncio.sleep(0.7)
    
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