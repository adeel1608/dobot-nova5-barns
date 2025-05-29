# services/automation/automation_functions.py
import asyncio


async def heat_water(params: dict):
    """Heat water to specified temperature"""
    target_temp = params.get("target_temp_c", 93)
    volume_ml = params.get("volume_ml", 250)
    
    # Simulate heating process
    await asyncio.sleep(3)  # Simulate heating time
    
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
    """Dispense milk from automated milk system"""
    milk_type = params.get("milk_type", "regular")
    amount = params.get("amount", 120)  # amount in ml
    temperature = params.get("temperature", "cold")
    
    # Simulate milk dispensing
    await asyncio.sleep(1.5)
    
    return {
        "success": True,
        "message": f"Dispensed {amount}ml of {milk_type} milk ({temperature})",
        "details": {
            "milk_type": milk_type,
            "amount_ml": amount,
            "temperature": temperature,
            "duration_sec": 1.5
        }
    }


# Map function names to actual implementations
AUTOMATION_FUNCTIONS = {
    "heat_water": heat_water,
    "dispense_milk": dispense_milk,
    # Add more automation functions as needed
} 