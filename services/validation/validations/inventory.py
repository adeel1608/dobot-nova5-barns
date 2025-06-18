"""Inventory validation functions for BARNS system."""

import random
from datetime import datetime, timedelta
from . import BaseValidation

# Comprehensive inventory tracking with all 28 specific items
# In production, this would be in a database
INVENTORY_LEVELS = {
    # 8 Types of Milk Products
    "whole_milk": {"level": random.randint(10, 100), "threshold_low": 20, "threshold_medium": 50, "last_refilled": None},
    "skim_milk": {"level": random.randint(10, 100), "threshold_low": 20, "threshold_medium": 50, "last_refilled": None},
    "almond_milk": {"level": random.randint(10, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "soy_milk": {"level": random.randint(10, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "oat_milk": {"level": random.randint(10, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "coconut_milk": {"level": random.randint(10, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "rice_milk": {"level": random.randint(10, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "heavy_cream": {"level": random.randint(10, 100), "threshold_low": 25, "threshold_medium": 55, "last_refilled": None},
    
    # 1 Type of Coffee Bean
    "coffee_beans": {"level": random.randint(15, 100), "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    
    # 12 Types of Syrups
    "vanilla_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "caramel_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "chocolate_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "hazelnut_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "cinnamon_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "peppermint_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "irish_cream_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "amaretto_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "coconut_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "raspberry_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "lavender_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    "maple_syrup": {"level": random.randint(10, 100), "threshold_low": 10, "threshold_medium": 30, "last_refilled": None},
    
    # 7 Types of Cups (Paper: 7,9,12 oz + Plastic: 7,9,12,16 oz)
    "paper_cup_7oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "paper_cup_9oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "paper_cup_12oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "plastic_cup_7oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "plastic_cup_9oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "plastic_cup_12oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "plastic_cup_16oz": {"level": random.randint(20, 200), "threshold_low": 30, "threshold_medium": 75, "last_refilled": None}
}

# Generate some random historical refill dates
def _generate_random_refill_dates():
    """Generate random refill dates for items that don't have them"""
    for item_key, item_data in INVENTORY_LEVELS.items():
        if item_data["last_refilled"] is None:
            # Random date within the last 7 days
            days_ago = random.randint(0, 7)
            hours_ago = random.randint(0, 23)
            minutes_ago = random.randint(0, 59)
            refill_date = datetime.now() - timedelta(days=days_ago, hours=hours_ago, minutes=minutes_ago)
            item_data["last_refilled"] = refill_date.isoformat()

# Initialize random refill dates
_generate_random_refill_dates()

class IngredientAvailabilityValidator(BaseValidation):
    """Validates if sufficient ingredients are available."""
    
    @property
    def function_name(self) -> str:
        return "check_ingredient_availability"
    
    def validate(self, params: dict) -> dict:
        """Check if ingredient is available in required quantity."""
        ingredient = params.get("ingredient")
        amount_needed = params.get("amount_needed", 1)
        
        if ingredient not in INVENTORY_LEVELS:
            return {"passed": False, "details": f"Unknown ingredient: {ingredient}"}
        
        available = INVENTORY_LEVELS[ingredient]["level"]
        if available >= amount_needed:
            return {
                "passed": True, 
                "details": f"Sufficient {ingredient} available: {available} >= {amount_needed}",
                "data": {
                    "ingredient": ingredient,
                    "available": available,
                    "needed": amount_needed
                }
            }
        else:
            return {
                "passed": False, 
                "details": f"Insufficient {ingredient}: {available} < {amount_needed}",
                "data": {
                    "ingredient": ingredient,
                    "available": available,
                    "needed": amount_needed,
                    "shortage": amount_needed - available
                }
            }


class InventoryUpdater(BaseValidation):
    """Updates inventory levels after ingredient usage."""
    
    @property
    def function_name(self) -> str:
        return "update_inventory"
    
    def validate(self, params: dict) -> dict:
        """Update inventory levels."""
        ingredient = params.get("ingredient")
        amount_used = params.get("amount_used", 1)
        
        if ingredient not in INVENTORY_LEVELS:
            return {"passed": False, "details": f"Unknown ingredient: {ingredient}"}
        
        current_level = INVENTORY_LEVELS[ingredient]["level"]
        new_level = max(0, current_level - amount_used)
        INVENTORY_LEVELS[ingredient]["level"] = new_level
        
        threshold_low = INVENTORY_LEVELS[ingredient]["threshold_low"]
        threshold_medium = INVENTORY_LEVELS[ingredient]["threshold_medium"]
        
        if new_level <= threshold_low:
            status = "low"
        elif new_level <= threshold_medium:
            status = "medium"
        else:
            status = "high"
        
        return {
            "passed": True, 
            "details": f"Inventory updated: {ingredient} level now {new_level}",
            "data": {
                "ingredient": ingredient,
                "previous_level": current_level,
                "new_level": new_level,
                "amount_used": amount_used,
                "status": status,
                "threshold_warning": new_level <= threshold_low
            }
        }


def get_inventory_levels():
    """Get current inventory levels."""
    return INVENTORY_LEVELS


def set_inventory_level(ingredient: str, level: int):
    """Set inventory level for refills."""
    if ingredient in INVENTORY_LEVELS:
        INVENTORY_LEVELS[ingredient]["level"] = level
        INVENTORY_LEVELS[ingredient]["last_refilled"] = datetime.now().isoformat()
        return True
    return False


def get_category_summary():
    """Get category summary with lowest levels per category"""
    categories = {
        'milk': ['whole_milk', 'skim_milk', 'almond_milk', 'soy_milk', 'oat_milk', 'coconut_milk', 'rice_milk', 'heavy_cream'],
        'beans': ['coffee_beans'],
        'syrups': ['vanilla_syrup', 'caramel_syrup', 'chocolate_syrup', 'hazelnut_syrup', 'cinnamon_syrup', 'peppermint_syrup', 'irish_cream_syrup', 'amaretto_syrup', 'coconut_syrup', 'raspberry_syrup', 'lavender_syrup', 'maple_syrup'],
        'cups': ['paper_cup_7oz', 'paper_cup_9oz', 'paper_cup_12oz', 'plastic_cup_7oz', 'plastic_cup_9oz', 'plastic_cup_12oz', 'plastic_cup_16oz']
    }
    
    summary = {}
    for category, items in categories.items():
        lowest_level = 100
        lowest_level_string = 'high'
        
        for item_key in items:
            if item_key in INVENTORY_LEVELS:
                item_data = INVENTORY_LEVELS[item_key]
                level = item_data["level"]
                threshold_low = item_data["threshold_low"]
                threshold_medium = item_data["threshold_medium"]
                
                if level < lowest_level:
                    lowest_level = level
                    
                    if level <= threshold_low:
                        lowest_level_string = "low"
                    elif level <= threshold_medium:
                        lowest_level_string = "medium"
                    else:
                        lowest_level_string = "high"
        
        summary[category] = {
            "level": lowest_level_string,
            "numeric": lowest_level,
            "last_refilled": None  # Could be the most recent refill
        }
    
    return summary 