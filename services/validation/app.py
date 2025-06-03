# services/validation/app.py
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import requests
import json
from typing import Dict, Optional
from datetime import datetime

app = FastAPI(title="Validation Service")

# Add CORS middleware to allow dashboard access
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Dashboard origin
        "http://127.0.0.1:3000",
        "http://localhost:3001",  # Allow alternative ports
        "http://127.0.0.1:3001"
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Inventory tracking - In a real implementation, this would be in a database
INVENTORY_LEVELS = {
    "milk": {"level": 100, "threshold_low": 20, "threshold_medium": 50, "last_refilled": None},
    "beans": {"level": 80, "threshold_low": 15, "threshold_medium": 40, "last_refilled": None},
    "cup": {"level": 150, "threshold_low": 30, "threshold_medium": 75, "last_refilled": None},
    "syrup": {"level": 60, "threshold_low": 10, "threshold_medium": 30, "last_refilled": None}
}

# OMS endpoint for sending threshold warnings
OMS_THRESHOLD_URL = "http://localhost:8000/inventory/threshold-warning"

# Define request models
class ValidationRequest(BaseModel):
    function: str
    params: dict = {}

class InventoryRefillRequest(BaseModel):
    ingredient: str

class IngredientCheckRequest(BaseModel):
    ingredient: str
    amount_needed: int = 1



# Dummy implementations of validation functions (in real life, these would interface with hardware or AI)
def check_cup_present(params: dict):
    # e.g., read a sensor or analyze an image to confirm cup presence
    # e.g for failed case:
    return {"passed": False, "details": "cup not detected"}
    # for success case:
    return {"passed": True, "details": "cup detected"}



# Map function names to actual implementations
VALIDATORS = {
    "check_cup_present": check_cup_present,
    # ... other validation functions ...
}

@app.post("/validate")
def validate(request: ValidationRequest):
    """Run a validation function by name with given parameters."""
    func_name = request.function
    if func_name not in VALIDATORS:
        return {"error": f"No such validation function '{func_name}'", "passed": False}
    
    result = VALIDATORS[func_name](request.params or {})
    
    return result

@app.post("/inventory/refill")
def handle_refill_acknowledgment(request: InventoryRefillRequest):
    """Handle refill acknowledgment from Dashboard - refill the ingredient in validation service database"""
    try:
        ingredient = request.ingredient.lower()
        
        # Validate ingredient
        if ingredient not in INVENTORY_LEVELS:
            raise HTTPException(
                status_code=400, 
                detail=f"Unknown ingredient: {ingredient}. Valid ingredients: {list(INVENTORY_LEVELS.keys())}"
            )
        
        # Refill the ingredient
        result = refill_inventory(ingredient)
        
        if result["success"]:
            return {
                "status": "success",
                "message": f"Ingredient {ingredient} refilled successfully",
                "ingredient": ingredient,
                "old_level": result["old_level"],
                "new_level": result["new_level"],
                "refilled_at": result["refilled_at"]
            }
        else:
            raise HTTPException(status_code=500, detail=result.get("error", "Failed to refill ingredient"))
            
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal error during refill: {str(e)}")

@app.get("/inventory/status")
def get_inventory_status():
    """Get current inventory status"""
    try:
        # Format the response to match what the dashboard expects
        formatted_inventory = {}
        
        for ingredient, data in INVENTORY_LEVELS.items():
            level = data["level"]
            
            # Determine level category
            if level <= data["threshold_low"]:
                level_category = "low"
            elif level <= data["threshold_medium"]:
                level_category = "medium"
            else:
                level_category = "high"
            
            formatted_inventory[ingredient] = {
                "level": level_category,
                "actual_amount": level,
                "last_refilled": data["last_refilled"]
            }
        
        return {
            "status": "success",
            "inventory": formatted_inventory,
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get inventory status: {str(e)}")

@app.post("/inventory/check")
def check_ingredient(request: IngredientCheckRequest):
    """Check ingredient availability (used by other services)"""
    try:
        ingredient = request.ingredient.lower()
        amount = request.amount_needed
        
        if ingredient not in INVENTORY_LEVELS:
            raise HTTPException(
                status_code=400,
                detail=f"Unknown ingredient: {ingredient}. Valid ingredients: {list(INVENTORY_LEVELS.keys())}"
            )
        
        result = check_ingredient_level(ingredient, amount)
        
        return {
            "ingredient": ingredient,
            "available": result["available"],
            "current_level": result["current_level"],
            "amount_requested": amount,
            "remaining_after": result["remaining_after"],
            "severity": result["severity"]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to check ingredient: {str(e)}")

@app.get("/health")
def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "validation",
        "timestamp": datetime.now().isoformat(),
        "inventory_items": len(INVENTORY_LEVELS)
    }
