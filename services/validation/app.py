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

def update_inventory(params: dict):
    # update the inventory levels
    # e.g.
    # e.g for failed case:
    return {"passed": False, "details": "cup not detected"}
    # for success case:
    return {"passed": True, "details": "cup detected"}




# Map function names to actual implementations
VALIDATORS = {
    "check_cup_present": check_cup_present,
    "update_inventory": update_inventory,
    # ... other validation functions ...
}


# ---- Endpoint for Routine Handler ----
@app.post("/update_inventory")
async def update_inventory(request: ValidationRequest):
    """Run a validation function by name with given parameters."""
    # func_name = request.function
    # if func_name not in VALIDATORS:
    #     return {"error": f"No such validation function '{func_name}'", "passed": False}
    
    result = {"passed": True, "details": {}}
    return result

@app.post("/check_cup_placed")
async def check_cup_placed(request: ValidationRequest):
    """Run a validation function by name with given parameters."""
    # func_name = request.function
    # if func_name not in VALIDATORS:
    #     return {"error": f"No such validation function '{func_name}'", "passed": False}
    
    result = {"passed": True, "details": {}}
    return result

@app.post("/check_cup_picked")
async def check_cup_picked(request: ValidationRequest):
    """Run a validation function by name with given parameters."""
    # func_name = request.function
    # if func_name not in VALIDATORS:
    #     return {"error": f"No such validation function '{func_name}'", "passed": False}

    result = {"passed": True, "details": {}}
    return result

@app.post("/pre_check") 
async def pre_check(request: ValidationRequest):
    """Run a validation function by name with given parameters.
    ## EXAMPLE RESPONSE:
                {
                "passed": false,
                "details": {
                    "cappuccino": {
                        "cup": {
                            "current": 5,
                            "needed": 1,
                            "critical_threshold": 10,
                            "status": "insufficient"
                        },
                        "espresso": {
                            "current": 100,
                            "needed": 1,
                            "critical_threshold": 50,
                            "status": "sufficient"
                        },
                        "milk": {
                            "current": 100,
                            "needed": 150,
                            "critical_threshold": 200,
                            "status": "insufficient"
                        }
                    }
                }
            }
    #NOTE: we're keeping it above the threshold to make sure the machine doesn't run out of inventory to amount for the error window
    # this threshold is tunable from the loadup of the validation service 
    """
    # func_name = request.function
    # if func_name not in VALIDATORS:
    #     return {"error": f"No such validation function '{func_name}'", "passed": False}
    
    result = {"passed": True, "details": {}}
    return result


# ------------------------------------------------------------
# TODO: @uzair @mais complete the health check endpoint
# @app.get("/health")
# def health_check():
#     """Health check endpoint"""
#     return {
#         "status": "healthy",
#         "service": "validation",
#         "timestamp": datetime.now().isoformat(),
#         "inventory_items": len(INVENTORY_LEVELS)
#     }
