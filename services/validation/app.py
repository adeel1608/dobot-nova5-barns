# services/validation/app.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

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

# Define request model for validation
class ValidationRequest(BaseModel):
    function: str
    params: dict = {}

# Dummy implementations of validation functions (in real life, these would interface with hardware or AI)
def check_cup_present(params: dict):
    # e.g., read a sensor or analyze an image to confirm cup presence
    return {"passed": True, "details": "cup detected"}

def check_weight(params: dict):
    required = params.get("min_weight", 0)
    # e.g., read from a scale sensor
    current_weight = 30  # dummy value
    return {"passed": current_weight >= required, "weight": current_weight}

def check_temperature(params: dict):
    min_temp = params.get("min_temp", 0)
    current_temp = 65  # dummy value (°C)
    return {"passed": current_temp >= min_temp, "temp": current_temp}

# Map function names to actual implementations
VALIDATORS = {
    "check_cup_present": check_cup_present,
    "check_weight": check_weight,
    "check_temperature": check_temperature,
    # ... other validation functions ...
}

@app.post("/validate")
def validate(request: ValidationRequest):
    """Run a validation function by name with given parameters."""
    func_name = request.function
    if func_name not in VALIDATORS:
        return {"error": f"No such validation function '{func_name}'", "passed": False}
    result = VALIDATORS[func_name](request.params or {})
    # Possibly publish an event if something is notably wrong (e.g., low ingredient)
    if func_name == "check_weight" and result.get("passed") is False:
        # If weight check fails (like not enough volume)
        # publish event e.g., "validation.weight_fail" (omitted for brevity)
        pass
    return result
