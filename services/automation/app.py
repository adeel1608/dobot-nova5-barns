# services/automation/app.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from .automation_functions import AUTOMATION_FUNCTIONS

app = FastAPI(title="Automation Service")

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

# Define request model for automation
class AutomationRequest(BaseModel):
    function: str
    params: dict = {}

@app.post("/automate")
async def automate(request: AutomationRequest):
    """Execute an automation function by name with given parameters."""
    func_name = request.function
    
    if func_name not in AUTOMATION_FUNCTIONS:
        return {
            "success": False,
            "error": f"No such automation function '{func_name}'",
            "message": f"Available functions: {list(AUTOMATION_FUNCTIONS.keys())}"
        }
    
    try:
        result = await AUTOMATION_FUNCTIONS[func_name](request.params or {})
        return result
    except Exception as e:
        return {
            "success": False,
            "error": f"Error executing automation function '{func_name}': {str(e)}",
            "message": "Automation function failed"
        }

@app.get("/health")
def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "automation"}

@app.get("/functions")
def list_functions():
    """List all available automation functions"""
    return {
        "functions": list(AUTOMATION_FUNCTIONS.keys()),
        "count": len(AUTOMATION_FUNCTIONS)
    } 