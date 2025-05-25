# services/scheduler/app.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, BackgroundTasks, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import json
import os
from pathlib import Path
from pydantic import BaseModel
from . import scheduler  # Import the scheduler module directly

app = FastAPI(title="Scheduler Service")

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

# WebSocket connections to push status updates
status_ws_clients: list[WebSocket] = []

# Load drink recipes on startup
recipes = {}

class TaskFeedback(BaseModel):
    cup_id: str
    action: str
    success: bool
    message: str = ""

@app.on_event("startup")
async def startup_event():
    """Load drink recipes on startup."""
    global recipes
    recipe_file = Path(__file__).parent / "data" / "recipes.json"
    recipes = scheduler.load_recipes(str(recipe_file))
    # Register the status callback function
    scheduler.register_status_callback(notify_status)
    
    # Set routine service URL from environment if provided
    routine_url = os.environ.get("ROUTINE_SERVICE_URL")
    if routine_url:
        scheduler.ROUTINE_SERVICE_URL = routine_url

@app.post("/process")
def process_order(order: dict, background_tasks: BackgroundTasks):
    """Receive an order to process (called by OMS)."""
    order_id = order.get("id")
    drinks = order.get("cups", [])  # list of cup definitions (type, etc.)
    # Start background task to handle the order so we can return immediately
    background_tasks.add_task(_handle_order, order_id, drinks)
    return {"msg": "Order accepted by scheduler", "order_id": order_id}

@app.post("/feedback")
def routine_feedback(feedback: TaskFeedback):
    """Receive feedback from routine service about task completion."""
    # Process the feedback through the scheduler
    scheduler.handle_routine_feedback(
        feedback.cup_id, 
        feedback.action, 
        feedback.success
    )
    
    # Notify clients about the update
    message = f"{feedback.action} for cup {feedback.cup_id} {'completed' if feedback.success else 'failed'}"
    if feedback.message:
        message += f": {feedback.message}"
    notify_status(message)
    
    return {"status": "received"}

async def _handle_order(order_id: int, drinks: list):
    """Background coroutine to process each drink using the scheduler."""
    try:
        # Use the scheduler's process_order_async function
        success = await scheduler.process_order_async(order_id, drinks, recipes)
        if not success:
            notify_status(f"Failed to process order {order_id}")
    except Exception as e:
        # Error already handled in process_order_async, just log here
        scheduler.current_status.update({"status": "error", "step": str(e)})
        notify_status(f"Error in order {order_id}: {e}")

@app.get("/status")
def get_status():
    """Get current processing status."""
    return scheduler.get_current_status()

@app.websocket("/ws/status")
async def status_ws(websocket: WebSocket):
    await websocket.accept()
    status_ws_clients.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        status_ws_clients.remove(websocket)

def notify_status(message: str):
    """Helper to send status updates to all WebSocket clients."""
    # This could also package the full current_status dict
    for ws in list(status_ws_clients):
        try:
            asyncio.create_task(ws.send_text(message))
        except Exception:
            status_ws_clients.remove(ws)
