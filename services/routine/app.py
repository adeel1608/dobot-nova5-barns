# services/routine/app.py
import os, json
from queue import Queue
from threading import Thread
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from .executer import process_task, VALIDATION_SERVICE_URL

# --- Models ---
class ItemDetail(BaseModel):
    cup_id: str
    cup_size: str
    drink: str
    addons: list[str] = []

class TaskRequest(BaseModel):
    arm_id: int                    # 1 or 2
    function: str                  # e.g. "pull_espresso"
    item: ItemDetail               # full item details

# --- App & Queues Setup ---
task_queues: dict[int, asyncio.Queue] = {
    1: asyncio.Queue(),
    2: asyncio.Queue()
}

task_configs = {}  # Will be loaded from config file

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global task_configs
    cfg_path = os.getenv("ROUTINE_CONFIG_PATH", "/app/config/tasks.json")
    with open(cfg_path) as f:
        task_configs = json.load(f)
    
    # Override services URLs from environment if provided
    validation_url = os.environ.get("VALIDATION_SERVICE_URL")
    if validation_url:
        from . import executer
        executer.VALIDATION_SERVICE_URL = validation_url
        print(f"Using validation service at: {executer.VALIDATION_SERVICE_URL}")
    
    # Start worker coroutines
    worker_tasks = []
    for arm_id in [1, 2]:
        task = asyncio.create_task(worker(arm_id, task_queues[arm_id]))
        worker_tasks.append(task)
    
    yield
    
    # Shutdown - cancel worker tasks
    for task in worker_tasks:
        task.cancel()
    
    # Wait for tasks to complete
    await asyncio.gather(*worker_tasks, return_exceptions=True)

app = FastAPI(title="Routine Operation Service", lifespan=lifespan)

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

async def worker(arm_id: int, q: asyncio.Queue):
    """Continuously process tasks assigned to this arm."""
    while True:
        try:
            task: TaskRequest = await q.get()
            try:
                await process_task(arm_id, task, task_configs)
            except Exception as e:
                # handle/log error, e.g., publish a failure event
                print(f"[Routine][Arm{arm_id}] error: {e}")
            finally:
                q.task_done()
        except asyncio.CancelledError:
            # Handle task cancellation
            break
        except Exception as e:
            # Handle unexpected errors
            print(f"[Routine][Arm{arm_id}] unexpected error: {e}")

# --- Endpoint to receive tasks from Scheduler ---
@app.post("/task")
async def submit_task(task: TaskRequest):
    if task.arm_id not in task_queues:
        raise HTTPException(status_code=400, detail="Invalid arm_id")
    if task.function not in task_configs:
        raise HTTPException(status_code=404, detail="Unknown function")
    
    await task_queues[task.arm_id].put(task)
    return {"status": "queued", "arm_id": task.arm_id, "function": task.function}

# --- Health check endpoint ---
@app.get("/health")
def health_check():
    """Simple health check endpoint for service monitoring."""
    return {
        "status": "ok", 
        "validation_service": VALIDATION_SERVICE_URL
    }
