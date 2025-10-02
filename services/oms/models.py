from typing import List, Optional, Any
from pydantic import BaseModel, Field

class Cup(BaseModel):
    """Model representing a cup of coffee in an order."""
    type: str  # e.g., "latte", "americano", etc.
    size: str = "regular"  # "small", "regular", "large"
    addons: List[str] = []  # e.g., ["extra_shot", "vanilla"]
    ingredients: Any = []  # e.g., ["extra_shot", "vanilla"] or dict

class Order(BaseModel):
    """Model representing a complete order with multiple cups."""
    id: Optional[int] = None
    status: str = "queued"  # "queued", "processing", "completed", "failed"
    cups: List[Cup]
    
    class Config:
        json_schema_extra = {
            "example": {
                "cups": [
                    {"type": "latte", "size": "large", "addons": ["extra_shot"]},
                    {"type": "americano", "size": "regular", "addons": []}
                ]
            }
        }

class Task(BaseModel):
    """Model representing a task assigned to a robotic arm."""
    id: Optional[int] = None
    order_id: int
    item_id: int
    arm_id: int
    function_name: str
    status: str = "queued"  # "queued", "running", "completed", "failed"
    error_message: Optional[str] = None

class TaskStep(BaseModel):
    """Model representing a step within a task."""
    task_id: int
    step_index: int
    step_type: str  # "validation" or "robot"
    function_name: str
    params: dict
    status: str = "pending"  # "pending", "running", "passed", "failed"
