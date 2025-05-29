# # services/routine/executor.py
# from validation_client import call_validation
# from robot_client import call_robot
# from event_publisher import publish_event

import os
import httpx

# Configuration for the scheduler service
SCHEDULER_SERVICE_URL = os.environ.get("SCHEDULER_SERVICE_URL", "http://scheduler:8000")
# Configuration for the validation service
VALIDATION_SERVICE_URL = os.environ.get("VALIDATION_SERVICE_URL", "http://validation:8000")
# Configuration for the automation service
AUTOMATION_SERVICE_URL = os.environ.get("AUTOMATION_SERVICE_URL", "http://automation:8000")

async def call_validation(func_name: str, params: dict):
    """
    Calls the validation service with the given function name and parameters.
    """
    url = f"{VALIDATION_SERVICE_URL}/validate"
    payload = {
        "function": func_name,
        "params": params
    }
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(url, json=payload)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Validation service error: {response.status_code} - {response.text}")
                return {"passed": False, "details": f"Validation service error: {response.status_code}"}
    except Exception as e:
        print(f"Error calling validation service: {str(e)}")
        return {"passed": False, "details": f"Error calling validation service: {str(e)}"}

async def call_automation(func_name: str, params: dict):
    """
    Calls the automation service with the given function name and parameters.
    """
    url = f"{AUTOMATION_SERVICE_URL}/automate"
    payload = {
        "function": func_name,
        "params": params
    }
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(url, json=payload)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Automation service error: {response.status_code} - {response.text}")
                return {"success": False, "message": f"Automation service error: {response.status_code}"}
    except Exception as e:
        print(f"Error calling automation service: {str(e)}")
        return {"success": False, "message": f"Error calling automation service: {str(e)}"}

def call_robot(func_name: str, params: dict, arm_id: int):
    """
    Calls the robot service with the given function name and parameters.
    """
    # TODO: Implement actual robot call
    return {"success": True, "message": "robot call successful"}    

def publish_event(event_name: str, data: dict):
    """
    Publishes an event to the event bus.
    """
    # TODO: Implement actual event publishing
    print(f"Publishing event: {event_name} with data: {data}")  

async def send_feedback_to_scheduler(cup_id: str, action: str, success: bool, message: str = ""):
    """
    Sends feedback to the scheduler service about task completion.
    """
    url = f"{SCHEDULER_SERVICE_URL}/feedback"
    payload = {
        "cup_id": cup_id,
        "action": action,
        "success": success,
        "message": message
    }
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(url, json=payload)
            if response.status_code == 200:
                print(f"Feedback sent to scheduler for {action} on cup {cup_id}")
                return True
            else:
                print(f"Failed to send feedback to scheduler: {response.text}")
                return False
    except Exception as e:
        print(f"Error sending feedback to scheduler: {str(e)}")
        return False

async def process_task(arm_id: int, task, configs: dict):
    """
    Executes the configured steps for a high-level function on a given arm.
    """
    cup_id = task.item.cup_id
    function = task.function
    success = True
    message = ""
    
    try:
        cfg = configs[function]
        for step in cfg["steps"]:
            step_type = step["type"]
            func_name = step["function"]
            # Merge static params + any item-specific params (if needed)
            params = {**step.get("params", {}), **task.item.dict()}
            if step_type == "validation":
                res = await call_validation(func_name, params)
                if not res.get("passed", False):
                    message = f"Validation failed: {res.get('details', '')}"
                    publish_event("validation.failed", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "reason": res})
                    success = False
                    break  # abort on validation failure
            elif step_type == "robot":
                res = call_robot(func_name, params, arm_id=arm_id)
                if not res.get("success", False):
                    message = f"Robot error: {res.get('message', '')}"
                    publish_event("robot.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")})
                    success = False
                    break  # abort on robot error
            elif step_type == "automation":
                res = await call_automation(func_name, params)
                if not res.get("success", False):
                    message = f"Automation error: {res.get('message', '')}"
                    publish_event("automation.error", 
                                {"arm": arm_id, "cup": cup_id,
                                "step": func_name, "error": res.get("message", "")})
                    success = False
                    break  # abort on automation error
            # publish a step-completed event
            publish_event("routine.step_completed", 
                        {"arm": arm_id, "cup": cup_id,
                        "step": func_name})
    except Exception as e:
        success = False
        message = f"Exception in routine: {str(e)}"
        print(f"Error processing task: {e}")
    
    # Send feedback to scheduler
    await send_feedback_to_scheduler(cup_id, function, success, message)
    
    # all steps done
    if success:
        publish_event("routine.completed", 
                    {"arm": arm_id, "cup": cup_id, "function": function})
