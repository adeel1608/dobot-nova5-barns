import asyncio
import datetime
from typing import Literal, Optional
from pydantic import BaseModel, ValidationError
from fastapi import HTTPException
import logging
import threading
import os
import sys
from queue import Queue
import json
from concurrent.futures import ThreadPoolExecutor

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from shared.logger import log
from .inventory_manager import InventoryManager
from .db_client import DatabaseClient
# Replace dummy detector with production detector
from .coffee_detection.camera_worker_production import ProductionCoffeeDetector, load_config
from .config import get_db_connection_string, config

# Cup detector import - can switch between real and dummy
USE_DUMMY_CUP_DETECTOR = os.getenv("USE_DUMMY_CUP_DETECTOR", "false").lower() == "true"

if USE_DUMMY_CUP_DETECTOR:
    from .cup_detection.dummy_detector import CupDetector
    log("INFO", "Using DUMMY Cup Detector for testing", service="validation")
else:
    from .cup_detection.cup_detector import CupDetector
    log("INFO", "Using RF-DETR Cup Detector", service="validation")


class MainValidation:
    def __init__(self):
        self._db_client = DatabaseClient(get_db_connection_string())
        # db_host = os.getenv("POSTGRES_HOST", "localhost")
        # db_port = os.getenv("POSTGRES_PORT", "5432")
        # db_name = os.getenv("POSTGRES_DB", "barns_validation")
        # db_user = os.getenv("POSTGRES_USER", "validation_user")
        # db_password = os.getenv("POSTGRES_PASSWORD", "validation_pass")
        # connection_string = f"dbname={db_name} user={db_user} password={db_password} host={db_host} port={db_port}"
        # self._db_client = DatabaseClient(connection_string)

        # the inventory manager
                # the inventory manager
        self._inventory_client = InventoryManager(self._db_client)
        # initialize the logging
        logging.basicConfig(level=getattr(logging, config.log_level))
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # Initialize production coffee detector with config
        try:
            # Load config from coffee_detection directory
            config_path = os.path.join(os.path.dirname(__file__), 'coffee_detection', 'detection_config.json')
            detection_config = load_config(config_path)
            
            # Fix debug directory path to be absolute
            debug_dir = os.path.join(os.path.dirname(__file__), 'coffee_detection', 'debug_frames_coffee')
            detection_config.debug_frame_dir = debug_dir
            
            self._coffee_beans_detector = ProductionCoffeeDetector(detection_config)
            log("INFO", f"Production coffee detector initialized successfully. Debug frames will be saved to: {debug_dir}", service="validation")
        except Exception as e:
            log("ERROR", f"Failed to initialize production coffee detector: {e}", service="validation")
            # Fallback to dummy detector if production detector fails
            from .coffee_beans_detector import CoffeeBeansDetector
            self._coffee_beans_detector = CoffeeBeansDetector()
            log("WARNING", "Falling back to dummy coffee detector", service="validation")

        # Initialize cup detector - ADD THIS BLOCK
        try:
            cup_detector_config_path = os.path.join(os.path.dirname(__file__), "cup_detection", "config.py")
            self._cup_detector = CupDetector(cup_detector_config_path)
            
            # Fix debug directory path to be absolute (same as coffee detection)
            debug_dir = os.path.join(os.path.dirname(__file__), 'cup_detection', 'debug_frames_cup')
            self._cup_detector.config.debug_folder = debug_dir
            
            # Create debug directory if it doesn't exist
            if self._cup_detector.config.debug_mode or self._cup_detector.config.save_frames:
                os.makedirs(debug_dir, exist_ok=True)
            
            log("INFO", f"Cup detector initialized successfully. Debug frames will be saved to: {debug_dir}", service="validation")
            
            # TEST CUP DETECTION - COMMENT OUT LATER
            try:
                log("DEBUG", "Testing cup detection on initialization...", service="validation")
                test_result = self._cup_detector.detect()
                log("DEBUG", f"Cup detection test result: {test_result}", service="validation")
                if "error" not in test_result:
                    detected_count = sum(1 for present in test_result.values() if present)
                    log("DEBUG", f"Cup detection working! Detected {detected_count} cups", service="validation")
                else:
                    log("DEBUG", f"Cup detection error: {test_result['error']}", service="validation")
            except Exception as test_e:
                log("DEBUG", f"Cup detection test failed: {test_e}", service="validation")
            # END TEST CODE
            
        except Exception as e:
            log("ERROR", f"Failed to initialize cup detector: {e}", service="validation")
            self._cup_detector = None

        # # Queues to receive requests and process responses
        # self._request_queue = Queue()
        # self._response_queue = Queue()

        # # the workers
        # self._request_worker = threading.Thread(target=self.request_worker, daemon=True)
        # self._response_worker = threading.Thread(target=self.response_worker, daemon=True)

        # # event flags for adding request and response
        # self._request_event = threading.Event()
        # self._response_event = threading.Event()

        # Thread pool for blocking operations
        self._thread_pool = ThreadPoolExecutor(max_workers=config.detection.max_detection_workers, thread_name_prefix="detection_worker")
        # Detection task control
        self._detection_task = None
        self._detection_running = False


    # def post_request(self, request):
    #     try:
    #         # # check if it is a valid request using pydantic !! ALWAYS VALID THOUGH !!
    #         # if not request or not request.payload or not request.payload.items:
    #         #     # raise a validation error
    #         #     raise HTTPException(status_code=422, detail="Invalid request")
    #         # # log the request
    #         logging.info(f"received request: {request} with request_id: {request.request_id}")
    #         # if the request is valid, add it to the queue
    #         self._request_queue.put(request)
    #         # raise the event flag
    #         self._request_event.set()
    #     except Exception as e:
    #         print(e) 
    #         print("failed to add request to queue")
    #         # log the error
    #         logging.error(f"failed to add request to queue: {e}")


    def process_update_inventory_request(self, payload):
        """
        Process update inventory request by updating each ingredient in the inventory. 
        Used by Scheduler/OMS to subtract inventory after use
        Used by Dashboard to add or subtract inventory after use
        """
        try:
            # Initialize result tracking
            result = {"passed": True, "details": {}}

            # Add request metadata to result
            result["request_id"] = payload["request_id"]
            result["client_type"] = payload["client_type"]
            
            # Process each item's ingredients
            for item in payload["payload"]["ingredients"]:
                for ingredient, details in item.items():
                    # Convert espresso to coffee_beans
                    ingredient_type = "coffee_beans" if ingredient == "espresso" else ingredient
                    # changes_by_mais: why to not use one: cup or cups
                    if ingredient == "cup":
                        ingredient_type = "cups"

                    subtype = details["type"]
                    amount = details["amount"]

                    # if the client type is scheduler, then we need to subtract the amount from the inventory
                    if payload["client_type"] == "scheduler":
                        amount = -amount

                    # Update inventory
                    success, warning = self._inventory_client.update_inventory(
                        ingredient_type=ingredient_type,
                        subtype=subtype,
                        amount=amount  # Negative amount to subtract from inventory
                    )
                    print(f"success: {success}, warning: {warning}")
                    # to be discussed: why the type and subtype in this format: "coffee_beans:regular"
                    if not success:
                        result["passed"] = False
                        result["details"][ingredient_type] = {
                            "type": subtype,
                            "updated_amount": 0,
                            "status": "failed",
                            "message": "Failed to update inventory"
                        }
                    elif warning in ["no_warning", "warning", "critical"]:
                        if ingredient_type in result["details"] and subtype in result["details"][ingredient_type].values():
                            result["details"][ingredient_type]["updated_amount"] += amount
                        else:
                            result["details"][ingredient_type] = {
                                "type": subtype,
                                "updated_amount": amount, # changes_by_mais: should it be the absolute value? or the inventory value?
                                "status": warning,
                                "message": f"Inventory {warning} level reached"
                            }

            # # Put result in response queue
            # self._response_queue.put(result)
            print("result after update inventory request")
            print(result)
            return result
            
        except Exception as e:
            log("ERROR", f"Error processing update inventory request: {e}", service="validation")
            error_result = {
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "passed": False,
                "details": {"error": str(e)}
            }
            # self._response_queue.put(error_result)
            return error_result



    # def process_ingredient_status_request(self, payload):
    #     # @Uzair verify this works properly
    #     # i believe the structure of the request should be without any item in the payload
    #     """ !!!!!!! NOTE: @Uzair refactor this function to be more efficient and readable
    #     Used to get the inventory status for the entire inventory OR a specific item in the inventory
    #     """
    #     try:
    #         inventory_status = {}
    #         if payload["client_type"] == "dashboard" or payload["client_type"] == "api_bridge":
                
    #             for ingredient_type, subtypes in self._inventory_client.inventory_cache.items():
    #                 inventory_status[ingredient_type] = {}

    #                 for subtype, data in subtypes.items():
    #                     current_amount = data["current_amount"]
    #                     warning_threshold = data["warning_threshold"]
    #                     critical_threshold = data["critical_threshold"]

    #                     status = "full"
    #                     final_res = True
    #                     if current_amount < critical_threshold:
    #                         status = "empty"
    #                         final_res = False
    #                     elif current_amount < warning_threshold:
    #                         status = "low"

    #                     inventory_status[ingredient_type][subtype] = {
    #                         "status": status,
    #                         "current_amount": current_amount,
    #                         "warning_threshold": warning_threshold,
    #                         "critical_threshold": critical_threshold,
    #                         "final_res": final_res #final_res is False if the inventory is empty when the amount is less than the critical threshold
    #                     }
                        
    #                     # another suggestion for response structure:
    #                     # inventory_status[ingredient_type][subtype] = {
    #                     #     "status": status, # better to be high, medium, low
    #                     #     "current_amount": current_amount,
    #                     # }
            
    #         else:
    #             # invalid client type
    #             inventory_status = {"final_res": False, "details": "Invalid client type"}
    #         final_result = { "passed": True, "request_id": payload["request_id"],
    #             "client_type": payload["client_type"], "details": inventory_status}
    #         self._response_queue.put(final_result)
    #         self._response_event.set()
    #         return final_result

    #     except Exception as e:
    #         logging.error(f"Error processing inventory status request: {e}")
    #         error_result = {
    #             "passed": False,
    #             "request_id": payload["request_id"],
    #             "client_type": payload["client_type"],
    #             "result": {
    #                 "final_res": False,
    #                 "details": f"Error processing request: {str(e)}"
    #             }
    #         }
    #         self._response_queue.put(error_result)
    #         # NOTE: @ UZAIR fix this to make sure the result is sent to the response queue
    #         self._response_event.set()
    #         return error_result

    
    def process_pre_check_request(self, payload):
        # NOTE: THIS IS PRE-CHECK REQUEST
        try: 
            result = {"passed": True, "details": {}}
            # Add request metadata to result
            result["request_id"] = payload["request_id"]
            result["client_type"] = payload["client_type"]


            if payload["client_type"] == "scheduler":
                # get the invenoty cache
                current_inventory_cache = self._inventory_client.inventory_cache.copy()
                
                for item in payload["payload"]["items"]:
                    item_details = {}
                    # set the status for the item to true
                    item_details["status"] = True
                    
                    # Check cup inventory
                    cup_id = item["cup_id"]
                    if cup_id in current_inventory_cache["cups"]:
                        current_amount = current_inventory_cache["cups"][cup_id]["current_amount"]
                        critical_threshold = current_inventory_cache["cups"][cup_id]["critical_threshold"]
                        if current_amount - 1 < critical_threshold:
                            result["passed"] = False
                            item_details["status"] = False
                        item_details["cup"] = {
                            "type": cup_id,
                            "current": current_amount,
                            "needed": 1,
                            "critical_threshold": critical_threshold,
                            "status": False if current_amount - 1 < critical_threshold else True
                        }
                        if item_details["cup"]["status"] == True:
                            # update the inventory cache
                            current_inventory_cache["cups"][cup_id]["current_amount"] = current_amount - 1

                    # Check other ingredients
                    for ingredient, details in item["ingredients"].items():
                        if ingredient == "espresso":
                            ingredient_type = "coffee_beans"
                        else:
                            ingredient_type = ingredient
                            
                        if ingredient_type in self._inventory_client.inventory_cache:
                            subtype = details["type"]
                            amount = details["amount"]
                            if ingredient_type == "coffee_beans":
                                # get the amount against the shot using the self._inventory_client.convert_shots_to_grams(amount)
                                amount = self._inventory_client.convert_shots_to_grams(item["ingredients"]["coffee_beans"]["amount"])
                            
                            if subtype in current_inventory_cache[ingredient_type]:
                                current_amount = current_inventory_cache[ingredient_type][subtype]["current_amount"]
                                critical_threshold = current_inventory_cache[ingredient_type][subtype]["critical_threshold"]
                                
                                if current_amount - amount < critical_threshold:
                                    result["passed"] = False
                                    item_details["status"] = False
                                    
                                item_details[ingredient] = {
                                    "type": subtype,
                                    "current": current_amount,
                                    "needed": amount,
                                    "critical_threshold": critical_threshold,
                                    "status": False if current_amount - amount < critical_threshold else True
                                }
                                if item_details[ingredient]["status"] == True:
                                # update the inventory cache
                                    current_inventory_cache[ingredient_type][subtype]["current_amount"] = current_amount - amount
                    

                    result["details"][item["drink_name"]] = item_details
                print(result)

            else:
                # invalid client type
                result = {"request_id": result['request_id'], 
                          "client_type": result['client_type'], 
                          "passed": False, 
                          "details": "Invalid client type"}
                
            # self._response_queue.put(result)
            # self._response_event.set()
            return result

        except Exception as e:
            log("ERROR", f"Error processing pre-check request: {e}", service="validation")
            error_result = {
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "passed": False,
                "details": f"Error processing request: {str(e)}"
            }
            # self._response_queue.put(error_result)
            # NOTE: @ UZAIR fix this to make sure the result is sent to the response queue
            # self._response_event.set()
            return error_result

    # def process_refill_ingredient_request(self, payload):
    #     try:
    #         result = {"passed": True, "details": {}}
    #         result["request_id"] = payload["request_id"]
    #         result["client_type"] = payload["client_type"]

    #         for ingredient in payload["payload"]["ingredients"]:
    #             ingredient_type = ingredient["ingredient_type"]
    #             subtype = ingredient["subtype"]

    #             if ingredient_type == "espresso":
    #                 ingredient_type = "coffee_beans"
    #             elif ingredient_type == "cup":
    #                 ingredient_type = "cups"

    #             is_refilled = self._inventory_client.refill_inventory(ingredient_type, subtype)
                
    #             if not is_refilled:
    #                 result["passed"] = False
    #                 result["details"][f"{ingredient_type}"] = {
    #                     "type": subtype,
    #                     "status": "failed",
    #                     "message": "Failed to refill inventory"
    #                 }
                
    #             else:
    #                 result["details"][f"{ingredient_type}"] = {
    #                     "type": subtype,
    #                     "status": "success",
    #                     "message": "Inventory refilled successfully"
    #                 }
            
    #         self._response_queue.put(result)
    #         self._response_event.set()
    #         return result
            
    #     except Exception as e:
    #         logging.error(f"Error processing refill ingredient request: {e}")
    #         error_result = {
    #             "request_id": payload["request_id"],
    #             "client_type": payload["client_type"],
    #             "passed": False,
    #             "details": f"Error processing request: {str(e)}"
    #         }
    #         self._response_queue.put(error_result)
    #         return error_result
    
    
    def process_refill_ingredient_request(self, payload):
        try:
            # Extract parameters from payload
            ingredient_type = payload.get("payload", {}).get("ingredient_type", None)
            subtype = payload.get("payload", {}).get("subtype", None)
            print(f"inside process_refill_ingredient_request: ingredient_type: {ingredient_type}, subtype: {subtype}")

            result = {"passed": True, "details": {}}
            result["request_id"] = payload["request_id"]
            result["client_type"] = payload["client_type"]

            # Check if we need coffee beans detection for regular coffee
            needs_coffee_detection = (
                (ingredient_type == "coffee_beans" and subtype == "regular") or 
                (ingredient_type == "coffee_beans" and subtype is None) or 
                (ingredient_type is None and subtype is None)  # Full refill
            )
            print(f"needs_coffee_detection: {needs_coffee_detection}")

            coffee_detection_success = True
            
            # Handle coffee beans regular detection if needed
            if needs_coffee_detection:
                detection_result = self._run_coffee_beans_detection(function_name="inventory_refill")
                
                if detection_result["success"] and detection_result.get("updated"):
                    result["details"]["coffee_beans_message"] = f"Coffee beans regular refilled successfully with {detection_result['percentage']}% detected"
                    result["details"]["coffee_beans_percentage"] = detection_result["percentage"]
                    coffee_detection_success = True
                elif detection_result["success"] and not detection_result.get("updated"):
                    # Detection successful but percentage <= 0
                    result["passed"] = False
                    result["details"]["error"] = detection_result["message"]
                    result["details"]["alert_type"] = detection_result.get("alert_type", "visibility_issue")
                    coffee_detection_success = False
                else:
                    # Detection failed - camera issue
                    result["passed"] = False
                    result["details"]["error"] = detection_result["message"]
                    result["details"]["alert_type"] = detection_result.get("alert_type", "camera_reconnect")
                    coffee_detection_success = False

            # Handle normal refill for other ingredients (skip coffee regular if detection was used)
            normal_refill_success = True
            
            if coffee_detection_success:  # Only proceed if coffee detection succeeded (or wasn't needed)
                if ingredient_type == "coffee_beans" and subtype == "regular":
                    # Coffee beans regular only - already handled by detection, no normal refill needed
                    pass
                else:
                    # All other cases: use normal refill with skip_coffee_regular flag when needed
                    skip_coffee_regular = needs_coffee_detection  # Skip if we already handled it with detection
                    
                    normal_refill_success = self._inventory_client.refill_inventory(
                        ingredient_type=ingredient_type,
                        subtype=subtype,
                        skip_coffee_regular=skip_coffee_regular
                    )
                    
                    if not normal_refill_success:
                        result["passed"] = False
                        if "error" not in result["details"]:  # Don't override coffee detection errors
                            result["details"]["error"] = f"Failed to refill {ingredient_type}:{subtype}"
                    else:
                        if "coffee_beans_message" not in result["details"]:
                            result["details"]["message"] = f"Successfully refilled {ingredient_type}:{subtype}"

            # Final result
            result["passed"] = coffee_detection_success and normal_refill_success

            log("INFO", f"Refill ingredient request result: {json.dumps(result, indent=2)}", service="validation")
            # self._response_queue.put(result)
            # self._response_event.set()
            return result
            
        except Exception as e:
            log("ERROR", f"Error processing refill ingredient request: {e}", service="validation")
            error_result = {
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "passed": False,
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result
        
    def process_ingredient_status_request(self, payload):
        """
        Process ingredient status request with flexible filtering
        """
        try:
            # Extract parameters from payload
            ingredient_type = payload.get("payload", {}).get("ingredient_type", None)
            print(f"****ingredient_status_request: {json.dumps(payload, indent=2)}")
            subtype = payload.get("payload", {}).get("subtype", None)
            print("###################################")
            print(ingredient_type, subtype)
            print("###################################")
            print(payload)
            # Get status from inventory manager
            inventory_status = self._inventory_client.get_inventory_status(
                ingredient_type=ingredient_type,
                subtype=subtype
            )
            print(f"inventory_status: {json.dumps(inventory_status, indent=2)}")
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": inventory_status
            }
            
            # self._response_queue.put(final_result)
            # self._response_event.set()
            return final_result
            
        except Exception as e:
            log("ERROR", f"Error processing inventory status request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result
    
    def process_category_info_request(self, payload):
        """Process category info request"""
        try:
            category_info = self._inventory_client.get_inventory_category_info()
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": category_info
            }
            # self._response_queue.put(final_result)
            # self._response_event.set()
            return final_result
        
        except Exception as e:
            log("ERROR", f"Error processing category info request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result
        

    def process_category_summary_request(self, payload):
        """Process category summary request"""
        try:
            print("###################################")
            print("process_category_summary_request")
            print(f"payload: {json.dumps(payload, indent=2)}")
            category_summary = self._inventory_client.get_category_summary()
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": category_summary
            }
            print(f"final_result: {json.dumps(final_result, indent=2)}")
            
            # self._response_queue.put(final_result)
            # self._response_event.set()
            return final_result
            
        except Exception as e:
            log("ERROR", f"Error processing category summary request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result

    def process_category_count_request(self, payload):
        """Process category count request"""
        try:
            category_count = self._inventory_client.get_category_count()
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": category_count
            }
            
            # self._response_queue.put(final_result)
            # self._response_event.set()
            return final_result
        
        except Exception as e:
            log("ERROR", f"Error processing category count request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result
        

    def process_stock_level_request(self, payload):
        """Process inventory stock level statistics request"""
        try:
            stock_level = self._inventory_client.get_inventory_stock_level_stats()
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": stock_level
            }
            
            # self._response_queue.put(final_result)
            # self._response_event.set()
            return final_result
            
        except Exception as e:
            log("ERROR", f"Error processing inventory severity request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            # self._response_queue.put(error_result)
            # self._response_event.set()
            return error_result
        
    
    
    def process_inventory_by_stock_level_request(self, payload):
        """Process inventory by stock level request"""
        try:
            stock_level = payload.get("payload", {}).get("stock_level")
            
            if not stock_level:
                return {
                    "passed": False,
                    "request_id": payload["request_id"],
                    "client_type": payload["client_type"],
                    "details": {"error": "Stock level parameter is required"}
                }
            
            # Get filtered inventory from inventory manager
            filtered_inventory = self._inventory_client.get_inventory_by_stock_level(stock_level)
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": filtered_inventory
            }
            
            return final_result
            
        except Exception as e:
            log("ERROR", f"Error processing inventory by stock level request: {e}", service="validation")
            error_result = {
                "passed": False,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": {"error": f"Error processing request: {str(e)}"}
            }
            return error_result
        

            
    
    # def request_worker(self):
    #     while True:
    #         try:
    #             self._request_event.wait()
    #             request = self._request_queue.get()
    #             self._request_event.clear()  # Clear the event flag
                
    #             if request["function_name"] == "update_inventory":
    #                 self.process_update_inventory_request(request)
    #             elif request["function_name"] == "ingredient_status" or request["function_name"] == "pre_check":
    #                 self.process_ingredient_status_request(request)
    #             else:
    #                 logging.error(f"Invalid function name: {request['function_name']}")
    #         except Exception as e:
    #             logging.error(f"Error processing request: {e}")


    # def response_worker(self):
    #     while True:
    #         try:
    #             self._response_event.wait()
    #             response = self._response_queue.get()
    #             ####################
    #             # @NOTE: @Uzair @Mais work with sending the response to the client here
    #             ## Ideally have a separate object to handle this
    #             print(response)
    #         #####################
    #             self._response_event.clear()
    #         except Exception as e:
    #             logging.error(f"Error processing response: {e}")


    async def start_periodic_detection(self):
        """Start the periodic coffee beans detection task"""
        if not config.detection.enable_periodic_detection:
            log("INFO", "Periodic detection disabled by configuration", service="validation")
            return
        
        if self._detection_task is None or self._detection_task.done():
            self._detection_running = True
            self._detection_task = asyncio.create_task(self._periodic_detection_loop())
            log("INFO", f"Started periodic coffee beans detection (every {config.detection.periodic_interval_minutes} minutes)", service="validation")

    async def stop_periodic_detection(self):
        """Stop the periodic coffee beans detection task"""
        self._detection_running = False
        if self._detection_task and not self._detection_task.done():
            self._detection_task.cancel()
            try:
                await self._detection_task
            except asyncio.CancelledError:
                pass
        log("INFO", "Stopped periodic coffee beans detection", service="validation")

    async def _periodic_detection_loop(self):
        """Main loop for periodic coffee beans detection"""
        while self._detection_running:
            try:
                log("INFO", "Starting coffee beans detection...", service="validation")
                
                # Run the blocking detection in thread pool
                loop = asyncio.get_event_loop()
                detection_result = await loop.run_in_executor(
                    self._thread_pool, 
                    self._run_coffee_beans_detection
                )
                
                # Log the result
                if detection_result.get("updated"):
                    log("INFO", f"Periodic detection updated inventory: {detection_result['percentage']}%", service="validation")
                else:
                    log("INFO", f"Periodic detection completed without update: {detection_result['message']}", service="validation")
                
            except asyncio.CancelledError:
                log("INFO", "Coffee beans detection task cancelled", service="validation")
                break
            except Exception as e:
                log("ERROR", f"Error in coffee beans detection: {e}", service="validation")
            
            # Wait for 10 minutes before next detection
            try:
                interval = config.detection.periodic_interval_seconds
                log("DEBUG", f"Waiting {interval} seconds ({config.detection.periodic_interval_minutes} minutes) until next detection", service="validation")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break

    def _run_coffee_beans_detection(self, function_name: str = "periodic_detection"):
        """Wrapper method to run detection in thread pool (this runs in a separate thread)"""
        try:
            # Use the production detector's detect_coffee method
            cv_result = self._coffee_beans_detector.detect_coffee()
            print(f"cv_result: {cv_result}") # convert to logger
            log("INFO", f"cv_result: {cv_result}", service="validation")
            
            # Check if there was an error in detection
            if cv_result.get("error"):
                # Detection failed - return error without raising exception
                return {
                    "success": False,
                    "updated": False,
                    "percentage": 0,
                    "timestamp": datetime.datetime.now().isoformat(),
                    "message": f"Detection failed: {cv_result['error']}",
                    "alert_type": "camera_reconnect"
                }
            
            # Detection successful - use the percentage directly
            percentage = cv_result.get("percentage", 0)
            
            if function_name == "periodic_detection":
                # Case 1: Periodic detection every 10 minutes
                if percentage > 0:
                    # Update inventory based on detected percentage
                    success = self._inventory_client.update_inventory_from_detection(percentage)
                    return {
                        "success": True,
                        "updated": True,
                        "percentage": percentage,
                        "timestamp": datetime.datetime.now().isoformat(),
                        "message": f"Periodic detection successful, inventory updated with {percentage}% detected"
                    }
                else:
                    # Percentage <= 0, don't update inventory
                    return {
                        "success": True,
                        "updated": False,
                        "percentage": percentage,
                        "timestamp": datetime.datetime.now().isoformat(),
                        "message": "Periodic detection completed, no inventory update (percentage <= 0)"
                    }
                    
            if function_name == "inventory_refill":
                # Case 4: Refill operation
                if percentage > 0:
                    # Update inventory based on detected percentage
                    success = self._inventory_client.update_inventory_from_detection(percentage)
                    return {
                        "success": True,
                        "updated": True,
                        "percentage": percentage,
                        "timestamp": datetime.datetime.now().isoformat(),
                        "message": f"Refill detection successful, inventory updated with {percentage}% detected"
                    }
                else:
                    # Percentage <= 0, send alert about visibility issue
                    return {
                        "success": True,
                        "updated": False,
                        "percentage": percentage,
                        "timestamp": datetime.datetime.now().isoformat(),
                        "message": "Refill detection failed - coffee beans should be above the unseen area",
                        "alert_type": "visibility_issue"
                    }
            else:
                # Default case - just return detection result
                return {
                    "success": True,
                    "result": {"percentage": percentage},
                    "timestamp": datetime.datetime.now().isoformat(),
                    "message": "Coffee beans detection completed successfully"
                }
                
        except Exception as e:
            # Unexpected exception during detection call
            log("ERROR", f"Unexpected error in coffee beans detection: {e}", service="validation")
            
            if function_name == "inventory_refill":
                # Case 4: Unexpected error during refill - alert to reconnect camera
                return {
                    "success": False,
                    "updated": False,
                    "error": str(e),
                    "timestamp": datetime.datetime.now().isoformat(),
                    "message": "Unexpected error during refill detection",
                    "alert_type": "camera_reconnect"
                }
            else:
                # Case 1: Unexpected error during periodic - keep current amount
                return {
                    "success": False,
                    "updated": False,
                    "error": str(e),
                    "timestamp": datetime.datetime.now().isoformat(),
                    "message": "Unexpected error in detection, keeping current inventory amount"
                }
            

    async def cleanup(self):
        """Cleanup resources when shutting down"""
        await self.stop_periodic_detection()
        
        # Shutdown the thread pool
        self._thread_pool.shutdown(wait=True)
        log("INFO", "MainValidation cleanup completed", service="validation")

    def process_cup_detection_request(self, payload):
        """Process cup detection requests"""
        try:
            result = {"passed": False, "details": {}}
            # Add request metadata to result
            result["request_id"] = payload.get("request_id")
            result["client_type"] = payload.get("client_type")

            if not self._cup_detector:
                result["error"] = "Cup detector not initialized"
                return result

            # Run cup detection
            detection_result = self._cup_detector.detect()
            log("INFO", f"detection_result: {detection_result}", service="validation")
            if "error" in detection_result:
                result["error"] = detection_result["error"]
                return result
            
            # Check if any cups are detected
            cups_detected = detection_result  # {1: bool, 2: bool, 3: bool, 4: bool} (1-indexed, reversed)
            total_cups = len(cups_detected)
            detected_count = sum(1 for present in cups_detected.values() if present)
            
            result["passed"] = True
            result["detection_result"] = cups_detected  # Add at top level for easy access

            result["details"] = {
                "cups_detected": cups_detected,
                "total_positions": total_cups,
                "detected_count": detected_count,
                "message": f"Detected {detected_count} out of {total_cups} cup positions"
            }
            
            return result
            
        except Exception as e:
            log("ERROR", f"Error in cup detection: {e}", service="validation")
            return {
                "request_id": payload.get("request_id"),
                "client_type": payload.get("client_type"),
                "passed": False,
                "error": f"Cup detection failed: {str(e)}"
            }