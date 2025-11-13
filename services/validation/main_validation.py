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
from .cup_detection.cup_detector import CupDetector
from .config import get_db_connection_string, config

# ID-to-Subtype mappings for routine service integration
# These map numeric IDs used by routine service to inventory database subtypes
MILK_ID_TO_SUBTYPE = {
    1: "whole_fat_milk",      # whole_fat -> whole_fat_milk
    2: "almond_milk",          # almond -> almond_milk
    3: "oat_milk",             # oat -> oat_milk (if exists in inventory)
    4: "soy_milk",             # soy -> soy_milk (if exists in inventory)
    5: None,                   # normal_water -> handled separately as water
    6: "lactose_free_milk",    # lactose_free -> lactose_free_milk
    7: "low_fat_milk",         # low_fat -> low_fat_milk
}

SYRUP_ID_TO_SUBTYPE = {
    9: "white_chocolate_sauce",   # white_chocolate -> white_chocolate_sauce
    10: "caramel_sauce",          # caramel_sauce -> caramel_sauce
    11: "condense_milk_sauce",    # condense_milk -> condense_milk_sauce
    12: "hazelnut_syrup",         # hazelnut -> hazelnut_syrup
    13: "vanilla_syrup",          # vanilla -> vanilla_syrup
    14: "peached_iced_syrup",     # peach_iced_tea -> peached_iced_syrup
    15: "passion_fruit_iced_syrup", # passion_fruit_puree -> passion_fruit_iced_syrup
    16: "ice_tea_syrup",          # ice_tea -> ice_tea_syrup
}

# Water is special - ID 5 in MILK_MAPPINGS
# Note: Water may not be tracked in inventory (unlimited supply)
WATER_ID_TO_SUBTYPE = {
    5: "normal_water",  # normal_water - may not exist in inventory database
}



class MainValidation:
    def __init__(self):
        self._db_client = DatabaseClient(get_db_connection_string())

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
                log("INFO", "Testing cup detection on initialization...", service="validation")
                test_result = self._cup_detector.detect_cups_on_station()
                log("INFO", f"Cup detection test result: {test_result}", service="validation")
                if "error" not in test_result:
                    detected_count = sum(1 for present in test_result.values() if present)
                    log("INFO", f"Cup detection working! Detected {detected_count} cups", service="validation")
                else:
                    log("INFO", f"Cup detection error: {test_result['error']}", service="validation")
            except Exception as test_e:
                log("INFO", f"Cup detection test failed: {test_e}", service="validation")
            # END TEST CODE
            
        except Exception as e:
            log("ERROR", f"Failed to initialize cup detector: {e}", service="validation")
            self._cup_detector = None



        # Thread pool for blocking operations
        self._thread_pool = ThreadPoolExecutor(max_workers=config.detection.max_detection_workers, thread_name_prefix="detection_worker")
        # Detection task control
        self._detection_task = None
        self._detection_running = False

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
    
    def process_pre_check_request(self, payload):
        # NOTE: THIS IS PRE-CHECK REQUEST
        try: 
            result = {"passed": True, "details": {}}
            # Add request metadata to result
            result["request_id"] = payload["request_id"]
            result["client_type"] = payload["client_type"]
            log("INFO", f"Pre-check request initiated", service="validation")
            print(f"payload: {payload}")

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

            else:
                # invalid client type
                result = {"request_id": result['request_id'], 
                          "client_type": result['client_type'], 
                          "passed": False, 
                          "details": "Invalid client type"}
                
            # self._response_queue.put(result)
            # self._response_event.set()
            log("INFO", f"Pre-check request result: {result['passed']}", service="validation")
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

    def process_refill_ingredient_request(self, payload):
        try:
            # Extract parameters from payload
            ingredient_type = payload.get("payload", {}).get("ingredient_type", None)
            subtype = payload.get("payload", {}).get("subtype", None)

            result = {"passed": True, "details": {}}
            result["request_id"] = payload["request_id"]
            result["client_type"] = payload["client_type"]

            # Check if we need coffee beans detection for regular coffee
            needs_coffee_detection = (
                (ingredient_type == "coffee_beans" and subtype == "regular") or 
                (ingredient_type == "coffee_beans" and subtype is None) or 
                (ingredient_type is None and subtype is None)  # Full refill
            )

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
            subtype = payload.get("payload", {}).get("subtype", None)
            # Get status from inventory manager
            inventory_status = self._inventory_client.get_inventory_status(
                ingredient_type=ingredient_type,
                subtype=subtype
            )
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
            category_summary = self._inventory_client.get_category_summary()
            
            final_result = {
                "passed": True,
                "request_id": payload["request_id"],
                "client_type": payload["client_type"],
                "details": category_summary
            }
            
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
                log("INFO", "Starting periodic coffee beans detection...", service="validation")
                
                # Run the blocking detection in thread pool
                loop = asyncio.get_event_loop()
                detection_result = await loop.run_in_executor(
                    self._thread_pool, 
                    self._run_coffee_beans_detection
                )
                
                # Log the result
                if detection_result.get("updated"):
                    log("INFO", f"Coffee inventory updated: {detection_result['percentage']}% (periodic scan)", service="validation")
                else:
                    log("INFO", f"☕ Periodic detection completed without update: {detection_result['message']}", service="validation")
                
            except asyncio.CancelledError:
                log("INFO", "Coffee beans detection task cancelled", service="validation")
                break
            except Exception as e:
                log("ERROR", f"Error in coffee beans detection: {str(e)[:100]}", service="validation")
            
            # Wait for 10 minutes before next detection
            try:
                interval = config.detection.periodic_interval_seconds
                log("INFO", f"Next coffee scan in {config.detection.periodic_interval_minutes} minutes", service="validation")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break

    def _run_coffee_beans_detection(self, function_name: str = "periodic_detection"):
        """Wrapper method to run detection in thread pool (this runs in a separate thread)"""
        try:
            # Use the production detector's detect_coffee method
            cv_result = self._coffee_beans_detector.detect_coffee()
            log("INFO", f"Coffee detection raw result: {cv_result}", service="validation")
            
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
            detection_result = self._cup_detector.detect_cups_on_station()
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
    
    def process_milk_detection_request(self, payload):
        """Process milk dispenser cup detection requests"""
        try:
            result = {"passed": False, "details": {}}
            # Add request metadata to result
            result["request_id"] = payload.get("request_id")
            result["client_type"] = payload.get("client_type")

            if not self._cup_detector:
                result["error"] = "Cup detector not initialized"
                return result

            # Run milk detection
            detection_result = self._cup_detector.detect_cup_milk_dispenser()
            log("INFO", f"milk_detection_result: {detection_result}", service="validation")
            
            if isinstance(detection_result, dict) and "error" in detection_result:
                result["error"] = detection_result["error"]
                return result
            
            # detection_result is a bool
            cup_detected = bool(detection_result)
            
            result["passed"] = True
            result["detection_result"] = cup_detected

            result["details"] = {
                "cup_detected": cup_detected,
                "message": f"Milk dispenser: Cup {'detected' if cup_detected else 'not detected'}"
            }
            
            return result
            
        except Exception as e:
            log("ERROR", f"Error in milk detection: {e}", service="validation")
            return {
                "request_id": payload.get("request_id"),
                "client_type": payload.get("client_type"),
                "passed": False,
                "error": f"Milk detection failed: {str(e)}"
            }
    
    def process_sauce_detection_request(self, payload):
        """Process sauce dispenser cup detection requests"""
        try:
            result = {"passed": False, "details": {}}
            # Add request metadata to result
            result["request_id"] = payload.get("request_id")
            result["client_type"] = payload.get("client_type")

            if not self._cup_detector:
                result["error"] = "Cup detector not initialized"
                return result

            # Run sauce detection
            detection_result = self._cup_detector.detect_cup_sauce_dispenser()
            log("INFO", f"sauce_detection_result: {detection_result}", service="validation")
            
            if isinstance(detection_result, dict) and "error" in detection_result:
                result["error"] = detection_result["error"]
                return result
            
            # detection_result is a bool
            cup_detected = bool(detection_result)
            
            result["passed"] = True
            result["detection_result"] = cup_detected

            result["details"] = {
                "cup_detected": cup_detected,
                "message": f"Sauce dispenser: Cup {'detected' if cup_detected else 'not detected'}"
            }
            
            return result
            
        except Exception as e:
            log("ERROR", f"Error in sauce detection: {e}", service="validation")
            return {
                "request_id": payload.get("request_id"),
                "client_type": payload.get("client_type"),
                "passed": False,
                "error": f"Sauce detection failed: {str(e)}"
            }
    
    def process_ingredient_validation_request(self, payload):
        """
        Process ingredient validation requests from routine service.
        
        Validates ingredients sent in the routine format:
        {
            'request_id': 'routine-cup123-...',
            'client_type': 'routine',
            'cup_id': 'cup123',
            'milk': {1: 110.0},
            'syrups': {10: 5.0, 13: 8.0},
            'cups': {'cup_H9': 1.0},
            'espresso': {'espresso_shot_single': 1.0},
            'water': {5: 100.0},
            ...
        }
        
        Returns validation result indicating whether all ingredients are available.
        """
        try:
            log("INFO", f"=== INGREDIENT VALIDATION START ===", service="validation")
            log("INFO", f"Payload type: {type(payload)}", service="validation")
            log("INFO", f"Payload keys: {list(payload.keys()) if isinstance(payload, dict) else 'NOT A DICT'}", service="validation")
            
            result = {"passed": True, "details": {}}
            result["request_id"] = payload.get("request_id")
            result["client_type"] = payload.get("client_type")
            result["cup_id"] = payload.get("cup_id")
            
            # Get current inventory cache
            current_inventory = self._inventory_client.inventory_cache.copy()
            
            # Define ingredient categories to check (skip non-ingredient keys)
            skip_keys = {'request_id', 'client_type', 'cup_id', 'position', 'temperature'}
            
            # Check each ingredient in the payload
            for ingredient_key, ingredient_data in payload.items():
                # Skip non-ingredient keys
                if ingredient_key in skip_keys:
                    continue
                
                # Validate that ingredient_data is a dict
                if not isinstance(ingredient_data, dict):
                    log("INFO", f"Skipping non-dict value for key '{ingredient_key}': {type(ingredient_data)}", service="validation")
                    continue
                
                # Skip if the dict is empty
                if not ingredient_data:
                    log("INFO", f"Skipping empty dict for key '{ingredient_key}'", service="validation")
                    continue
                
                # Map ingredient names to inventory categories
                ingredient_type = ingredient_key
                if ingredient_key == "espresso":
                    ingredient_type = "coffee_beans"
                elif ingredient_key == "cups":
                    ingredient_type = "cups"
                
                # Check if this ingredient type exists in inventory
                if ingredient_type not in current_inventory:
                    log("WARNING", f"Ingredient type '{ingredient_type}' not found in inventory", service="validation")
                    continue
                
                # Validate each subtype/variant in this ingredient
                for subtype_key, amount in ingredient_data.items():
                    # Validate that amount is numeric
                    if not isinstance(amount, (int, float)):
                        log("WARNING", f"Skipping non-numeric amount for {ingredient_key}:{subtype_key}: {amount}", service="validation")
                        continue
                    # For espresso, convert to coffee beans and check regular subtype
                    if ingredient_key == "espresso":
                        # Convert shots to grams for coffee beans
                        amount_grams = self._inventory_client.convert_shots_to_grams(amount)
                        subtype = "regular"  # Coffee beans regular is used for espresso
                        
                        # Check coffee beans inventory
                        if subtype in current_inventory[ingredient_type]:
                            current_amount = current_inventory[ingredient_type][subtype]["current_amount"]
                            critical_threshold = current_inventory[ingredient_type][subtype]["critical_threshold"]
                            
                            log("INFO", f"[INGREDIENT CHECK] Checking {ingredient_key}: current={current_amount}g, needed={amount_grams}g, threshold={critical_threshold}g", service="validation")
                            
                            if current_amount < amount_grams:
                                log("WARNING", f"[INGREDIENT CHECK] INSUFFICIENT {ingredient_key}: current={current_amount}g < needed={amount_grams}g", service="validation")
                                result["passed"] = False
                                result["details"][ingredient_key] = {
                                    "subtype": subtype,
                                    "current": current_amount,
                                    "needed": amount_grams,
                                    "critical_threshold": critical_threshold,
                                    "status": "insufficient"
                                }
                            else:
                                log("INFO", f"[INGREDIENT CHECK] SUFFICIENT {ingredient_key}: current={current_amount}g >= needed={amount_grams}g", service="validation")
                                result["details"][ingredient_key] = {
                                    "subtype": subtype,
                                    "current": current_amount,
                                    "needed": amount_grams,
                                    "critical_threshold": critical_threshold,
                                    "status": "available"
                                }
                        
                    elif ingredient_key == "cups":
                        # Cup subtypes are stored directly (e.g., 'cup_H9')
                        subtype = subtype_key
                        
                        # Check cups inventory
                        if subtype in current_inventory[ingredient_type]:
                            current_amount = current_inventory[ingredient_type][subtype]["current_amount"]
                            critical_threshold = current_inventory[ingredient_type][subtype]["critical_threshold"]
                            
                            log("INFO", f"[INGREDIENT CHECK] Checking {subtype}: current={current_amount}, needed={amount}, threshold={critical_threshold}", service="validation")
                            
                            if current_amount < amount:
                                log("WARNING", f"[INGREDIENT CHECK] INSUFFICIENT {subtype}: current={current_amount} < needed={amount}", service="validation")
                                result["passed"] = False
                                result["details"]["cups"] = {
                                    "subtype": subtype,
                                    "current": current_amount,
                                    "needed": amount,
                                    "critical_threshold": critical_threshold,
                                    "status": "insufficient"
                                }
                            else:
                                log("INFO", f"[INGREDIENT CHECK] SUFFICIENT {subtype}: current={current_amount} >= needed={amount}", service="validation")
                                result["details"]["cups"] = {
                                    "subtype": subtype,
                                    "current": current_amount,
                                    "needed": amount,
                                    "critical_threshold": critical_threshold,
                                    "status": "available"
                                }
                    
                    else:
                        # For milk, water, syrups - these use numeric IDs that need mapping
                        try:
                            if isinstance(subtype_key, int):
                                numeric_id = subtype_key
                            elif isinstance(subtype_key, str):
                                numeric_id = int(subtype_key)
                            else:
                                log("WARNING", f"Invalid subtype_key type for {ingredient_key}: {type(subtype_key)}", service="validation")
                                continue
                        except (ValueError, TypeError) as e:
                            log("WARNING", f"Failed to convert subtype_key to int for {ingredient_key}:{subtype_key}: {e}", service="validation")
                            continue
                        
                        # Determine the inventory category and subtype based on ingredient_key
                        inventory_category = None
                        inventory_subtype = None
                        check_amount = amount
                        
                        if ingredient_key == "milk":
                            # Map numeric ID to milk subtype
                            if numeric_id in MILK_ID_TO_SUBTYPE:
                                inventory_subtype = MILK_ID_TO_SUBTYPE[numeric_id]
                                if inventory_subtype is None:  # ID 5 is water, not milk
                                    log("INFO", f"Milk ID {numeric_id} is actually water, skipping validation", service="validation")
                                    continue
                                inventory_category = "milk"
                            else:
                                log("WARNING", f"Unknown milk ID: {numeric_id}", service="validation")
                                continue
                        
                        elif ingredient_key == "water":
                            # Water is not tracked in inventory
                            log("INFO", f"Water validation requested (ID: {numeric_id}) - water not tracked, assuming available", service="validation")
                            continue
                        
                        elif ingredient_key == "syrups":
                            # Map numeric ID to syrup subtype
                            if numeric_id in SYRUP_ID_TO_SUBTYPE:
                                inventory_subtype = SYRUP_ID_TO_SUBTYPE[numeric_id]
                                inventory_category = "syrups"
                            else:
                                log("WARNING", f"Unknown syrup ID: {numeric_id}", service="validation")
                                continue
                        
                        # If we have a valid mapping, check inventory
                        if inventory_category and inventory_subtype:
                            if inventory_subtype in current_inventory[inventory_category]:
                                current_amount = current_inventory[inventory_category][inventory_subtype]["current_amount"]
                                critical_threshold = current_inventory[inventory_category][inventory_subtype]["critical_threshold"]
                                
                                log("INFO", f"[INGREDIENT CHECK] Checking {ingredient_key} ({inventory_subtype}): current={current_amount}ml, needed={check_amount}ml, threshold={critical_threshold}ml", service="validation")
                                
                                if current_amount < check_amount:
                                    log("WARNING", f"[INGREDIENT CHECK] INSUFFICIENT {ingredient_key} ({inventory_subtype}): current={current_amount}ml < needed={check_amount}ml", service="validation")
                                    result["passed"] = False
                                    result["details"][ingredient_key] = {
                                        "id": numeric_id,
                                        "inventory_category": inventory_category,
                                        "subtype": inventory_subtype,
                                        "current": current_amount,
                                        "needed": check_amount,
                                        "critical_threshold": critical_threshold,
                                        "status": "insufficient"
                                    }
                                else:
                                    log("INFO", f"[INGREDIENT CHECK] SUFFICIENT {ingredient_key} ({inventory_subtype}): current={current_amount}ml >= needed={check_amount}ml", service="validation")
                                    result["details"][ingredient_key] = {
                                        "id": numeric_id,
                                        "inventory_category": inventory_category,
                                        "subtype": inventory_subtype,
                                        "current": current_amount,
                                        "needed": check_amount,
                                        "critical_threshold": critical_threshold,
                                        "status": "available"
                                    }
                            else:
                                log("WARNING", f"Subtype {inventory_subtype} not found in {inventory_category} inventory", service="validation")
                                continue
            
            # Add summary message
            if result["passed"]:
                result["details"]["message"] = "All ingredients available for this task"
                log("INFO", f"[INGREDIENT VALIDATION] PASSED - All ingredients available", service="validation")
            else:
                result["details"]["message"] = "Insufficient ingredients for this task"
                log("WARNING", f"[INGREDIENT VALIDATION] FAILED - Insufficient ingredients detected", service="validation")
            
            log("INFO", f"[INGREDIENT VALIDATION] Final result: passed={result['passed']}", service="validation")
            log("DEBUG", f"[INGREDIENT VALIDATION] Full result: {json.dumps(result, indent=2)}", service="validation")
            return result
            
        except Exception as e:
            import traceback
            log("ERROR", f"!!! EXCEPTION in ingredient validation: {e}", service="validation")
            log("ERROR", f"!!! Traceback: {traceback.format_exc()}", service="validation")
            return {
                "request_id": payload.get("request_id") if isinstance(payload, dict) else "unknown",
                "client_type": payload.get("client_type") if isinstance(payload, dict) else "unknown",
                "cup_id": payload.get("cup_id") if isinstance(payload, dict) else "unknown",
                "passed": False,
                "error": f"Ingredient validation failed: {str(e)}"
            }
    
    def process_ingredient_update_request(self, payload, specific_ingredient=None):
        """
        Process ingredient update (deduction) requests from routine service.
        
        Deducts ingredients sent in the routine format from inventory:
        {
            'request_id': 'routine-cup123-...',
            'client_type': 'routine',
            'cup_id': 'cup123',
            'milk': {1: 110.0},
            'syrups': {10: 5.0, 13: 8.0},
            'cups': {'cup_H9': 1.0},
            'espresso': {'espresso_shot_single': 1.0},
            'water': {5: 100.0},
            ...
        }
        
        Args:
            payload: Request payload with ingredients
            specific_ingredient: If provided, only deduct this specific ingredient type
                                (e.g., 'milk', 'water', 'syrups')
        
        Returns:
            Result dict with deduction details and warnings
        """
        try:
            log("INFO", f"=== INGREDIENT UPDATE START ===", service="validation")
            log("INFO", f"Specific ingredient filter: {specific_ingredient}", service="validation")
            log("INFO", f"Payload type: {type(payload)}", service="validation")
            log("INFO", f"Payload keys: {list(payload.keys()) if isinstance(payload, dict) else 'NOT A DICT'}", service="validation")
            
            result = {"passed": True, "details": {}}
            result["request_id"] = payload.get("request_id")
            result["client_type"] = payload.get("client_type")
            result["cup_id"] = payload.get("cup_id")
            
            # Define ingredient categories to check (skip non-ingredient keys)
            skip_keys = {'request_id', 'client_type', 'cup_id', 'position', 'temperature'}
            
            updated_ingredients = []

            def _capture_remaining(category: str, subtype_name: str):
                entry = self._inventory_client.inventory_cache.get(category, {}).get(subtype_name)
                if not entry:
                    return None, None
                remaining_amount = entry.get("current_amount")
                max_capacity = entry.get("max_capacity", 0)
                if max_capacity:
                    remaining_percentage = int((remaining_amount / max_capacity) * 100)
                else:
                    remaining_percentage = None
                return remaining_amount, remaining_percentage
            
            # Check each ingredient in the payload
            log("INFO", f"Iterating through payload keys: {list(payload.keys())}", service="validation")
            
            for ingredient_key, ingredient_data in payload.items():
                # Skip non-ingredient keys
                if ingredient_key in skip_keys:
                    log("INFO", f"Skipping metadata key: {ingredient_key}", service="validation")
                    continue
                
                # Validate that ingredient_data is a dict
                if not isinstance(ingredient_data, dict):
                    log("INFO", f"Skipping non-dict value for key '{ingredient_key}': {type(ingredient_data)}", service="validation")
                    continue
                
                # Skip if the dict is empty
                if not ingredient_data:
                    log("INFO", f"Skipping empty dict for key '{ingredient_key}'", service="validation")
                    continue
                
                # If specific_ingredient is set, only process that ingredient
                if specific_ingredient and ingredient_key != specific_ingredient:
                    log("INFO", f"Skipping {ingredient_key} (looking for {specific_ingredient})", service="validation")
                    continue
                
                log("INFO", f"Processing ingredient '{ingredient_key}' with data: {ingredient_data}", service="validation")
                
                # Map ingredient names to inventory categories
                ingredient_type = ingredient_key
                if ingredient_key == "espresso":
                    ingredient_type = "coffee_beans"
                elif ingredient_key == "cups":
                    ingredient_type = "cups"
                
                # Process each subtype/variant in this ingredient
                for subtype_key, amount in ingredient_data.items():
                    # Validate that amount is numeric
                    if not isinstance(amount, (int, float)):
                        log("WARNING", f"Skipping non-numeric amount for {ingredient_key}:{subtype_key}: {amount}", service="validation")
                        continue
                    # For espresso, convert to coffee beans
                    if ingredient_key == "espresso":
                        subtype = "regular"  # Coffee beans regular is used for espresso
                        shots_value = int(round(float(amount)))
                        grams_deducted = self._inventory_client.convert_shots_to_grams(abs(shots_value))

                        # Deduct from inventory (negative amount = deduction in shots)
                        success, warning = self._inventory_client.update_inventory(
                            ingredient_type="coffee_beans",
                            subtype=subtype,
                            amount=-shots_value
                        )

                        if success:
                            updated_ingredients.append(f"coffee_beans:{subtype}")
                            remaining_amount, remaining_percentage = _capture_remaining("coffee_beans", subtype)
                            result["details"][ingredient_key] = {
                                "subtype": subtype,
                                "shots": shots_value,
                                "deducted_grams": grams_deducted,
                                "remaining_amount": remaining_amount,
                                "remaining_percentage": remaining_percentage,
                                "status": warning,
                                "message": f"Deducted {shots_value} shot(s) ({grams_deducted}g) of coffee beans"
                            }
                        else:
                            result["passed"] = False
                            result["details"][ingredient_key] = {
                                "subtype": subtype,
                                "status": "failed",
                                "message": f"Failed to deduct {shots_value} shot(s) of coffee beans"
                            }
                    
                    elif ingredient_key == "cups":
                        # Cup subtypes are stored directly (e.g., 'cup_H9')
                        subtype = subtype_key
                        
                        # Deduct from inventory
                        success, warning = self._inventory_client.update_inventory(
                            ingredient_type="cups",
                            subtype=subtype,
                            amount=-amount
                        )
                        
                        if success:
                            updated_ingredients.append(f"cups:{subtype}")
                            remaining_amount, remaining_percentage = _capture_remaining("cups", subtype)
                            result["details"]["cups"] = {
                                "subtype": subtype,
                                "deducted_amount": amount,
                                "remaining_amount": remaining_amount,
                                "remaining_percentage": remaining_percentage,
                                "status": warning,
                                "message": f"Deducted {amount} cup(s) of {subtype}"
                            }
                        else:
                            result["passed"] = False
                            result["details"]["cups"] = {
                                "subtype": subtype,
                                "status": "failed",
                                "message": f"Failed to deduct {amount} cup(s) of {subtype}"
                            }
                    
                    else:
                        # For milk, water, syrups - these use numeric IDs that need mapping
                        try:
                            if isinstance(subtype_key, int):
                                numeric_id = subtype_key
                            elif isinstance(subtype_key, str):
                                numeric_id = int(subtype_key)
                            else:
                                log("WARNING", f"Invalid subtype_key type for {ingredient_key}: {type(subtype_key)}", service="validation")
                                continue
                        except (ValueError, TypeError) as e:
                            log("WARNING", f"Failed to convert subtype_key to int for {ingredient_key}:{subtype_key}: {e}", service="validation")
                            continue
                        
                        # Determine the inventory category and subtype based on ingredient_key
                        inventory_category = None
                        inventory_subtype = None
                        
                        if ingredient_key == "milk":
                            # Map numeric ID to milk subtype
                            log("INFO", f"Mapping milk ID {numeric_id} to subtype", service="validation")
                            if numeric_id in MILK_ID_TO_SUBTYPE:
                                inventory_subtype = MILK_ID_TO_SUBTYPE[numeric_id]
                                if inventory_subtype is None:  # ID 5 is water, not milk
                                    log("WARNING", f"Milk ID {numeric_id} is actually water, skipping", service="validation")
                                    continue
                                inventory_category = "milk"
                                log("INFO", f"Mapped to: {inventory_category}:{inventory_subtype}", service="validation")
                            else:
                                log("WARNING", f"Unknown milk ID: {numeric_id}", service="validation")
                                result["details"][f"milk_id_{numeric_id}"] = {
                                    "id": numeric_id,
                                    "status": "unknown_id",
                                    "message": f"Unknown milk ID: {numeric_id}"
                                }
                                continue
                        
                        elif ingredient_key == "water":
                            # Water may not be tracked in inventory (unlimited supply)
                            # Log the usage but don't try to deduct from inventory
                            log("INFO", f"Water deduction requested (ID: {numeric_id}, amount: {amount}) - water not tracked in inventory", service="validation")
                            result["details"]["water"] = {
                                "id": numeric_id,
                                "deducted_amount": amount,
                                "status": "not_tracked",
                                "message": f"Water usage logged ({amount} units) - not tracked in inventory"
                            }
                            continue  # Skip inventory deduction for water
                        
                        elif ingredient_key == "syrups":
                            # Map numeric ID to syrup subtype
                            if numeric_id in SYRUP_ID_TO_SUBTYPE:
                                inventory_subtype = SYRUP_ID_TO_SUBTYPE[numeric_id]
                                inventory_category = "syrups"
                            else:
                                log("WARNING", f"Unknown syrup ID: {numeric_id}", service="validation")
                                result["details"][f"syrup_id_{numeric_id}"] = {
                                    "id": numeric_id,
                                    "status": "unknown_id",
                                    "message": f"Unknown syrup ID: {numeric_id}"
                                }
                                continue
                        
                        # If we have a valid mapping, deduct from inventory
                        if inventory_category and inventory_subtype:
                            log("INFO", f"Deducting {ingredient_key} (ID: {numeric_id} -> {inventory_category}:{inventory_subtype}, amount: {amount})", service="validation")
                            
                            # Deduct from inventory
                            success, warning = self._inventory_client.update_inventory(
                                ingredient_type=inventory_category,
                                subtype=inventory_subtype,
                                amount=-amount
                            )
                            
                            if success:
                                updated_ingredients.append(f"{inventory_category}:{inventory_subtype}")
                                remaining_amount, remaining_percentage = _capture_remaining(inventory_category, inventory_subtype)
                                result["details"][ingredient_key] = {
                                    "id": numeric_id,
                                    "inventory_category": inventory_category,
                                    "subtype": inventory_subtype,
                                    "deducted_amount": amount,
                                    "remaining_amount": remaining_amount,
                                    "remaining_percentage": remaining_percentage,
                                    "status": warning,
                                    "message": f"Deducted {amount} units of {inventory_subtype}"
                                }
                            else:
                                result["passed"] = False
                                result["details"][ingredient_key] = {
                                    "id": numeric_id,
                                    "inventory_category": inventory_category,
                                    "subtype": inventory_subtype,
                                    "status": "failed",
                                    "message": f"Failed to deduct {amount} units of {inventory_subtype}"
                                }
            
            # Add summary
            if updated_ingredients:
                result["details"]["updated_ingredients"] = updated_ingredients
                result["details"]["message"] = f"Successfully deducted {len(updated_ingredients)} ingredient type(s)"
            else:
                result["details"]["message"] = "No ingredients were deducted (may require ID mapping)"
            
            log("INFO", f"Ingredient update result: {json.dumps(result, indent=2)}", service="validation")
            return result
            
        except Exception as e:
            import traceback
            log("ERROR", f"!!! EXCEPTION in ingredient update: {e}", service="validation")
            log("ERROR", f"!!! Traceback: {traceback.format_exc()}", service="validation")
            return {
                "request_id": payload.get("request_id") if isinstance(payload, dict) else "unknown",
                "client_type": payload.get("client_type") if isinstance(payload, dict) else "unknown",
                "cup_id": payload.get("cup_id") if isinstance(payload, dict) else "unknown",
                "passed": False,
                "error": f"Ingredient update failed: {str(e)}"
            }
    
    def process_update_limits_request(self, request: dict) -> dict:
        """
        Process requests to update inventory capacity limits.
        Updates both the inventory_rules.json file and the in-memory cache.
        
        Args:
            request: {
                "request_id": "...",
                "client_type": "api_bridge",
                "function_name": "update_limits",
                "payload": {
                    "updates": [
                        {"category": "milk", "subtype": "whole_fat_milk", "field": "max_capacity", "value": 20000},
                        {"category": "cups", "subtype": "cup_H7", "field": "warning_threshold", "value": 60},
                        ...
                    ]
                }
            }
        
        Returns:
            dict: Result with success status and details
        """
        try:
            request_id = request.get("request_id")
            updates = request.get("payload", {}).get("updates", [])
            
            log("INFO", f"Processing update limits request: {request_id} with {len(updates)} updates", service="validation")
            
            if not updates:
                return {
                    "request_id": request_id,
                    "passed": False,
                    "error": "No updates provided"
                }
            
            # Load current inventory rules
            import os
            current_dir = os.path.dirname(os.path.abspath(__file__))
            rules_file = os.path.join(current_dir, 'inventory_rules.json')
            
            with open(rules_file, 'r') as f:
                inventory_rules = json.load(f)
            
            updated_items = []
            errors = []
            
            # Process each update
            for update in updates:
                category = update.get("category")
                subtype = update.get("subtype")
                field = update.get("field")
                value = update.get("value")
                
                # Validate update
                if not all([category, subtype, field is not None, value is not None]):
                    errors.append(f"Invalid update format: {update}")
                    continue
                
                if field not in ['max_capacity', 'warning_threshold', 'critical_threshold', 'low_threshold']:
                    errors.append(f"Invalid field '{field}' for {category}:{subtype}")
                    continue
                
                # Check if category and subtype exist
                if category not in inventory_rules:
                    errors.append(f"Invalid category: {category}")
                    continue
                
                if 'subtypes' not in inventory_rules[category]:
                    errors.append(f"Category {category} has no subtypes")
                    continue
                
                if subtype not in inventory_rules[category]['subtypes']:
                    errors.append(f"Invalid subtype '{subtype}' for category {category}")
                    continue
                
                # Update the value in inventory_rules
                old_value = inventory_rules[category]['subtypes'][subtype].get(field, 0)
                inventory_rules[category]['subtypes'][subtype][field] = value
                
                # Update in-memory cache
                if category in self._inventory_client.inventory_cache:
                    if subtype in self._inventory_client.inventory_cache[category]:
                        self._inventory_client.inventory_cache[category][subtype][field] = value
                
                # Special handling for max_capacity: If new max_capacity is lower than current_amount,
                # reduce current_amount to match new max_capacity to prevent >100% display
                current_amount_adjusted = False
                if field == 'max_capacity':
                    current_amount = self._inventory_client.inventory_cache.get(category, {}).get(subtype, {}).get("current_amount", 0)
                    
                    if current_amount > value:
                        # Current amount exceeds new max capacity, reduce it
                        success = self._db_client.update_inventory(category, subtype, value)
                        
                        if success:
                            # Update cache with new current_amount
                            if category in self._inventory_client.inventory_cache:
                                if subtype in self._inventory_client.inventory_cache[category]:
                                    self._inventory_client.inventory_cache[category][subtype]["current_amount"] = value
                            
                            current_amount_adjusted = True
                            log("INFO", f"Adjusted current_amount for {category}:{subtype} from {current_amount} to {value} (new max_capacity)", service="validation")
                
                updated_items.append({
                    "category": category,
                    "subtype": subtype,
                    "field": field,
                    "old_value": old_value,
                    "new_value": value,
                    "current_amount_adjusted": current_amount_adjusted
                })
                
                log("INFO", f"Updated {category}:{subtype}.{field} from {old_value} to {value}", service="validation")
            
            # Save updated inventory rules back to file
            if updated_items:
                with open(rules_file, 'w') as f:
                    json.dump(inventory_rules, f, indent=4)
                log("INFO", f"Saved {len(updated_items)} updates to inventory_rules.json", service="validation")
            
            # Prepare response
            if errors and not updated_items:
                return {
                    "request_id": request_id,
                    "passed": False,
                    "error": "All updates failed",
                    "details": {
                        "errors": errors
                    }
                }
            
            return {
                "request_id": request_id,
                "passed": True,
                "success": True,
                "details": {
                    "updated_items": updated_items,
                    "updated_count": len(updated_items),
                    "errors": errors if errors else []
                }
            }
            
        except Exception as e:
            log("ERROR", f"Error processing update limits request: {e}", service="validation")
            import traceback
            traceback.print_exc()
            return {
                "request_id": request.get("request_id"),
                "passed": False,
                "error": f"Error updating inventory limits: {str(e)}"
            }