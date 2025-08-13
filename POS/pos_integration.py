#!/usr/bin/env python3
"""
POS_Integration.py
~~~~~~~~~~~~~~~~~~
Fully Dynamic Flask API for parsing BARNS-style POS payloads.
Returns flat array structure for ingredients with proper quantity/amount handling.

Author: BARNS Team
Created: 2025-01-16
"""

import json
import os
from copy import deepcopy
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from flask import Flask, request, jsonify

app = Flask(__name__)

# Global variables to store reference data
MENU_ITEMS: Dict[str, Dict[str, Any]] = {}
INGREDIENTS: Dict[str, str] = {}
INGREDIENT_DETAILS: Dict[str, Dict[str, Any]] = {}

# ------------------------------------------------------------------------------
# 1) Load reference data from JSON file
# ------------------------------------------------------------------------------

def load_reference_data(json_file_path: str = "reference_data.json") -> bool:
    """Load menu items, ingredients, and ingredient details from JSON file."""
    global MENU_ITEMS, INGREDIENTS, INGREDIENT_DETAILS
    
    try:
        current_dir = Path(__file__).parent
        json_path = current_dir / json_file_path
        
        if not json_path.exists():
            print(f"Reference data file not found: {json_path}")
            return False
            
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        MENU_ITEMS = data.get("menu_items", {})
        INGREDIENTS = data.get("ingredients", {})
        INGREDIENT_DETAILS = data.get("ingredient_details", {})
        
        print(f"Loaded {len(MENU_ITEMS)} menu items, {len(INGREDIENTS)} ingredients, "
              f"and {len(INGREDIENT_DETAILS)} ingredient details")
        return True
        
    except Exception as e:
        print(f"Error loading reference data: {e}")
        return False

# ------------------------------------------------------------------------------
# 2) Helper functions
# ------------------------------------------------------------------------------

def _lookup_ingredient_name(ingredient_id: str) -> str:
    """Return human-readable name for an ingredient ID."""
    return INGREDIENTS.get(ingredient_id, f"Unknown Ingredient {ingredient_id}")

def _get_ingredient_details(ingredient_id: str) -> Dict[str, Any]:
    """Get ingredient details from reference data."""
    return INGREDIENT_DETAILS.get(ingredient_id, {})

def _extract_kitchen_info(kitchen_notes: List[Dict[str, Any]]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """
    Extract kitchen information and return structured data.
    Returns: (kitchen_modifiers, manual_notes)
    """
    ice_level = "normal"
    temperature = "normal"
    foam = "normal"
    manual_notes = []
    
    for note in kitchen_notes:
        note_type = note.get("type", "").lower()
        detail = note.get("detail", "").lower()
        qty = note.get("qty", 0)
        
        # If qty is 0, it's a modifier for automation
        if qty == 0:
            if "ice" in note_type:
                ice_level = detail.replace(" ", "_")
            elif "temperature" in note_type or "temprature" in note_type:
                temperature = detail.replace(" ", "_")
            elif "foam" in note_type:
                foam = detail.replace(" ", "_")
        else:
            # If qty > 0, it's a manual note for barista
            manual_notes.append({
                "type": note.get("type"),
                "quantity": qty,
                "detail": note.get("detail")
            })
    
    kitchen_modifiers = {
        "ice_level": ice_level,
        "temperature": temperature, 
        "foam": foam
    }
    
    return kitchen_modifiers, manual_notes

def _apply_ingredient_modifications(
    base_ingredients: List[Dict[str, Any]],
    modifications: List[Dict[str, Any]],
    kitchen_modifiers: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """Apply modifications to base ingredients and return final ingredient list."""
    
    # Deep copy to avoid modifying original
    final_ingredients = deepcopy(base_ingredients)

    print(f"modifications: {json.dumps(modifications, indent=4)}")
    
    # Apply replacements (modifications where isAddon=False, isModified=True)
    for mod in modifications:
        if not mod.get("isAddon", False) and mod.get("isModified"):
            initial_id = mod.get("initialItemId")
            new_id = mod.get("itemId")
            qty = mod.get("qty", 1)
            print(f"the ingredient is {initial_id} and the new ingredient is {new_id}")
            
            # Find and replace the ingredient
            for i, ingredient in enumerate(final_ingredients):
                if ingredient.get("ingredient_id") == initial_id:
                    # Get new ingredient details
                    new_details = _get_ingredient_details(new_id)
                    print(f"the new details are {new_details}")
                    print(f"the ingredient is {ingredient}")
                    if new_details:
                        # Create replacement ingredient
                        replacement = {
                            "ingredient_id": new_id,
                            "ingredient_name": _lookup_ingredient_name(new_id),
                            "category": new_details["category"],
                            "type": new_details["type"],
                            "quantity": ingredient.get("quantity", 1) * qty,
                            "unit_amount": ingredient.get("unit_amount", 1),
                            "automated": new_details.get("automated", True),
                            "needs_validation": new_details.get("needs_validation", True),
                            "modified": True,
                            "original_ingredient": initial_id
                        }
                        
                        # Preserve sensitive flags
                        if new_details.get("temperature_sensitive"):
                            replacement["temperature_sensitive"] = True
                        if new_details.get("foam_sensitive"):
                            replacement["foam_sensitive"] = True
                        if new_details.get("ice_sensitive"):
                            replacement["ice_sensitive"] = True
                        if new_details.get("is_topping"):
                            replacement["is_topping"] = True
                            
                        final_ingredients[i] = replacement
                    break
    
    # Add addons (isAddon=True)
    for mod in modifications:
        if mod.get("isAddon", False):
            addon_id = mod.get("itemId")
            qty = mod.get("qty", 1)
            addon_details = _get_ingredient_details(addon_id)
            
            if addon_details:
                addon_ingredient = {
                    "ingredient_id": addon_id,
                    "ingredient_name": _lookup_ingredient_name(addon_id),
                    "category": addon_details["category"],
                    "type": addon_details["type"],
                    "quantity": qty,
                    "unit_amount": addon_details.get("default_amount", 1),
                    "automated": addon_details.get("automated", True),
                    "needs_validation": addon_details.get("needs_validation", True),
                    "is_addon": True
                }
                
                # Add sensitive flags if present
                if addon_details.get("temperature_sensitive"):
                    addon_ingredient["temperature_sensitive"] = True
                if addon_details.get("foam_sensitive"):
                    addon_ingredient["foam_sensitive"] = True
                if addon_details.get("is_topping"):
                    addon_ingredient["is_topping"] = True
                
                final_ingredients.append(addon_ingredient)
    
    return final_ingredients

def _create_ingredient_list(
    ingredients: List[Dict[str, Any]], 
    kitchen_modifiers: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """Create flat list of ingredients with kitchen modifications applied."""
    
    ingredient_list = []
    
    for ingredient in ingredients:
        category = ingredient["category"]
        
        # Create base ingredient object
        ingredient_obj = {
            "category": ingredient["category"],
            "type": ingredient["type"],
            "ingredient_id": ingredient["ingredient_id"],
            "ingredient_name": ingredient.get("ingredient_name", _lookup_ingredient_name(ingredient["ingredient_id"])),
            "quantity": ingredient.get("quantity", 1),
            "unit_amount": ingredient.get("unit_amount", 1),
            "total_amount": ingredient.get("quantity", 1) * ingredient.get("unit_amount", 1),
            "automated": ingredient.get("automated", True),
            "needs_validation": ingredient.get("needs_validation", True)
        }
        
        # Add modification flags if present
        if ingredient.get("modified"):
            ingredient_obj["modified"] = True
            ingredient_obj["original_ingredient"] = ingredient.get("original_ingredient")
        
        if ingredient.get("is_addon"):
            ingredient_obj["is_addon"] = True
            
        if ingredient.get("is_topping"):
            ingredient_obj["is_topping"] = True
        
        # Apply temperature to temperature-sensitive ingredients
        if ingredient.get("temperature_sensitive", False):
            ingredient_obj["temperature"] = kitchen_modifiers.get("temperature", "normal")
        
        # Apply foam to milk ingredients
        if category == "milk" and ingredient.get("foam_sensitive", False):
            ingredient_obj["foam"] = kitchen_modifiers.get("foam", "normal")
        
        # Apply ice level to ice ingredients
        if category == "ice" and ingredient.get("ice_sensitive", False):
            ice_level = kitchen_modifiers.get("ice_level", "normal")
            ingredient_obj["level"] = ice_level
            
            # Adjust quantity based on ice level
            base_qty = ingredient.get("quantity", 8)
            if ice_level == "extra" or ice_level == "extra_ice":
                ingredient_obj["quantity"] = int(base_qty * 1.5)
            elif ice_level == "light" or ice_level == "light_ice":
                ingredient_obj["quantity"] = int(base_qty * 0.5)
            elif ice_level == "no_ice":
                ingredient_obj["quantity"] = 0
            # Recalculate total amount
            ingredient_obj["total_amount"] = ingredient_obj["quantity"] * ingredient_obj["unit_amount"]
        
        ingredient_list.append(ingredient_obj)
    
    return ingredient_list

# ------------------------------------------------------------------------------
# 3) Main transformation
# ------------------------------------------------------------------------------

def parse_transaction(tx: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert raw JSON transaction into structured format with flat ingredient list.
    """
    parsed_items: List[Dict[str, Any]] = []
    print(f"tx: {tx}")
    for idx, item in enumerate(tx.get("items", []), start=1):
        item_id = item["item_id"]
        menu_entry = MENU_ITEMS.get(item_id, {})
        print(MENU_ITEMS.get(item_id, {}))
        
        if not menu_entry:
            # Skip unknown items
            continue
            
        base_ingredients = menu_entry.get("default_ingredients", [])
        
        # Extract kitchen information
        kitchen_notes = item.get("kitchen_notes", [])
        kitchen_modifiers, manual_notes = _extract_kitchen_info(kitchen_notes)
        
        # Apply ingredient modifications
        modifications = item.get("item_ingredients", [])
        final_ingredients = _apply_ingredient_modifications(
            base_ingredients, modifications, kitchen_modifiers
        )
        
        # Create flat ingredient list with modifiers applied
        ingredients_list = _create_ingredient_list(final_ingredients, kitchen_modifiers)
        
        # Filter validation requirements (excluding items that don't need validation)
        validation_required = []
        for ing in ingredients_list:
            if ing.get("needs_validation", False):
                # Map espresso to coffee_beans for validation
                val_category = ing["category"]
                amount = ing["total_amount"]
                
                validation_required.append({
                    "category": val_category,
                    "type": ing["type"],
                    "amount": amount
                })
        
        # Build parsed item
        parsed_item = {
            "line": idx,
            "recipe_id": item_id,
            "recipe_name": menu_entry.get("name", f"Unknown Item {item_id}"),
            "recipe": menu_entry.get("recipe", "Unknown"),
            "category": menu_entry.get("category", "unknown"),
            "size": menu_entry.get("size", "unknown"),
            "ordered_qty": item.get("quantity", 1),
            "automation": menu_entry.get("automation", False),
            "manual_notes": manual_notes,  # Notes for barista to handle manually
            "ingredients": ingredients_list,
            "validation_required": validation_required
        }
        
        parsed_items.append(parsed_item)

    return {
        "transaction_id": tx["transaction_id"],
        "date": tx["date"],
        "time": tx["time"],
        "store_number": tx["store_number"],
        "pos_reg_id": tx["pos_reg_id"],
        "customer_id": tx.get("customer_id"),
        "items": parsed_items,
    }

# ------------------------------------------------------------------------------
# 4) API Endpoints
# ------------------------------------------------------------------------------

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    return jsonify({
        "status": "healthy",
        "service": "Dynamic POS Integration API",
        "menu_items_loaded": len(MENU_ITEMS),
        "ingredients_loaded": len(INGREDIENTS),
        "ingredient_details_loaded": len(INGREDIENT_DETAILS)
    })

@app.route('/process-order', methods=['POST'])
def process_order():
    """
    Process POS order and return parsed transaction with flat ingredient list.
    """
    try:
        if not request.is_json:
            return jsonify({"error": "Content-Type must be application/json"}), 400
        
        order_data = request.get_json()
        
        # Validate required fields
        required_fields = ["transaction_id", "date", "time", "store_number", "pos_reg_id", "items"]
        for field in required_fields:
            if field not in order_data:
                return jsonify({"error": f"Missing required field: {field}"}), 400
        
        if not isinstance(order_data["items"], list) or len(order_data["items"]) == 0:
            return jsonify({"error": "Items must be a non-empty list"}), 400
        
        # Process the order
        parsed_order = parse_transaction(order_data)
        
        return jsonify({
            "success": True,
            "parsed_order": parsed_order
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error processing order: {str(e)}"
        }), 500

@app.route('/filter-for-validation', methods=['POST'])
def filter_for_validation():
    """Extract validation requirements from a parsed order."""
    try:
        if not request.is_json:
            return jsonify({"error": "Content-Type must be application/json"}), 400
        
        order_data = request.get_json()
        
        if "parsed_order" not in order_data:
            return jsonify({"error": "Missing parsed_order in request"}), 400
        
        parsed_order = order_data["parsed_order"]
        validation_items = []
        
        for item in parsed_order.get("items", []):
            if item.get("validation_required"):
                validation_items.append({
                    "drink_name": item["recipe_name"],
                    "line": item["line"],
                    "ingredients": item["validation_required"]
                })
        
        return jsonify({
            "success": True,
            "validation_items": validation_items
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Error filtering for validation: {str(e)}"
        }), 500

# ------------------------------------------------------------------------------
# 5) Application initialization
# ------------------------------------------------------------------------------

def initialize_app():
    """Initialize the application by loading reference data."""
    success = load_reference_data()
    if not success:
        print("Warning: Could not load reference data. API will run with empty data.")
    return success

if __name__ == "__main__":
    # Initialize the app
    initialize_app()
    
    # Run the Flask application
    port = int(os.environ.get("PORT", 5000))
    debug = os.environ.get("DEBUG", "False").lower() == "true"
    
    print(f"Starting Dynamic POS Integration API on port {port}")
    app.run(host="0.0.0.0", port=port, debug=debug)