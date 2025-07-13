#!/usr/bin/env python3
"""
POS_Integration.py
~~~~~~~~~~~~~~~~~~
Flask API for parsing BARNS-style POS payloads and resolving all numeric IDs 
to recipe/ingredient names using reference data from JSON file.

Author: BARNS Team
Created: 2025-01-16
"""

import json
import os
from copy import deepcopy
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Any, Optional
from flask import Flask, request, jsonify

app = Flask(__name__)

# Global variables to store reference data
MENU_ITEMS: Dict[str, Dict[str, Any]] = {}
INGREDIENTS: Dict[str, str] = {}

# ------------------------------------------------------------------------------
# 1) Load reference data from JSON file
# ------------------------------------------------------------------------------

def load_reference_data(json_file_path: str = "reference_data.json") -> bool:
    """Load menu items and ingredients from JSON file."""
    global MENU_ITEMS, INGREDIENTS
    
    try:
        # Get the directory of the current script
        current_dir = Path(__file__).parent
        json_path = current_dir / json_file_path
        
        if not json_path.exists():
            print(f"Reference data file not found: {json_path}")
            return False
            
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        MENU_ITEMS = data.get("menu_items", {})
        INGREDIENTS = data.get("ingredients", {})
        
        print(f"Loaded {len(MENU_ITEMS)} menu items and {len(INGREDIENTS)} ingredients")
        return True
        
    except Exception as e:
        print(f"Error loading reference data: {e}")
        return False

# ------------------------------------------------------------------------------
# 2) Helper functions
# ------------------------------------------------------------------------------

def _lookup_name(db: Dict[str, Any], _id: str, kind: str = "item") -> str:
    """Return human-readable name for an ID or a placeholder if unknown."""
    if _id in db:
        if isinstance(db[_id], dict):
            return db[_id].get("name", f"<unknown {kind}>")
        return db[_id]
    return f"<unmapped {kind} {_id}>"

def _apply_replacements(
    base: List[str],
    mods: List[Dict[str, Any]],
) -> List[str]:
    """
    Start with a list of default ingredient IDs (`base`) and apply the
    replacements contained in `mods` where isAddon == False and isModified == True.
    """
    updated = deepcopy(base)
    for m in mods:
        if (not m.get("isAddon")) and m.get("isModified") and m.get("initialItemId"):
            try:
                idx = updated.index(m["initialItemId"])
                updated[idx] = m["itemId"]
            except ValueError:
                # initialItemId not in list – just append replacement
                updated.append(m["itemId"])
    return updated

def _tally_quantities(ids: List[str]) -> Dict[str, int]:
    """Convert list of ingredient IDs to a {id: qty} tally."""
    tally = defaultdict(int)
    for _id in ids:
        tally[_id] += 1
    return dict(tally)

# ------------------------------------------------------------------------------
# 3) Main transformation
# ------------------------------------------------------------------------------

def parse_transaction(tx: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert raw JSON transaction into a structure with resolved names.
    """
    parsed_items: List[Dict[str, Any]] = []

    for idx, item in enumerate(tx.get("items", []), start=1):
        item_id = item["item_id"]
        menu_entry = MENU_ITEMS.get(item_id, {})
        base_ing = menu_entry.get("default_ingredients", [])

        # 1) Apply milk replacements etc.
        mods = item.get("item_ingredients", [])
        ingredients_after_replacements = _apply_replacements(base_ing, mods)

        # 2) Add addons (isAddon == True)
        for m in mods:
            if m.get("isAddon"):
                ingredients_after_replacements.extend([m["itemId"]] * m["qty"])

        # 3) Count final quantities
        ingredient_tally = _tally_quantities(ingredients_after_replacements)

        # 4) Stitch human names
        ingredient_list: List[Dict[str, Any]] = [
            {
                "ingredient_id": ing_id,
                "ingredient_name": _lookup_name(INGREDIENTS, ing_id, "ingredient"),
                "qty": qty,
            }
            for ing_id, qty in ingredient_tally.items()
        ]

        # 5) Attach kitchen notes (simple passthrough)
        kitchen_notes = item.get("kitchen_notes", [])

        # 6) Get automation status from menu entry
        automation = menu_entry.get("automation", False)

        parsed_items.append(
            {
                "line": idx,
                "recipe_id": item_id,
                "recipe_name": _lookup_name(MENU_ITEMS, item_id, "recipe"),
                "category": menu_entry.get("category", "unknown"),
                "size": menu_entry.get("size", "unknown"),
                "ordered_qty": item["quantity"],
                "ingredients": ingredient_list,
                "kitchen_notes": kitchen_notes,
                "automation": automation,
            },
        )

    # Top‑level structure mirrors the incoming payload but is enriched
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
        "service": "POS Integration API",
        "menu_items_loaded": len(MENU_ITEMS),
        "ingredients_loaded": len(INGREDIENTS)
    })

@app.route('/menu', methods=['GET'])
def get_menu():
    """Get all available menu items."""
    return jsonify({
        "menu_items": MENU_ITEMS,
        "total_items": len(MENU_ITEMS)
    })

@app.route('/ingredients', methods=['GET'])
def get_ingredients():
    """Get all available ingredients."""
    return jsonify({
        "ingredients": INGREDIENTS,
        "total_ingredients": len(INGREDIENTS)
    })

@app.route('/process-order', methods=['POST'])
def process_order():
    """
    Process POS order and return parsed transaction with resolved names.
    
    Expected JSON payload format:
    {
        "transaction_id": "string",
        "date": "YYYY-MM-DD",
        "time": "HH:MM:SS",
        "store_number": "string",
        "pos_reg_id": "string",
        "customer_id": "string (optional)",
        "items": [
            {
                "item_id": "string",
                "quantity": int,
                "item_ingredients": [
                    {
                        "itemId": "string",
                        "qty": int,
                        "isAddon": boolean,
                        "isModified": boolean,
                        "initialItemId": "string (optional)"
                    }
                ],
                "kitchen_notes": [
                    {
                        "type": "string",
                        "qty": int,
                        "detail": "string"
                    }
                ]
            }
        ]
    }
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
        
        # Validate each item
        for i, item in enumerate(order_data["items"]):
            if "item_id" not in item:
                return jsonify({"error": f"Item {i+1}: Missing item_id"}), 400
            if "quantity" not in item:
                return jsonify({"error": f"Item {i+1}: Missing quantity"}), 400
        
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

@app.route('/validate-items', methods=['POST'])
def validate_items():
    """Validate if provided item IDs exist in the menu."""
    try:
        if not request.is_json:
            return jsonify({"error": "Content-Type must be application/json"}), 400
        
        data = request.get_json()
        item_ids = data.get("item_ids", [])
        
        if not isinstance(item_ids, list):
            return jsonify({"error": "item_ids must be a list"}), 400
        
        validation_results = []
        for item_id in item_ids:
            exists = item_id in MENU_ITEMS
            result = {
                "item_id": item_id,
                "exists": exists,
                "name": MENU_ITEMS[item_id]["name"] if exists else None,
                "category": MENU_ITEMS[item_id]["category"] if exists else None,
                "size": MENU_ITEMS[item_id]["size"] if exists else None,
                "automation": MENU_ITEMS[item_id]["automation"] if exists else None
            }
            validation_results.append(result)
        
        return jsonify({
            "validation_results": validation_results,
            "total_checked": len(item_ids),
            "valid_items": sum(1 for r in validation_results if r["exists"])
        })
        
    except Exception as e:
        return jsonify({
            "error": f"Error validating items: {str(e)}"
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
    
    print(f"Starting POS Integration API on port {port}")
    app.run(host="0.0.0.0", port=port, debug=debug)