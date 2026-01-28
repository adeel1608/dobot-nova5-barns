"""
Core POS parsing module (no web framework dependencies).
Exposes dataclasses, reference-data loading, and a single entry `parse_transaction`.
"""

import json
import sqlite3
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# Global reference data containers (populated via load_reference_data_from_db)
MENU_ITEMS: Dict[str, Dict[str, Any]] = {}
INGREDIENTS: Dict[str, str] = {}
INGREDIENT_DETAILS: Dict[str, Dict[str, Any]] = {}

# Hardcoded mappings for milk and syrups categories
MILK_MAPPINGS = {
    "whole_fat": 20,
    "whole": 20,  # Same as whole_fat
    "almond": 18,
    "oat": 17,
 # Water uses milk pump 5
    "lactose_free": 15,
    "low_fat": 19,
}

SYRUP_MAPPINGS = {
    "normal_water": 1, 
    "hazelnut": 14,
    "vanilla": 7,
    "peach_iced_tea": 9,
    "passion_fruit_puree": 11,
    "ice_tea": 13,
    "caramel_syrup": 23,
}
SAUCE_MAPPINGS = {
    "white_chocolate": 12,
    "caramel": 16,
    "condense_milk": 8,
}

# ------------------------------------------------------------------------------
# Dataclasses for parsed order representation
# ------------------------------------------------------------------------------


@dataclass
class ParsedIngredient:
    category: str
    type: str
    ingredient_id: Any  # Can be str or int (mapped for milk/syrups)
    ingredient_name: str
    quantity: int
    unit_amount: Any
    total_amount: Any
    automated: Any
    needs_validation: Any
    modified: Optional[bool] = None
    original_ingredient: Optional[str] = None
    is_addon: Optional[bool] = None
    is_topping: Optional[bool] = None
    temperature: Optional[str] = None
    foam: Optional[str] = None
    level: Optional[str] = None
    cup_position: Optional[Any] = None


@dataclass
class ParsedItem:
    line: int
    recipe_id: str
    recipe_name: str
    recipe: str
    category: str
    size: str
    ordered_qty: int
    automation: Any
    manual_notes: List[Dict[str, Any]]
    ingredients: List[ParsedIngredient]
    validation_required: List[Dict[str, Any]]


@dataclass
class ParsedOrder:
    transaction_id: str
    date: str
    time: str
    store_number: Any
    pos_reg_id: Any
    customer_id: Optional[Any]
    items: List[ParsedItem]


# ------------------------------------------------------------------------------
# Reference data loading
# ------------------------------------------------------------------------------


def load_reference_data_from_db(db_file_path: str = "pos_reference.db") -> bool:
    """Load reference data from SQLite DB built by DB_parser into in-memory dicts."""
    global MENU_ITEMS, INGREDIENTS, INGREDIENT_DETAILS

    try:
        current_dir = Path(__file__).parent
        db_path = current_dir / db_file_path
        if not db_path.exists():
            print(f"POS reference DB not found: {db_path}")
            return False

        connection = sqlite3.connect(db_path)
        try:
            cursor = connection.cursor()

            # Load ingredients map
            INGREDIENTS = {}
            for row in cursor.execute("SELECT id, name FROM ingredients"):
                INGREDIENTS[row[0]] = row[1]

            # Load ingredient details
            INGREDIENT_DETAILS = {}
            for row in cursor.execute(
                """
                SELECT ingredient_id, category, type, base_units, automated,
                       temperature_sensitive, needs_validation, foam_sensitive,
                       ice_sensitive, is_topping, default_amount
                FROM ingredient_details
                """
            ):
                (
                    ingredient_id,
                    category,
                    type_,
                    base_units,
                    automated,
                    temperature_sensitive,
                    needs_validation,
                    foam_sensitive,
                    ice_sensitive,
                    is_topping,
                    default_amount,
                ) = row

                INGREDIENT_DETAILS[ingredient_id] = {
                    "category": category,
                    "type": type_,
                    "base_units": base_units,
                    "automated": automated,
                    "temperature_sensitive": bool(temperature_sensitive or 0),
                    "needs_validation": bool(needs_validation or 0),
                    "foam_sensitive": bool(foam_sensitive or 0),
                    "ice_sensitive": bool(ice_sensitive or 0),
                    "is_topping": bool(is_topping or 0),
                    "default_amount": default_amount,
                }

            # Load menu items with default ingredients
            MENU_ITEMS = {}
            for row in cursor.execute(
                "SELECT id, name, category, size, automation, recipe FROM menu_items"
            ):
                item_id, name, category, size, automation, recipe = row
                MENU_ITEMS[item_id] = {
                    "name": name,
                    "category": category,
                    "size": size,
                    "automation": automation,
                    "recipe": recipe,
                    "default_ingredients": [],
                }

            # Map defaults - check if cup_position column exists
            cursor.execute("PRAGMA table_info(menu_item_default_ingredients)")
            columns = [col[1] for col in cursor.fetchall()]
            has_cup_position = 'cup_position' in columns
            
            if has_cup_position:
                query = """
                    SELECT menu_item_id, ingredient_id, category, type, unit_amount, automated,
                           temperature_sensitive, needs_validation, quantity, base_units,
                           foam_sensitive, is_topping, cup_position
                    FROM menu_item_default_ingredients
                """
            else:
                query = """
                    SELECT menu_item_id, ingredient_id, category, type, unit_amount, automated,
                           temperature_sensitive, needs_validation, quantity, base_units,
                           foam_sensitive, is_topping
                    FROM menu_item_default_ingredients
                """
            
            for row in cursor.execute(query):
                if has_cup_position:
                    (
                        menu_item_id,
                        ingredient_id,
                        category,
                        type_,
                        unit_amount,
                        automated,
                        temperature_sensitive,
                        needs_validation,
                        quantity,
                        base_units,
                        foam_sensitive,
                        is_topping,
                        cup_position,
                    ) = row
                else:
                    (
                        menu_item_id,
                        ingredient_id,
                        category,
                        type_,
                        unit_amount,
                        automated,
                        temperature_sensitive,
                        needs_validation,
                        quantity,
                        base_units,
                        foam_sensitive,
                        is_topping,
                    ) = row
                    cup_position = None

                if menu_item_id in MENU_ITEMS:
                    ingredient_data = {
                        "ingredient_id": ingredient_id,
                        "category": category,
                        "type": type_,
                        "unit_amount": unit_amount,
                        "automated": automated,
                        "temperature_sensitive": bool(temperature_sensitive or 0),
                        "needs_validation": bool(needs_validation or 0),
                        "quantity": quantity,
                        "base_units": base_units,
                        "foam_sensitive": bool(foam_sensitive or 0),
                        "is_topping": bool(is_topping or 0),
                    }
                    
                    # Add cup_position if present (from dedicated column)
                    # Or if category is 'position', use unit_amount as the position value
                    if cup_position is not None:
                        ingredient_data["cup_position"] = cup_position
                    elif category == "position" and ingredient_id == "cup_position":
                        # Fallback: use unit_amount as position value for position category
                        ingredient_data["cup_position"] = unit_amount
                        
                    MENU_ITEMS[menu_item_id]["default_ingredients"].append(ingredient_data)

        finally:
            connection.close()

        return True
    except Exception as e:
        print(f"Error loading reference data from DB: {e}")
        return False


# ------------------------------------------------------------------------------
# Helpers and transformation
# ------------------------------------------------------------------------------


def _lookup_ingredient_name(ingredient_id: str) -> str:
    """Return human-readable name for an ingredient ID."""
    return INGREDIENTS.get(ingredient_id, f"Unknown Ingredient {ingredient_id}")


def _get_ingredient_details(ingredient_id: str) -> Dict[str, Any]:
    """Get ingredient details from reference data."""
    return INGREDIENT_DETAILS.get(ingredient_id, {})


def _map_ingredient_id(ingredient_id: str, category: str, ingredient_type: str) -> Any:
    """
    Map ingredient ID to numeric value for milk, water, syrups, and sauce categories.
    For other categories, return the original ingredient_id.
    """
    if category == "milk":
        # Try to map using type or id
        mapped_id = MILK_MAPPINGS.get(ingredient_type) or MILK_MAPPINGS.get(ingredient_id)
        return mapped_id if mapped_id is not None else ingredient_id
    elif category == "water":
        # Map water to pump 5 (using milk dispenser hardware)
        mapped_id = MILK_MAPPINGS.get(ingredient_type) or MILK_MAPPINGS.get(ingredient_id) or 5
        return mapped_id if mapped_id is not None else ingredient_id
    elif category == "syrups":
        # Try to map using type or id
        mapped_id = SYRUP_MAPPINGS.get(ingredient_type) or SYRUP_MAPPINGS.get(ingredient_id)
        return mapped_id if mapped_id is not None else ingredient_id
    elif category == "sauce":
        # Try to map using type or id
        mapped_id = SAUCE_MAPPINGS.get(ingredient_type) or SAUCE_MAPPINGS.get(ingredient_id)
        return mapped_id if mapped_id is not None else ingredient_id
    else:
        return ingredient_id


def _extract_kitchen_info(
    kitchen_notes: List[Dict[str, Any]],
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
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
            manual_notes.append(
                {"type": note.get("type"), "quantity": qty, "detail": note.get("detail")}
            )

    kitchen_modifiers = {"ice_level": ice_level, "temperature": temperature, "foam": foam}

    return kitchen_modifiers, manual_notes


def _apply_ingredient_modifications(
    base_ingredients: List[Dict[str, Any]],
    modifications: List[Dict[str, Any]],
    kitchen_modifiers: Dict[str, Any],
) -> List[Dict[str, Any]]:
    """Apply modifications to base ingredients and return final ingredient list."""

    # Deep copy to avoid modifying original
    final_ingredients = deepcopy(base_ingredients)


    # Apply replacements (modifications where isAddon=False, isModified=True)
    for mod in modifications:
        if not mod.get("isAddon", False) and mod.get("isModified"):
            initial_id = mod.get("initialItemId")
            new_id = mod.get("itemId")
            qty = mod.get("qty", 1)

            # Find and replace the ingredient
            for i, ingredient in enumerate(final_ingredients):
                if ingredient.get("ingredient_id") == initial_id:
                    # Get new ingredient details
                    new_details = _get_ingredient_details(new_id)
                    if new_details:
                        # Create replacement ingredient
                        replacement = {
                            "ingredient_id": new_id,
                            "ingredient_name": _lookup_ingredient_name(new_id),
                            "category": new_details["category"],
                            "type": new_details["type"],
                            "quantity": qty,
                            "unit_amount": new_details.get("default_amount", 1),
                            "automated": new_details.get("automated", True),
                            "needs_validation": new_details.get("needs_validation", True),
                            "modified": True,
                            "original_ingredient": initial_id,
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
                        
                        # Preserve cup_position from original if present
                        if ingredient.get("cup_position") is not None:
                            replacement["cup_position"] = ingredient.get("cup_position")

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
                    "is_addon": True,
                }

                # Add sensitive flags if present
                if addon_details.get("temperature_sensitive"):
                    addon_ingredient["temperature_sensitive"] = True
                if addon_details.get("foam_sensitive"):
                    addon_ingredient["foam_sensitive"] = True
                if addon_details.get("is_topping"):
                    addon_ingredient["is_topping"] = True
                
                # Add cup_position if this is a position category addon
                if addon_details["category"] == "position":
                    addon_ingredient["cup_position"] = addon_details.get("default_amount", 1)

                final_ingredients.append(addon_ingredient)

    return final_ingredients


def _create_ingredient_list(
    ingredients: List[Dict[str, Any]], kitchen_modifiers: Dict[str, Any]
) -> List[ParsedIngredient]:
    """Create flat list of ingredients with kitchen modifications applied."""

    ingredient_list: List[ParsedIngredient] = []

    for ingredient in ingredients:
        category = ingredient["category"]
        ingredient_type = ingredient.get("type", "")
        original_ingredient_id = ingredient["ingredient_id"]
        
        # Apply hardcoded mapping for milk and syrups categories
        mapped_ingredient_id = _map_ingredient_id(
            original_ingredient_id, category, ingredient_type
        )

        # For position category, use unit_amount as the cup_position value
        cup_pos_value = ingredient.get("cup_position")
        if category == "position" and cup_pos_value is None:
            cup_pos_value = ingredient.get("unit_amount", 1)
        
        # Create base ingredient object
        ingredient_obj = ParsedIngredient(
            category=ingredient["category"],
            type=ingredient["type"],
            ingredient_id=mapped_ingredient_id,
            ingredient_name=ingredient.get(
                "ingredient_name", _lookup_ingredient_name(original_ingredient_id)
            ),
            quantity=ingredient.get("quantity", 1),
            unit_amount=ingredient.get("unit_amount", 1),
            total_amount=ingredient.get("quantity", 1) * ingredient.get("unit_amount", 1),
            automated=ingredient.get("automated", True),
            needs_validation=ingredient.get("needs_validation", True),
            cup_position=cup_pos_value
        )

        # Add modification flags if present
        if ingredient.get("modified"):
            ingredient_obj.modified = True
            ingredient_obj.original_ingredient = ingredient.get("original_ingredient")

        if ingredient.get("is_addon"):
            ingredient_obj.is_addon = True

        if ingredient.get("is_topping"):
            ingredient_obj.is_topping = True

        # Apply temperature to temperature-sensitive ingredients
        if ingredient.get("temperature_sensitive", False):
            ingredient_obj.temperature = kitchen_modifiers.get("temperature", "normal")

        # Apply foam to milk ingredients
        if category == "milk" and ingredient.get("foam_sensitive", False):
            ingredient_obj.foam = kitchen_modifiers.get("foam", "normal")

        # Apply ice level to ice ingredients
        if category == "ice" and ingredient.get("ice_sensitive", False):
            ice_level = kitchen_modifiers.get("ice_level", "normal")
            ingredient_obj.level = ice_level

            # Adjust quantity based on ice level
            base_qty = ingredient.get("quantity", 8)
            if ice_level == "extra" or ice_level == "extra_ice":
                ingredient_obj.quantity = int(base_qty * 1.5)
            elif ice_level == "light" or ice_level == "light_ice":
                ingredient_obj.quantity = int(base_qty * 0.5)
            elif ice_level == "no_ice":
                ingredient_obj.quantity = 0
            # Recalculate total amount
            ingredient_obj.total_amount = (
                ingredient_obj.quantity * ingredient_obj.unit_amount
            )

        ingredient_list.append(ingredient_obj)

    return ingredient_list


def parse_transaction(tx: Dict[str, Any]) -> ParsedOrder:
    """
    Convert raw JSON transaction into structured format with flat ingredient list.
    """
    parsed_items: List[ParsedItem] = []
    for idx, item in enumerate(tx.get("items", []), start=1):
        item_id = item["item_id"]
        menu_entry = MENU_ITEMS.get(item_id, {})
        # print(MENU_ITEMS.get(item_id, {}))

        if not menu_entry:
            # Include unknown items with manual automation instead of skipping
            menu_entry = {
                "name": item.get("name", item_id),
                "category": "unknown",
                "size": item.get("size", "unknown"),
                "automation": "manual",
                "recipe": item.get("name", item_id),
                "default_ingredients": [],
            }

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
            if bool(getattr(ing, "needs_validation", False)):
                val_category = getattr(ing, "category", "unknown")
                amount = getattr(ing, "total_amount", 0)

                validation_required.append(
                    {
                        "category": val_category,
                        "type": getattr(ing, "type", "unknown"),
                        "amount": amount,
                    }
                )

        # Build parsed item
        parsed_item = ParsedItem(
            line=idx,
            recipe_id=menu_entry.get("name", f"Unknown Item {item_id}"),
            recipe_name=menu_entry.get("name", f"Unknown Item {item_id}"),
            recipe=menu_entry.get("recipe", "Unknown"),
            category=menu_entry.get("category", "unknown"),
            size=menu_entry.get("size", "unknown"),
            ordered_qty=item.get("quantity", 1),
            automation=menu_entry.get("automation", False),
            manual_notes=manual_notes,
            ingredients=ingredients_list,
            validation_required=validation_required,
        )

        parsed_items.append(parsed_item)

    return {
        "transaction_id":tx["transaction_id"],
        "date":tx["date"],
        "time":tx["time"],
        "store_number":tx["store_number"],
        "pos_reg_id":tx["pos_reg_id"],
        "customer_id":tx.get("customer_id"),
        "items":parsed_items}


