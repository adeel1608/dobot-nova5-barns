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
    "lactose_free": 15,
    "low_fat": 19,
}

SYRUP_MAPPINGS = {
    "water": 1, 
    "hazelnut": 14,
    "vanilla": 7,
    "peach_iced_tea": 9,
    "passion_fruit_puree": 11,
    "ice_tea": 13,
    "caramel_syrup": 23,
}
SAUCE_MAPPINGS = {
    "white_chocolate": 12,
    "caramel_sauce": 16,
    "condense_milk": 8,
}

# Cup capacity and ingredient density constants
CUP_VOLUMES = {
    "H7": 207,   # 7oz hot cup (ml)
    "H9": 266,   # 9oz hot cup (ml)
    "H12": 355,  # 12oz hot cup (ml)
    "C7": 207,   # 7oz cold cup (ml)
    "C9": 266,   # 9oz cold cup (ml)
    "C12": 355,  # 12oz cold cup (ml)
    "C16": 473,  # 16oz cold cup (ml)
}

# Ingredient densities (g/ml) for volume conversion
INGREDIENT_DENSITIES = {
    "milk": 1.03,
    "syrups": 1.32,
    "sauce": 1.35,
    "espresso": 1.02,
    "toppings": 1.20,
    "extras": 1.15,
}

# Temperature foam percentages (% of milk replaced by foam)
TEMPERATURE_FOAM_PERCENTAGES = {
    "kids": 0,
    "standard": 10,
    "extra_hot": 15,
    "regular": 10,  # Alias for standard
    "normal": 10,   # Default
}

# Espresso shot weights (in grams)
ESPRESSO_SHOT_WEIGHTS = {
    "single_shot": 18,
    "double_shot": 36,
    "tripple_shot": 54,
    "single": 18,
    "double": 36,
    "triple": 54,
    "tripple": 54,  # Typo alias
}

# Maximum milk substitution percentage (after free space is filled)
MAX_MILK_SUBSTITUTION_PERCENT = 30

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
    adjusted_amount: Optional[float] = None  # Final amount after capacity adjustments
    milk_reduction_percent: Optional[float] = None  # Percentage reduced for capacity


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
        # Map water to pump 1 (using milk dispenser hardware)
        mapped_id = SYRUP_MAPPINGS.get(ingredient_type) or SYRUP_MAPPINGS.get(ingredient_id) or SYRUP_MAPPINGS.get("water", 1)
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
    ice_amount_grams = None  # Actual ice amount in grams if specified
    temperature = "normal"
    foam = "normal"
    manual_notes = []

    for note in kitchen_notes:
        note_type = note.get("type", "").lower()
        detail = note.get("detail", "").lower() if note.get("detail") else ""
        qty = note.get("qty", 0)

        # Handle ice specially - it can have qty > 0 (grams) AND be an automation modifier
        if "ice" in note_type:
            ice_level = detail.replace(" ", "_") if detail else ice_level
            if qty > 0:
                ice_amount_grams = qty  # Store actual grams for ice adjustment
            continue  # Don't add ice to manual notes
        
        # Temperature and foam with qty=0 are automation modifiers
        if qty == 0:
            if "temperature" in note_type or "temprature" in note_type:
                temperature = detail.replace(" ", "_")
            elif "foam" in note_type:
                foam = detail.replace(" ", "_")
        else:
            # If qty > 0 and not ice, it's a manual note for barista
            manual_notes.append(
                {"type": note.get("type"), "quantity": qty, "detail": note.get("detail")}
            )

    kitchen_modifiers = {
        "ice_level": ice_level, 
        "ice_amount_grams": ice_amount_grams,
        "temperature": temperature, 
        "foam": foam
    }

    return kitchen_modifiers, manual_notes


def _apply_ingredient_modifications(
    base_ingredients: List[Dict[str, Any]],
    modifications: List[Dict[str, Any]],
    kitchen_modifiers: Dict[str, Any],
) -> List[Dict[str, Any]]:
    """Apply modifications to base ingredients and return final ingredient list."""

    # Deep copy to avoid modifying original
    final_ingredients = deepcopy(base_ingredients)


    # Handle ice modifications separately (ice uses grams directly)
    for mod in modifications:
        if mod.get("isIceModification") or (mod.get("category") == "ice" and mod.get("isModified")):
            initial_id = mod.get("initialItemId")
            ice_amount_grams = mod.get("iceAmountGrams") or mod.get("qty", 0)
            
            # Find and update the ice ingredient with actual gram amount
            for i, ingredient in enumerate(final_ingredients):
                if ingredient.get("ingredient_id") == initial_id or ingredient.get("category") == "ice":
                    # Update ice with actual gram amount
                    final_ingredients[i]["unit_amount"] = ice_amount_grams
                    final_ingredients[i]["quantity"] = 1  # quantity is 1, unit_amount is grams
                    final_ingredients[i]["modified"] = True
                    final_ingredients[i]["ice_amount_grams"] = ice_amount_grams
                    break

    # Handle milk amount modifications (user-set milk amount in grams)
    for mod in modifications:
        if mod.get("isMilkAmountModification") or (mod.get("category") == "milk" and mod.get("isAmountModification")):
            initial_id = mod.get("initialItemId")
            amount_grams = mod.get("amountGrams") or mod.get("qty", 0)

            for i, ingredient in enumerate(final_ingredients):
                if ingredient.get("ingredient_id") == initial_id or ingredient.get("category") == "milk":
                    final_ingredients[i]["unit_amount"] = amount_grams
                    final_ingredients[i]["quantity"] = 1  # quantity is 1, unit_amount is grams
                    final_ingredients[i]["modified"] = True
                    break

    # Handle water amount modifications (user-set water amount in grams)
    for mod in modifications:
        if mod.get("isWaterAmountModification") or (mod.get("category") == "water" and mod.get("isAmountModification")):
            initial_id = mod.get("initialItemId")
            amount_grams = mod.get("amountGrams") or mod.get("qty", 0)

            for i, ingredient in enumerate(final_ingredients):
                if ingredient.get("ingredient_id") == initial_id or ingredient.get("category") == "water":
                    final_ingredients[i]["unit_amount"] = amount_grams
                    final_ingredients[i]["quantity"] = 1  # quantity is 1, unit_amount is grams
                    final_ingredients[i]["modified"] = True
                    break
    
    # Apply replacements (modifications where isAddon=False, isModified=True, but NOT ice/milk-amount/water-amount)
    for mod in modifications:
        # Skip ice, milk-amount, and water-amount modifications (handled above)
        if mod.get("isIceModification") or mod.get("category") == "ice":
            continue
        if mod.get("isMilkAmountModification") or mod.get("isWaterAmountModification") or mod.get("isAmountModification"):
            continue
            
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


def _calculate_milk_adjustments(
    ingredients: List[Dict[str, Any]], 
    kitchen_modifiers: Dict[str, Any],
    cup_size: str
) -> Tuple[float, float, float]:
    """
    Calculate milk adjustments based on three-phase capacity logic.
    
    Phase 1: Apply foam reduction based on temperature
    Phase 2: Calculate free space in cup
    Phase 3: Apply milk substitution from addon overflow (max 30%)
    
    Returns: (foam_reduced_milk, milk_substitution_amount, milk_substitution_percent)
    """
    # Find milk ingredient and get base amount
    milk_ingredient = next((ing for ing in ingredients if (ing.get("category", "") or "").lower() == "milk"), None)
    if not milk_ingredient:
        return 0.0, 0.0, 0.0
    
    # Calculate milk volume - convert from grams to ml if needed
    milk_weight = milk_ingredient.get("quantity", 1) * milk_ingredient.get("unit_amount", 1)
    milk_base_units = (milk_ingredient.get("base_units", "") or "").lower()
    
    if milk_base_units == "ml":
        base_milk_amount = milk_weight
    else:
        # Convert grams to ml using milk density
        base_milk_amount = milk_weight / INGREDIENT_DENSITIES.get("milk", 1.03)
    
    # Check if this is an iced drink (cold cup = no foam)
    # Cold cups start with 'C', hot cups start with 'H'
    is_iced = cup_size and str(cup_size).upper().startswith('C')
    
    # Phase 1: Apply foam reduction based on temperature (skip for iced drinks)
    temperature = kitchen_modifiers.get("temperature", "normal")
    if is_iced:
        # Iced drinks have no foam - milk is poured cold, not steamed
        foam_percent = 0
    else:
        foam_percent = TEMPERATURE_FOAM_PERCENTAGES.get(temperature, TEMPERATURE_FOAM_PERCENTAGES["normal"])
    foam_reduced_milk = base_milk_amount * (1 - foam_percent / 100)
    
    # Phase 2: Calculate cup free space
    cup_volume = CUP_VOLUMES.get(cup_size, 266)  # Default to H9
    
    # Calculate foam volume (foam takes up space in the cup)
    # For iced drinks, this will be 0
    foam_volume = base_milk_amount * (foam_percent / 100)
    
    # Calculate fixed recipe volume (non-milk ingredients that cannot be adjusted)
    fixed_recipe_volume = foam_volume  # Foam takes up cup space
    
    # Categories that don't occupy liquid volume OR are handled separately
    skip_categories = ["cups", "position", "ice", "milk", "temperature"]  # milk handled separately, temperature is not a volume
    
    for ing in ingredients:
        category = (ing.get("category", "") or "").lower()
        
        # Skip non-liquid categories, milk (calculated separately), and add-ons
        if category in skip_categories or ing.get("is_addon"):
            continue
            
        qty = ing.get("quantity", 1)
        amount = ing.get("unit_amount", 0)
        base_units = (ing.get("base_units", "") or "").lower()
        
        # Espresso shots - use actual weights
        if base_units == "shots" or category == "espresso":
            espresso_type = (ing.get("type", "double_shot") or "double_shot").lower().replace(" ", "_")
            shot_weight = ESPRESSO_SHOT_WEIGHTS.get(espresso_type, ESPRESSO_SHOT_WEIGHTS["double_shot"])
            espresso_volume = (shot_weight * qty) / INGREDIENT_DENSITIES.get("espresso", 1.02)
            fixed_recipe_volume += espresso_volume
        # Direct liquid volumes (ml)
        elif base_units == "ml":
            fixed_recipe_volume += qty * amount
        # Default: treat as grams and convert to volume using density
        # This handles 'grams', 'g', 'pumps', undefined, etc.
        else:
            density = INGREDIENT_DENSITIES.get(category, 1.0)
            volume = (amount * qty) / density
            fixed_recipe_volume += volume
    
    # Phase 3: Calculate addon volume
    total_addon_volume = 0.0
    for ing in ingredients:
        if ing.get("is_addon"):
            category = (ing.get("category", "extras") or "extras").lower()
            
            # Skip ice - it's measured in grams and doesn't affect liquid volume
            if category == "ice":
                continue
            
            qty = ing.get("quantity", 1)
            weight = ing.get("unit_amount", 10) * qty  # Assume weight in grams
            
            # Convert weight to volume using density
            density = INGREDIENT_DENSITIES.get(category, INGREDIENT_DENSITIES.get("extras", 1.15))
            volume = weight / density
            total_addon_volume += volume
    
    # Calculate available space for milk
    # Cup capacity - fixed ingredients - add-ons = space available for milk
    space_for_milk = cup_volume - fixed_recipe_volume - total_addon_volume
    
    # Milk must be reduced if foam_reduced_milk exceeds available space
    if space_for_milk < foam_reduced_milk:
        milk_substitution_amount = foam_reduced_milk - max(0, space_for_milk)
    else:
        milk_substitution_amount = 0
    
    milk_substitution_percent = (milk_substitution_amount / foam_reduced_milk * 100) if foam_reduced_milk > 0 else 0
    
    # Check if exceeds 30% limit
    max_substitution = foam_reduced_milk * (MAX_MILK_SUBSTITUTION_PERCENT / 100)
    if milk_substitution_amount > max_substitution:
        print(f"WARNING: Milk substitution ({milk_substitution_percent:.1f}%) exceeds {MAX_MILK_SUBSTITUTION_PERCENT}% limit")
    
    return foam_reduced_milk, milk_substitution_amount, milk_substitution_percent


def _create_ingredient_list(
    ingredients: List[Dict[str, Any]], 
    kitchen_modifiers: Dict[str, Any],
    cup_size: str = "H9"
) -> List[ParsedIngredient]:
    """Create flat list of ingredients with kitchen modifications applied."""

    ingredient_list: List[ParsedIngredient] = []
    
    # Calculate milk adjustments using three-phase logic
    foam_reduced_milk, milk_substitution_amount, milk_substitution_percent = _calculate_milk_adjustments(
        ingredients, kitchen_modifiers, cup_size
    )

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
        
        # For ice, use ice_amount_grams directly if available (skip quantity * unit_amount multiplication)
        # Check both ingredient and kitchen_modifiers for ice grams
        ice_grams_from_ingredient = ingredient.get("ice_amount_grams")
        ice_grams_from_modifiers = kitchen_modifiers.get("ice_amount_grams")
        ice_grams = ice_grams_from_ingredient or ice_grams_from_modifiers
        
        if category == "ice" and ice_grams:
            ingredient_obj = ParsedIngredient(
                category=ingredient["category"],
                type=ingredient["type"],
                ingredient_id=mapped_ingredient_id,
                ingredient_name=ingredient.get(
                    "ingredient_name", _lookup_ingredient_name(original_ingredient_id)
                ),
                quantity=1,
                unit_amount=ice_grams,
                total_amount=ice_grams,  # Use grams directly, no multiplication
                automated=ingredient.get("automated", True),
                needs_validation=ingredient.get("needs_validation", True),
                cup_position=cup_pos_value
            )
        else:
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
        if category == "milk":
            # Apply foam setting if ingredient is foam-sensitive
            if ingredient.get("foam_sensitive", False):
                ingredient_obj.foam = kitchen_modifiers.get("foam", "normal")
            
            # ALWAYS apply milk adjustments from capacity calculation for milk ingredients
            if foam_reduced_milk > 0:
                adjusted_milk = foam_reduced_milk - milk_substitution_amount
                ingredient_obj.adjusted_amount = adjusted_milk
                ingredient_obj.milk_reduction_percent = milk_substitution_percent
                # Update total_amount to reflect adjustment
                ingredient_obj.total_amount = adjusted_milk

        # Apply ice level to ice ingredients
        if category == "ice":
            ice_level = kitchen_modifiers.get("ice_level", "normal")
            ingredient_obj.level = ice_level
            
            # If ice_grams was already set at creation time, use that
            # Otherwise fall back to legacy ice level behavior for unmodified ice
            if ice_grams:
                ingredient_obj.adjusted_amount = ice_grams
            elif ingredient.get("ice_sensitive", False) and not ingredient.get("ice_amount_grams"):
                # Legacy behavior for drinks where ice wasn't explicitly modified
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
        
        # Get cup size from ingredients for capacity calculation
        cup_size = "H9"  # Default
        cup_ingredient = next((ing for ing in final_ingredients if ing.get("category") == "cups"), None)
        if cup_ingredient:
            cup_size = cup_ingredient.get("type", "H9")

        # Create flat ingredient list with modifiers applied
        ingredients_list = _create_ingredient_list(final_ingredients, kitchen_modifiers, cup_size)

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


