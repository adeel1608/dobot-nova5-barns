#!/usr/bin/env python3
"""
pos_mapper.py
~~~~~~~~~~~~~
Parse BARNS‑style POS payloads (see example scenarios in the README)
and resolve all numeric IDs to recipe / ingredient names using a
simple in‑memory reference “database”.

Author: <your‑name>
Created: 2025‑06‑24
"""

import json
from copy import deepcopy
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Any

# ------------------------------------------------------------------------------
# 1) “DATABASE” – replace with real tables or API calls in production
# ------------------------------------------------------------------------------

MENU_ITEMS: Dict[str, Dict[str, Any]] = {
    # Drinks
    "243825": {                       # This ID is reused in the scenarios
        "name": "Cappuccino / Base Coffee",
        "default_ingredients": ["espresso_shot", "213411", "milk_foam"],
    },
    "243826": {                       # Example extra mapping
        "name": "Espresso",
        "default_ingredients": ["espresso_shot"],
    },
    "LATTE001": {
        "name": "Latte",
        "default_ingredients": ["espresso_shot", "213411", "milk_foam"],
    },
    "FLATW001": {
        "name": "Flat White",
        "default_ingredients": ["espresso_shot", "213411", "microfoam"],
    },
    # Bakery
    "muffin": {
        "name": "Chocolate Muffin",
        "default_ingredients": [],
    },
}

INGREDIENTS: Dict[str, str] = {
    # Espresso & dairy
    "espresso_shot": "Espresso Shot",
    "213411": "Full-Fat Milk",
    "213419": "Almond Milk",
    "milk_foam": "Milk Foam",
    "microfoam": "Velvety Micro-foam",

    # Syrups & addons
    "243211": "Flavoured Syrup (Vanilla/Caramel etc.)",
    "245611": "“Addon B” (placeholder)",

    # Sugars (represented only in kitchen notes here)
}

# ------------------------------------------------------------------------------
# 2) Helper functions
# ------------------------------------------------------------------------------

def _lookup_name(db: Dict[str, Any], _id: str, kind: str = "item") -> str:
    """Return human‑readable name for an ID or a placeholder if unknown."""
    if _id in db:
        if isinstance(db[_id], dict):
            return db[_id].get("name", f"<unknown {kind}>")
        return db[_id]
    return f"<unmapped {kind} {_id}>"

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

        parsed_items.append(
            {
                "line": idx,
                "recipe_id": item_id,
                "recipe_name": _lookup_name(MENU_ITEMS, item_id, "recipe"),
                "ordered_qty": item["quantity"],
                "ingredients": ingredient_list,
                "kitchen_notes": kitchen_notes,
            }
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
# 4) Convenience CLI
# ------------------------------------------------------------------------------

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(
        description="Resolve POS JSON payload to recipe + ingredient names"
    )
    parser.add_argument(
        "json_file",
        type=Path,
        help="Path to a single scenario JSON file (or '-' for stdin)",
    )
    args = parser.parse_args()

    raw_json = (
        json.load(open(args.json_file, "r", encoding="utf-8"))
        if args.json_file != Path("-")
        else json.load(sys.stdin)
    )

    parsed = parse_transaction(raw_json)
    print(json.dumps(parsed, indent=4, ensure_ascii=False))


if __name__ == "__main__":
    main()
"""
{
    "transaction_id": "AJDX01-000168-1749705835893",
    "date": "2024-06-12",
    "time": "10:30:00",
    "store_number": "AJDX01",
    "pos_reg_id": "000168",
    "customer_id": "ALC004",
    "items": [
        {
            "line": 1,
            "recipe_id": "243825",
            "recipe_name": "Cappuccino / Base Coffee",
            "ordered_qty": 1,
            "ingredients": [
                {
                    "ingredient_id": "espresso_shot",
                    "ingredient_name": "Espresso Shot",
                    "qty": 1
                },
                {
                    "ingredient_id": "213419",
                    "ingredient_name": "Almond Milk",
                    "qty": 1
                },
                {
                    "ingredient_id": "milk_foam",
                    "ingredient_name": "Milk Foam",
                    "qty": 1
                },
                {
                    "ingredient_id": "243211",
                    "ingredient_name": "Flavoured Syrup (Vanilla/Caramel etc.)",
                    "qty": 1
                }
            ],
            "kitchen_notes": [
                {
                    "type": "Drink Foam",
                    "qty": 0,
                    "detail": "Normal"
                },
                ...
            ]
        }
    ]
}

"""