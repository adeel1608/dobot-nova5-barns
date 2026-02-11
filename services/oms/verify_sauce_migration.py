#!/usr/bin/env python3
"""Quick verification script to check sauce category migration"""
import sqlite3

conn = sqlite3.connect('pos_reference.db')
cursor = conn.cursor()

print("\n" + "=" * 70)
print("Sauce & Syrup Ingredients Verification")
print("=" * 70)

# Check sauces
cursor.execute("""
    SELECT ingredient_id, category, type 
    FROM ingredient_details 
    WHERE category = 'sauce'
    ORDER BY ingredient_id
""")
sauces = cursor.fetchall()

print(f"\nSAUCE Category ({len(sauces)} items):")
print("-" * 70)
for ingredient_id, category, ing_type in sauces:
    print(f"  {ingredient_id:30} | {ing_type:20} | {category}")

# Check syrups
cursor.execute("""
    SELECT ingredient_id, category, type 
    FROM ingredient_details 
    WHERE category = 'syrups'
    ORDER BY ingredient_id
""")
syrups = cursor.fetchall()

print(f"\nSYRUPS Category ({len(syrups)} items):")
print("-" * 70)
for ingredient_id, category, ing_type in syrups:
    print(f"  {ingredient_id:30} | {ing_type:20} | {category}")

conn.close()

print("\n" + "=" * 70)
if sauces and syrups:
    print("[SUCCESS] Sauces and syrups are properly separated!")
elif not sauces:
    print("[WARNING] No sauce ingredients found!")
elif not syrups:
    print("[WARNING] No syrup ingredients found!")
print("=" * 70 + "\n")
