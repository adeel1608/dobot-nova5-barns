#!/usr/bin/env python3
"""
Script to update sauce ingredients from 'syrups' to 'sauce' category in pos_reference.db
Run this script in the services/oms directory.
"""

import sqlite3
import os

def update_sauce_categories():
    """Update sauce ingredients to use 'sauce' category instead of 'syrups'"""
    
    # Get the database path
    db_path = os.path.join(os.path.dirname(__file__), 'pos_reference.db')
    
    if not os.path.exists(db_path):
        print(f"Error: Database not found at {db_path}")
        return False
    
    try:
        # Connect to the database
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # List of sauce ingredient types to update
        sauce_types = [
            'white_chocolate',
            'white_chocolate_sauce',
            'caramel_sauce',
            'condense_milk',
            'condense_milk_sauce',
            'condensed_milk',
            'condensed_milk_sauce'
        ]
        
        print("Checking current sauce ingredients...")
        cursor.execute("""
            SELECT ingredient_id, category, type 
            FROM ingredient_details 
            WHERE type IN ({})
               OR ingredient_id IN ({})
        """.format(
            ','.join(['?' for _ in sauce_types]),
            ','.join(['?' for _ in sauce_types])
        ), sauce_types + sauce_types)
        
        current_sauces = cursor.fetchall()
        print(f"\nFound {len(current_sauces)} sauce ingredients:")
        for ingredient_id, category, ing_type in current_sauces:
            print(f"  - {ingredient_id} (type: {ing_type}, current category: {category})")
        
        if not current_sauces:
            print("\nNo sauce ingredients found to update.")
            return True
        
        # Update sauce ingredients to 'sauce' category
        print("\nUpdating sauce ingredients to 'sauce' category...")
        cursor.execute("""
            UPDATE ingredient_details 
            SET category = 'sauce'
            WHERE type IN ({})
               OR ingredient_id IN ({})
        """.format(
            ','.join(['?' for _ in sauce_types]),
            ','.join(['?' for _ in sauce_types])
        ), sauce_types + sauce_types)
        
        updated_count = cursor.rowcount
        
        # Commit the changes
        conn.commit()
        
        # Verify the update
        print(f"\nUpdated {updated_count} ingredients.")
        print("\nVerifying update...")
        cursor.execute("""
            SELECT ingredient_id, category, type 
            FROM ingredient_details 
            WHERE category = 'sauce'
        """)
        
        sauce_ingredients = cursor.fetchall()
        print(f"\nIngredients now in 'sauce' category:")
        for ingredient_id, category, ing_type in sauce_ingredients:
            print(f"  - {ingredient_id} (type: {ing_type}, category: {category})")
        
        # Close connection
        conn.close()
        
        print("\n[SUCCESS] Migration completed successfully!")
        print("\nNext steps:")
        print("1. Restart the OMS service to reload the database")
        print("2. Clear browser cache and refresh the frontend")
        print("3. Test creating a new order with sauce ingredients")
        
        return True
        
    except Exception as e:
        print(f"\nError updating database: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("Sauce Category Migration Script")
    print("=" * 60)
    print("\nThis script will update sauce ingredients from 'syrups'")
    print("to 'sauce' category in the pos_reference.db database.\n")
    
    response = input("Continue? (y/n): ")
    if response.lower() == 'y':
        success = update_sauce_categories()
        exit(0 if success else 1)
    else:
        print("Migration cancelled.")
        exit(0)
