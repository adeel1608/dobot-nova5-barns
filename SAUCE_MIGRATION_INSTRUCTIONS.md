# Sauce Category Migration - Quick Fix Guide

## The Error You're Seeing

```
2026-01-26 07:16:29 - WARNING - No inventory found for sauce:white_chocolate_sauce
2026-01-26 07:16:29 - ERROR - Error loading inventory data: 'NoneType' object has no attribute 'get'
```

This error occurs because the database still has sauce items under the 'syrups' category, but the code is now looking for them under the 'sauce' category.

---

## Solution

### Option 1: Run the Migration Script (Recommended)

Execute the migration script to update your existing database:

```bash
# Navigate to your project root
cd /path/to/BARNS

# Run the migration script
docker exec -i barns-postgresql psql -U validation_user -d barns_validation < db-validation-sauce-migration.sql
```

Or if running PostgreSQL locally:

```bash
psql -h localhost -U validation_user -d barns_validation -f db-validation-sauce-migration.sql
```

### Option 2: Manual SQL Update

Connect to your PostgreSQL database and run:

```sql
-- Connect to the validation database
\c barns_validation validation_user;

-- Update sauce items from 'syrups' to 'sauce' category
UPDATE public.inventory 
SET ingredient_type = 'sauce' 
WHERE ingredient_type = 'syrups' 
  AND subtype IN ('white_chocolate_sauce', 'caramel_sauce', 'condense_milk_sauce');

-- Verify the update
SELECT * FROM public.inventory WHERE ingredient_type = 'sauce';
```

### Option 3: Rebuild Database from Schema (Fresh Start)

If you can afford to reset your database:

```bash
# Stop services
docker-compose down

# Remove volumes (WARNING: This deletes all data)
docker volume rm barns_postgresql_data

# Start services (will recreate database with updated schema)
docker-compose up -d
```

---

## Verification

After running the migration, verify it worked:

1. **Check database:**
   ```sql
   SELECT ingredient_type, subtype, current_amount 
   FROM public.inventory 
   WHERE subtype LIKE '%sauce%';
   ```
   
   Should show:
   ```
   ingredient_type |        subtype         | current_amount
   ----------------+------------------------+----------------
   sauce          | white_chocolate_sauce  | 1000.00
   sauce          | caramel_sauce          | 1000.00
   sauce          | condense_milk_sauce    | 200.00
   ```

2. **Check validation service logs:**
   ```bash
   docker logs barns-validation-1 2>&1 | grep -i sauce
   ```
   
   Should NOT show any "No inventory found" errors for sauce items.

3. **Check inventory API:**
   ```bash
   curl http://localhost:8000/api/inventory/status | jq .sauce
   ```
   
   Should return sauce inventory data.

---

## What Was Fixed

1. **inventory_manager.py:**
   - Added 'sauce' to inventory_cache initialization
   - Fixed NoneType error when database doesn't have sauce records
   - Improved error handling in load_inventory_data()

2. **db-validation-schema.sql:**
   - Updated sauce INSERT statements to use 'sauce' category instead of 'syrups'

3. **Migration script created:**
   - `db-validation-sauce-migration.sql` - Safe migration for existing databases

---

## Need Help?

If you continue to see errors after migration:

1. Check validation service logs:
   ```bash
   docker logs barns-validation-1 --tail 100
   ```

2. Verify database connection:
   ```bash
   docker exec -it barns-postgresql psql -U validation_user -d barns_validation
   ```

3. Check inventory cache is loading:
   Look for "Loaded inventory data!" in validation logs

4. Restart validation service:
   ```bash
   docker restart barns-validation-1
   ```

---

## Files Modified

- `services/validation/inventory_manager.py` - Fixed error handling
- `db-validation-schema.sql` - Updated initial data
- `db-validation-sauce-migration.sql` - NEW migration script

See `SAUCE_CATEGORY_SPLIT_CHANGELOG.md` for complete list of all changes.
