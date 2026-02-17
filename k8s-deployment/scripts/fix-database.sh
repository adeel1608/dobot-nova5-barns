#!/bin/bash

# BARNS Database Initialization and Fix Script
# Fixes inventory subtypes to match inventory_rules.json

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Database Initialization v${BARNS_DEPLOY_VERSION}"

# Check if kubectl is available
if ! command_exists kubectl; then
    print_error "kubectl not found"
    exit 1
fi

# Get configuration
NAMESPACE=$(get_config "namespace" "barns")
POSTGRES_POD="postgres-0"

print_info "Namespace: $NAMESPACE"
print_info "Postgres Pod: $POSTGRES_POD"

# Check if postgres pod exists
if ! kubectl get pod "$POSTGRES_POD" -n "$NAMESPACE" &> /dev/null; then
    print_error "PostgreSQL pod not found: $POSTGRES_POD"
    print_info "Check if pods are running:"
    echo "  kubectl get pods -n $NAMESPACE"
    exit 1
fi

print_status "PostgreSQL pod found"

# Wait for postgres to be ready
print_header "Waiting for PostgreSQL"

wait_for_condition "kubectl get pod $POSTGRES_POD -n $NAMESPACE -o jsonpath='{.status.conditions[?(@.type==\"Ready\")].status}' | grep -q True" 300 "Waiting for PostgreSQL to be ready..."

print_status "PostgreSQL is ready"

# Step 1: Check database initialization
print_header "Step 1: Checking Database Initialization"

print_info "Checking if databases exist..."

DBS=$(kubectl exec "$POSTGRES_POD" -n "$NAMESPACE" -- psql -U postgres -t -c "SELECT datname FROM pg_database WHERE datname LIKE 'barns%';" 2>/dev/null | tr -d ' ' | grep -v '^$' || true)

if echo "$DBS" | grep -q "barns_validation"; then
    print_status "barns_validation database exists"
else
    print_error "barns_validation database not found"
    print_info "Database may not be initialized. Check postgres logs:"
    echo "  kubectl logs $POSTGRES_POD -n $NAMESPACE"
    exit 1
fi

if echo "$DBS" | grep -q "barns_oms"; then
    print_status "barns_oms database exists"
else
    print_warning "barns_oms database not found (may be normal)"
fi

# Step 2: Fix inventory subtypes
print_header "Step 2: Fixing Inventory Subtypes"

print_info "Updating inventory to match inventory_rules.json..."

# Execute SQL fixes
kubectl exec -it "$POSTGRES_POD" -n "$NAMESPACE" -- psql -U validation_user -d barns_validation <<'EOSQL'
-- Fix cups (add "cup_" prefix)
UPDATE inventory SET subtype = 'cup_H7' WHERE subtype = 'H7' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_H9' WHERE subtype = 'H9' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_H12' WHERE subtype = 'H12' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_C7' WHERE subtype = 'C7' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_C9' WHERE subtype = 'C9' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_C12' WHERE subtype = 'C12' AND category = 'cups';
UPDATE inventory SET subtype = 'cup_C16' WHERE subtype = 'C16' AND category = 'cups';

-- Fix milk (add "_milk" suffix if needed)
UPDATE inventory SET subtype = 'whole_fat_milk' WHERE subtype = 'whole_fat' AND category = 'milk';
UPDATE inventory SET subtype = 'low_fat_milk' WHERE subtype = 'low_fat' AND category = 'milk';
UPDATE inventory SET subtype = 'lactose_free_milk' WHERE subtype = 'lactose_free' AND category = 'milk';
UPDATE inventory SET subtype = 'almond_milk' WHERE subtype = 'almond' AND category = 'milk';

-- Fix syrups (add "_syrup" suffix)
UPDATE inventory SET subtype = 'vanilla_syrup' WHERE subtype = 'vanilla' AND category = 'syrups';
UPDATE inventory SET subtype = 'caramel_syrup' WHERE subtype = 'caramel' AND category = 'syrups';
UPDATE inventory SET subtype = 'hazelnut_syrup' WHERE subtype = 'hazelnut' AND category = 'syrups';
UPDATE inventory SET subtype = 'peached_iced_syrup' WHERE subtype LIKE '%peach%' AND category = 'syrups';
UPDATE inventory SET subtype = 'passion_fruit_iced_syrup' WHERE subtype LIKE '%passion%' AND category = 'syrups';
UPDATE inventory SET subtype = 'ice_tea_syrup' WHERE subtype LIKE '%tea%' AND category = 'syrups';

-- Move sauces to syrups category and fix names
UPDATE inventory SET category = 'syrups', subtype = 'white_chocolate_sauce' 
WHERE (subtype = 'white_chocolate' OR subtype = 'white chocolate') AND category = 'sauces';

UPDATE inventory SET category = 'syrups', subtype = 'caramel_sauce' 
WHERE subtype LIKE '%caramel%' AND category = 'sauces';

UPDATE inventory SET category = 'syrups', subtype = 'condense_milk_sauce' 
WHERE (subtype = 'condense_milk' OR subtype = 'condensed_milk') AND category = 'sauces';

-- Show current inventory status
\echo ''
\echo 'Current Inventory:'
\echo '=================='
SELECT category, subtype, quantity, COALESCE(max_capacity, 0) as max_capacity 
FROM inventory 
ORDER BY category, subtype;
EOSQL

if [ $? -eq 0 ]; then
    print_status "Inventory fixed successfully"
else
    print_error "Failed to fix inventory"
    exit 1
fi

# Step 3: Verify inventory
print_header "Step 3: Verifying Inventory"

print_info "Checking inventory by category..."

kubectl exec "$POSTGRES_POD" -n "$NAMESPACE" -- psql -U validation_user -d barns_validation -c "
SELECT 
    category, 
    COUNT(*) as item_count,
    SUM(quantity) as total_quantity
FROM inventory 
GROUP BY category 
ORDER BY category;
"

# Step 4: Restart validation service
print_header "Step 4: Restarting Validation Service"

if kubectl get deployment validation-service -n "$NAMESPACE" &> /dev/null; then
    print_info "Restarting validation service to apply changes..."
    kubectl rollout restart deployment validation-service -n "$NAMESPACE"
    
    print_info "Waiting for validation service to be ready..."
    sleep 10
    
    if kubectl wait --for=condition=available deployment/validation-service -n "$NAMESPACE" --timeout=60s 2>/dev/null; then
        print_status "Validation service restarted successfully"
    else
        print_warning "Validation service restart timeout (may still be starting)"
    fi
else
    print_warning "Validation service deployment not found"
fi

# Completion
print_header "Database Initialization Complete!"

echo ""
print_status "Database is initialized and inventory is fixed"
echo ""
print_info "If you still see inventory-related errors, check logs:"
echo "  kubectl logs -l app=validation-service -n $NAMESPACE"
echo ""
print_info "To manually connect to database:"
echo "  kubectl exec -it $POSTGRES_POD -n $NAMESPACE -- psql -U validation_user -d barns_validation"
echo ""

log_message "INFO" "Database initialization completed"

