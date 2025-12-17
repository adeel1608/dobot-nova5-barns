#!/bin/bash

# BARNS Kubernetes Cleanup Script
# Removes BARNS deployment from Kubernetes

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Cleanup v${BARNS_DEPLOY_VERSION}"

# Check if kubectl is available
if ! command_exists kubectl; then
    print_error "kubectl not found"
    exit 1
fi

# Get configuration
NAMESPACE=$(get_config "namespace" "barns")

print_warning "This will DELETE all BARNS resources from Kubernetes"
print_warning "Namespace: $NAMESPACE"
echo ""

if ! ask_yes_no "Are you sure you want to continue?"; then
    print_info "Cleanup cancelled"
    exit 0
fi

echo ""
if ! ask_yes_no "Delete persistent data as well? (Cannot be undone!)"; then
    DELETE_DATA=false
    print_info "Persistent data will be preserved"
else
    DELETE_DATA=true
    print_warning "Persistent data will be DELETED"
fi

echo ""

# Step 1: Delete deployments and statefulsets
print_header "Step 1: Deleting Deployments"

kubectl delete deployments --all -n "$NAMESPACE" 2>/dev/null || true
kubectl delete statefulsets --all -n "$NAMESPACE" 2>/dev/null || true

print_status "Deployments and StatefulSets deleted"

# Wait for pods to terminate
print_info "Waiting for pods to terminate..."
sleep 10

# Force delete remaining pods
REMAINING_PODS=$(kubectl get pods -n "$NAMESPACE" --no-headers 2>/dev/null | wc -l)
if [ "$REMAINING_PODS" -gt 0 ]; then
    print_info "Force deleting remaining pods..."
    kubectl delete pods --all -n "$NAMESPACE" --force --grace-period=0 2>/dev/null || true
fi

# Step 2: Delete services
print_header "Step 2: Deleting Services"

kubectl delete services --all -n "$NAMESPACE" 2>/dev/null || true

print_status "Services deleted"

# Step 3: Delete configmaps and secrets
print_header "Step 3: Deleting ConfigMaps and Secrets"

kubectl delete configmaps --all -n "$NAMESPACE" 2>/dev/null || true
kubectl delete secrets --all -n "$NAMESPACE" 2>/dev/null || true

print_status "ConfigMaps and Secrets deleted"

# Step 4: Delete storage
if [ "$DELETE_DATA" = true ]; then
    print_header "Step 4: Deleting Persistent Storage"
    
    print_warning "Deleting PVCs and PVs..."
    kubectl delete pvc --all -n "$NAMESPACE" 2>/dev/null || true
    kubectl delete pv -l app=barns 2>/dev/null || true
    
    print_status "Storage deleted"
    
    # Get worker node info
    WORKER_IP=$(get_config "ip" "auto")
    WORKER_USER=$(get_config "ssh_user" "barns")
    STORAGE_BASE=$(get_config "base_path" "/mnt/barns-data")
    
    if [ "$WORKER_IP" != "auto" ]; then
        print_info "Attempting to delete data from worker node..."
        
        if ask_yes_no "Delete data directories on worker node ($STORAGE_BASE)?"; then
            ssh "${WORKER_USER}@${WORKER_IP}" "sudo rm -rf ${STORAGE_BASE}/*" 2>/dev/null || \
                print_warning "Could not delete data from worker node (may need manual cleanup)"
        fi
    else
        print_warning "Worker IP not configured, cannot delete data automatically"
        echo "  Manually delete data on worker node:"
        echo "  sudo rm -rf /mnt/barns-data/*"
    fi
else
    print_info "Skipping storage deletion (data preserved)"
fi

# Step 5: Delete namespace
print_header "Step 5: Deleting Namespace"

if ask_yes_no "Delete namespace '$NAMESPACE'?"; then
    kubectl delete namespace "$NAMESPACE" 2>/dev/null || true
    
    print_info "Waiting for namespace to be deleted..."
    kubectl wait --for=delete namespace/"$NAMESPACE" --timeout=60s 2>/dev/null || true
    
    print_status "Namespace deleted"
else
    print_info "Namespace preserved"
fi

# Completion
print_header "Cleanup Complete!"

echo ""
print_status "BARNS has been removed from Kubernetes"
echo ""

if [ "$DELETE_DATA" = true ]; then
    print_warning "All data has been deleted"
else
    print_info "Persistent data has been preserved"
    echo "  PVCs and PVs can be reused for future deployments"
fi

echo ""
echo "To redeploy BARNS:"
echo "  ./deploy-k8s.sh"
echo ""

log_message "INFO" "Cleanup completed"

