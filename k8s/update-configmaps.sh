#!/bin/bash
###############################
# Update BARNS ConfigMaps from local config/data files
# Run this after editing config/tasks.json or data/recipes.json
# to push the changes into the running K8s cluster.
###############################

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[OK]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[!]${NC} $1"; }
print_error()   { echo -e "${RED}[ERR]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NAMESPACE="barns"

# Validate required files exist
TASKS_FILE="${PROJECT_ROOT}/config/tasks.json"
DATA_DIR="${PROJECT_ROOT}/data"

if [ ! -f "$TASKS_FILE" ]; then
    print_error "config/tasks.json not found at ${TASKS_FILE}"
    exit 1
fi

if [ ! -d "$DATA_DIR" ]; then
    print_error "data/ directory not found at ${DATA_DIR}"
    exit 1
fi

if ! command -v kubectl &> /dev/null; then
    print_error "kubectl is not installed or not in PATH"
    exit 1
fi

echo "========================================="
echo "Updating BARNS ConfigMaps"
echo "========================================="
echo ""

# 1. Update routine-tasks-config (from config/tasks.json)
echo "Updating routine-tasks-config from config/tasks.json..."
kubectl create configmap routine-tasks-config \
    --from-file=tasks.json="${TASKS_FILE}" \
    -n "${NAMESPACE}" --dry-run=client -o yaml | kubectl apply -f -
print_status "routine-tasks-config updated"

# 2. Update scheduler-data (from data/ directory, includes recipes.json)
echo ""
echo "Updating scheduler-data from data/ directory..."
kubectl create configmap scheduler-data \
    --from-file="${DATA_DIR}/" \
    -n "${NAMESPACE}" --dry-run=client -o yaml | kubectl apply -f -
print_status "scheduler-data updated"

# 3. Restart pods that mount these configmaps so they pick up changes
echo ""
echo "Restarting services to pick up new config..."

kubectl rollout restart deployment/routine-service -n "${NAMESPACE}" 2>/dev/null && \
    print_status "routine-service restarted" || \
    print_warning "routine-service not found or restart failed"

kubectl rollout restart deployment/scheduler-service -n "${NAMESPACE}" 2>/dev/null && \
    print_status "scheduler-service restarted" || \
    print_warning "scheduler-service not found or restart failed"

echo ""
echo "========================================="
echo "ConfigMaps Updated"
echo "========================================="
echo ""
print_warning "Pods are restarting. Check status with:"
echo "  kubectl get pods -n ${NAMESPACE} -l 'app in (routine-service,scheduler-service)' -w"
