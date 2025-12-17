#!/bin/bash

# BARNS Deployment Validation Script
# Validates the health and status of BARNS deployment

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Deployment Validation v${BARNS_DEPLOY_VERSION}"

# Check if kubectl is available
if ! command_exists kubectl; then
    print_error "kubectl not found"
    exit 1
fi

# Get configuration
NAMESPACE=$(get_config "namespace" "barns")

# Get worker IP
WORKER_IP=$(kubectl get nodes -o wide | grep -v "control-plane" | grep -v "NAME" | awk '{print $6}' | head -1)
if [ -z "$WORKER_IP" ]; then
    WORKER_IP="<UNKNOWN>"
fi

print_info "Namespace: $NAMESPACE"
print_info "Worker IP: $WORKER_IP"

VALIDATION_FAILED=0

# Test 1: Check Namespace
print_header "Test 1: Namespace"

if kubectl get namespace "$NAMESPACE" &> /dev/null; then
    print_status "Namespace '$NAMESPACE' exists"
else
    print_error "Namespace '$NAMESPACE' not found"
    VALIDATION_FAILED=1
    exit 1
fi

# Test 2: Check Nodes
print_header "Test 2: Cluster Nodes"

echo ""
kubectl get nodes -o wide
echo ""

READY_NODES=$(kubectl get nodes --no-headers | grep -c " Ready" || true)
TOTAL_NODES=$(kubectl get nodes --no-headers | wc -l)

if [ "$READY_NODES" -eq "$TOTAL_NODES" ]; then
    print_status "All $TOTAL_NODES nodes are Ready"
else
    print_warning "$READY_NODES/$TOTAL_NODES nodes are Ready"
    VALIDATION_FAILED=1
fi

# Test 3: Check Pods
print_header "Test 3: Pod Status"

echo ""
kubectl get pods -n "$NAMESPACE"
echo ""

TOTAL_PODS=$(kubectl get pods -n "$NAMESPACE" --no-headers | wc -l)
RUNNING_PODS=$(kubectl get pods -n "$NAMESPACE" --no-headers | grep -c "Running" || true)
FAILED_PODS=$(kubectl get pods -n "$NAMESPACE" --no-headers | grep -Ec "Error|CrashLoopBackOff|ImagePullBackOff|ErrImagePull" || true)

echo "Pod Statistics:"
echo "  Total: $TOTAL_PODS"
echo "  Running: $RUNNING_PODS"
echo "  Failed: $FAILED_PODS"
echo ""

if [ "$FAILED_PODS" -eq 0 ]; then
    print_status "No failed pods"
else
    print_error "$FAILED_PODS pod(s) are failing"
    echo ""
    echo "Failed Pods:"
    kubectl get pods -n "$NAMESPACE" | grep -E "Error|CrashLoopBackOff|ImagePullBackOff|ErrImagePull"
    VALIDATION_FAILED=1
fi

# Test 4: Check Services
print_header "Test 4: Services"

echo ""
kubectl get svc -n "$NAMESPACE"
echo ""

NODEPORT_SERVICES=$(kubectl get svc -n "$NAMESPACE" | grep -c "NodePort" || true)

if [ "$NODEPORT_SERVICES" -gt 0 ]; then
    print_status "Found $NODEPORT_SERVICES NodePort service(s)"
else
    print_warning "No NodePort services found"
fi

# Test 5: Check Storage
print_header "Test 5: Persistent Storage"

echo ""
echo "Persistent Volumes:"
kubectl get pv | grep "$NAMESPACE" || echo "  None found"
echo ""
echo "Persistent Volume Claims:"
kubectl get pvc -n "$NAMESPACE"
echo ""

BOUND_PVCS=$(kubectl get pvc -n "$NAMESPACE" --no-headers | grep -c "Bound" || true)
TOTAL_PVCS=$(kubectl get pvc -n "$NAMESPACE" --no-headers | wc -l)

if [ "$TOTAL_PVCS" -gt 0 ]; then
    if [ "$BOUND_PVCS" -eq "$TOTAL_PVCS" ]; then
        print_status "All $TOTAL_PVCS PVCs are Bound"
    else
        print_warning "$BOUND_PVCS/$TOTAL_PVCS PVCs are Bound"
        VALIDATION_FAILED=1
    fi
else
    print_warning "No PVCs found"
fi

# Test 6: Check Database
print_header "Test 6: Database Connectivity"

POSTGRES_POD=$(kubectl get pods -n "$NAMESPACE" -l app=postgres -o name | head -1 | cut -d'/' -f2)

if [ -n "$POSTGRES_POD" ]; then
    if kubectl exec "$POSTGRES_POD" -n "$NAMESPACE" -- pg_isready -U postgres &> /dev/null; then
        print_status "PostgreSQL is ready"
        
        # Check databases
        DBS=$(kubectl exec "$POSTGRES_POD" -n "$NAMESPACE" -- psql -U postgres -t -c "SELECT datname FROM pg_database WHERE datname LIKE 'barns%';" 2>/dev/null | tr -d ' ' | grep -v '^$' || true)
        
        if echo "$DBS" | grep -q "barns_validation"; then
            print_status "barns_validation database exists"
        else
            print_error "barns_validation database not found"
            VALIDATION_FAILED=1
        fi
        
        if echo "$DBS" | grep -q "barns_oms"; then
            print_status "barns_oms database exists"
        else
            print_warning "barns_oms database not found"
        fi
    else
        print_error "PostgreSQL is not ready"
        VALIDATION_FAILED=1
    fi
else
    print_error "PostgreSQL pod not found"
    VALIDATION_FAILED=1
fi

# Test 7: Check RabbitMQ
print_header "Test 7: RabbitMQ Status"

RABBITMQ_POD=$(kubectl get pods -n "$NAMESPACE" -l app=rabbitmq -o name | head -1 | cut -d'/' -f2)

if [ -n "$RABBITMQ_POD" ]; then
    if kubectl get pod "$RABBITMQ_POD" -n "$NAMESPACE" | grep -q "Running"; then
        print_status "RabbitMQ is running"
    else
        print_error "RabbitMQ is not running"
        VALIDATION_FAILED=1
    fi
else
    print_error "RabbitMQ pod not found"
    VALIDATION_FAILED=1
fi

# Test 8: Check Redis
print_header "Test 8: Redis Status"

REDIS_POD=$(kubectl get pods -n "$NAMESPACE" -l app=redis -o name | head -1 | cut -d'/' -f2)

if [ -n "$REDIS_POD" ]; then
    if kubectl get pod "$REDIS_POD" -n "$NAMESPACE" | grep -q "Running"; then
        print_status "Redis is running"
    else
        print_error "Redis is not running"
        VALIDATION_FAILED=1
    fi
else
    print_error "Redis pod not found"
    VALIDATION_FAILED=1
fi

# Test 9: Check Network Connectivity
print_header "Test 9: Service Endpoints"

echo ""
echo "Testing external access..."
echo ""

services=("dashboard:30003" "api-bridge:30000" "video-stream-service:30001")

for svc_port in "${services[@]}"; do
    IFS=':' read -r svc port <<< "$svc_port"
    
    if [ "$WORKER_IP" != "<UNKNOWN>" ]; then
        if timeout 3 bash -c "echo > /dev/tcp/${WORKER_IP}/${port}" 2>/dev/null; then
            print_status "$svc is accessible on port $port"
        else
            print_warning "$svc is not accessible on port $port (may be normal if not on same network)"
        fi
    else
        print_warning "Cannot test $svc (worker IP unknown)"
    fi
done

# Test 10: Check Recent Logs for Errors
print_header "Test 10: Recent Error Logs"

echo ""
echo "Checking for recent errors in logs..."
echo ""

VALIDATION_POD=$(kubectl get pods -n "$NAMESPACE" -l app=validation-service -o name | head -1 | cut -d'/' -f2)

if [ -n "$VALIDATION_POD" ]; then
    ERRORS=$(kubectl logs "$VALIDATION_POD" -n "$NAMESPACE" --tail=50 2>/dev/null | grep -i "error" | wc -l)
    
    if [ "$ERRORS" -gt 0 ]; then
        print_warning "Found $ERRORS error(s) in validation service logs"
        echo "  Run: kubectl logs $VALIDATION_POD -n $NAMESPACE"
    else
        print_status "No recent errors in validation service"
    fi
fi

# Summary
print_header "Validation Summary"

echo ""
if [ "$VALIDATION_FAILED" -eq 0 ]; then
    print_status "All validation tests passed!"
    echo ""
    echo "🎉 BARNS deployment is healthy and ready to use!"
    echo ""
    show_summary "$WORKER_IP"
    echo ""
    exit 0
else
    print_error "Some validation tests failed"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check pod logs:"
    echo "     kubectl logs <pod-name> -n $NAMESPACE"
    echo ""
    echo "  2. Describe failing pods:"
    echo "     kubectl describe pod <pod-name> -n $NAMESPACE"
    echo ""
    echo "  3. Check events:"
    echo "     kubectl get events -n $NAMESPACE --sort-by='.lastTimestamp'"
    echo ""
    echo "  4. Run database fix:"
    echo "     ./fix-database.sh"
    echo ""
    exit 1
fi

