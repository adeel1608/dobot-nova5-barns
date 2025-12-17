#!/bin/bash

# BARNS Kubernetes Deployment Script
# Deploys all BARNS services to Kubernetes

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Deployment v${BARNS_DEPLOY_VERSION}"

# Check if kubectl is available
if ! command_exists kubectl; then
    print_error "kubectl not found"
    print_info "Please install kubectl or run this on the master node"
    exit 1
fi

# Check cluster connectivity
if ! kubectl cluster-info &> /dev/null; then
    print_error "Cannot connect to Kubernetes cluster"
    print_info "Make sure you're on the master node or kubectl is configured"
    exit 1
fi

print_status "Connected to Kubernetes cluster"

# Get configuration
NAMESPACE=$(get_config "namespace" "barns")
K8S_DIR="../../../k8s"  # Relative to BARNS/k8s-deployment/scripts
WAIT_TIMEOUT=$(get_config "timeout" "300")

# Find k8s directory
if [ ! -d "$K8S_DIR" ]; then
    # Try alternate locations
    if [ -d "../../k8s" ]; then
        K8S_DIR="../../k8s"
    elif [ -d "$HOME/BARNS/k8s" ]; then
        K8S_DIR="$HOME/BARNS/k8s"
    elif [ -d "$HOME/git-BARNS/BARNS/k8s" ]; then
        K8S_DIR="$HOME/git-BARNS/BARNS/k8s"
    else
        print_error "Cannot find k8s directory"
        read -p "Enter path to BARNS k8s directory: " K8S_DIR
    fi
fi

K8S_DIR=$(realpath "$K8S_DIR")
print_info "Using k8s directory: $K8S_DIR"

if [ ! -d "$K8S_DIR" ]; then
    print_error "k8s directory not found: $K8S_DIR"
    exit 1
fi

cd "$K8S_DIR"

echo ""
echo "Deployment Configuration:"
echo "  Namespace: $NAMESPACE"
echo "  K8s Dir: $K8S_DIR"
echo "  Timeout: ${WAIT_TIMEOUT}s"
echo ""

if ! ask_yes_no "Start deployment?"; then
    print_warning "Deployment cancelled"
    exit 0
fi

# Step 1: Create Namespace
print_header "Step 1: Creating Namespace"

if kubectl get namespace "$NAMESPACE" &> /dev/null; then
    print_info "Namespace '$NAMESPACE' already exists"
else
    if [ -f namespace.yaml ]; then
        kubectl apply -f namespace.yaml
        print_status "Namespace created"
    else
        kubectl create namespace "$NAMESPACE"
        print_status "Namespace created (no YAML file)"
    fi
fi

# Step 2: Create Secrets
print_header "Step 2: Creating Secrets"

if [ -f secrets.yaml ]; then
    kubectl apply -f secrets.yaml
    print_status "Secrets applied"
else
    print_warning "secrets.yaml not found, skipping"
fi

# Step 3: Create ConfigMaps
print_header "Step 3: Creating ConfigMaps"

if [ -d configmaps ]; then
    kubectl apply -f configmaps/
    print_status "ConfigMaps applied"
else
    print_warning "configmaps directory not found, skipping"
fi

# Step 4: Create Storage
print_header "Step 4: Creating Persistent Storage"

if [ -d storage ]; then
    kubectl apply -f storage/
    print_status "Storage created"
    
    print_info "Waiting for PVCs to bind..."
    sleep 10
else
    print_warning "storage directory not found, skipping"
fi

# Step 5: Deploy Infrastructure
print_header "Step 5: Deploying Infrastructure Services"

if [ -d infrastructure ]; then
    kubectl apply -f infrastructure/
    print_status "Infrastructure services deployed"
    
    print_info "Waiting for infrastructure to be ready..."
    
    # Wait for key infrastructure services
    services=("postgres" "rabbitmq" "redis" "influxdb")
    for svc in "${services[@]}"; do
        echo -n "  Waiting for $svc..."
        if kubectl wait --for=condition=ready pod -l app="$svc" -n "$NAMESPACE" --timeout=120s 2>/dev/null; then
            echo " ✓"
        else
            echo " timeout (may still be starting)"
        fi
    done
    
    print_status "Infrastructure ready"
else
    print_warning "infrastructure directory not found, skipping"
fi

# Step 6: Deploy Application Services
print_header "Step 6: Deploying Application Services"

if [ -d services ]; then
    kubectl apply -f services/
    print_status "Application services deployed"
else
    print_warning "services directory not found, skipping"
fi

# Step 7: Deploy Frontend Services
print_header "Step 7: Deploying Frontend Services"

if [ -d frontend ]; then
    kubectl apply -f frontend/
    print_status "Frontend services deployed"
else
    print_warning "frontend directory not found, skipping"
fi

# Step 8: Wait for pods to be ready
print_header "Step 8: Waiting for Pods"

print_info "Waiting for all pods to be ready..."
sleep 15

kubectl get pods -n "$NAMESPACE"

# Step 9: Show deployment status
print_header "Step 9: Deployment Status"

echo ""
echo "All Resources:"
kubectl get all -n "$NAMESPACE"

echo ""
echo "NodePort Services:"
kubectl get svc -n "$NAMESPACE" | grep NodePort || echo "No NodePort services found"

# Step 10: Get worker IP for access URLs
WORKER_IP=$(kubectl get nodes -o wide | grep -v "control-plane" | grep -v "NAME" | awk '{print $6}' | head -1)

if [ -z "$WORKER_IP" ]; then
    WORKER_IP="<WORKER-NODE-IP>"
    print_warning "Could not detect worker IP automatically"
fi

# Completion
print_header "Deployment Complete!"

show_summary "$WORKER_IP"

print_warning "Note: Some pods may still be initializing. Monitor with:"
echo "  kubectl get pods -n $NAMESPACE -w"
echo ""
print_warning "Check logs if pods are failing:"
echo "  kubectl logs <pod-name> -n $NAMESPACE"
echo ""

# Check for failed pods
FAILED_PODS=$(kubectl get pods -n "$NAMESPACE" --no-headers 2>/dev/null | grep -E "Error|CrashLoopBackOff|ImagePullBackOff" | wc -l)

if [ "$FAILED_PODS" -gt 0 ]; then
    print_warning "Warning: $FAILED_PODS pod(s) are in failed state"
    echo ""
    kubectl get pods -n "$NAMESPACE" | grep -E "Error|CrashLoopBackOff|ImagePullBackOff"
    echo ""
    print_info "Run ./fix-database.sh if you see database-related errors"
    print_info "Run ./validate.sh to check deployment health"
fi

log_message "INFO" "Kubernetes deployment completed"

