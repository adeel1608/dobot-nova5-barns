#!/bin/bash

# Force Kubernetes to use newly built ARM64 images
# Run this on the worker node where pods are running

set -e

echo "========================================="
echo "Force Update Kubernetes Images"
echo "========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check if we're on the worker node or control plane
if command -v kubectl &> /dev/null; then
    IS_CONTROL_PLANE=true
    print_status "Running on control plane, will use kubectl"
else
    IS_CONTROL_PLANE=false
    print_warning "Running on worker node, will use containerd directly"
fi

# List of images to update
IMAGES=(
    "barns-api-bridge:latest"
    "barns-validation:latest"
    "barns-automation:latest"
    "barns-routine:latest"
    "barns-robot-arm:latest"
    "barns-scheduler:latest"
    "barns-oms:latest"
    "barns-video-stream:latest"
    "barns-dashboard:latest"
)

if [ "$IS_CONTROL_PLANE" = true ]; then
    echo ""
    echo "Step 1: Deleting pods to force recreation..."
    for image in "${IMAGES[@]}"; do
        service_name=$(echo $image | sed 's/barns-\(.*\):latest/\1/' | sed 's/-service//' | sed 's/-/_/g')
        if [ "$service_name" = "api_bridge" ]; then
            service_name="api-bridge"
        elif [ "$service_name" = "video_stream" ]; then
            service_name="video-stream-service"
        elif [ "$service_name" = "robot_arm" ]; then
            service_name="robot-arm-service"
        fi
        
        echo "  - Deleting pods for ${service_name}..."
        kubectl delete pods -l app=${service_name} -n barns --force --grace-period=0 2>/dev/null || true
    done
    
    print_status "Pods deleted"
    
    echo ""
    echo "Step 2: Restarting deployments..."
    kubectl rollout restart deployment -n barns
    
    print_status "Deployments restarted"
    
    echo ""
    echo "Waiting for pods to restart..."
    sleep 5
    
    echo ""
    echo "Current pod status:"
    kubectl get pods -n barns
    
else
    echo ""
    echo "Step 1: Removing old images from containerd..."
    
    # Remove old images using ctr
    for image in "${IMAGES[@]}"; do
        echo "  - Removing old ${image}..."
        sudo ctr -n k8s.io images rm "docker.io/library/${image}" 2>/dev/null || \
        sudo ctr -n k8s.io images rm "${image}" 2>/dev/null || \
        echo "    Image not found in containerd (may already be removed)"
    done
    
    print_status "Old images removed from containerd"
    
    echo ""
    print_warning "Now you need to restart pods from control plane:"
    echo "  kubectl rollout restart deployment -n barns"
fi

echo ""
print_status "Done! Monitor pod status with:"
echo "  kubectl get pods -n barns -w"
echo ""
print_warning "If pods still fail, check logs:"
echo "  kubectl logs <pod-name> -n barns"

