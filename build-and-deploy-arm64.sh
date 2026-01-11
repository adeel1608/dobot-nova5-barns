#!/bin/bash

# BARNS Docker Image Build and Deploy Script for ARM64
# This script builds images and makes them available on Kubernetes nodes
# Run from the BARNS root directory

set -e  # Exit on error

echo "========================================="
echo "BARNS ARM64 Build and Deploy Script"
echo "========================================="
echo ""

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check if docker is available
if ! command -v docker &> /dev/null; then
    print_error "docker is not installed or not in PATH"
    exit 1
fi

print_status "docker found"

# Check if buildx is available
if ! docker buildx version &> /dev/null; then
    print_error "docker buildx is not available"
    exit 1
fi

print_status "docker buildx found"

# Check if kubectl is available
if ! command -v kubectl &> /dev/null; then
    print_error "kubectl is not installed or not in PATH"
    exit 1
fi

print_status "kubectl found"

# Get the worker node name
WORKER_NODE=$(kubectl get nodes -o jsonpath='{.items[?(@.metadata.labels.node-role\.kubernetes\.io/control-plane=="" || @.metadata.labels.node-role\.kubernetes\.io/master=="")].metadata.name}' | head -1)
if [ -z "$WORKER_NODE" ]; then
    # If no control plane label, get the first non-control-plane node
    WORKER_NODE=$(kubectl get nodes -o jsonpath='{.items[0].metadata.name}')
fi

echo ""
echo "Detected worker node: $WORKER_NODE"
echo ""
echo "Choose deployment method:"
echo "1) Build on this machine and copy to worker node (requires SSH access)"
echo "2) Build directly on worker node (requires SSH access)"
echo "3) Build locally only (manual copy required)"
echo ""
read -p "Enter choice [1-3]: " choice

case $choice in
    1)
        echo ""
        read -p "Enter SSH user for worker node [default: root]: " SSH_USER
        SSH_USER=${SSH_USER:-root}
        read -p "Enter worker node IP or hostname: " WORKER_HOST
        
        print_status "Building images locally..."
        ./build-images-arm64.sh
        
        print_status "Copying images to worker node..."
        for image in barns-api-bridge barns-validation barns-automation barns-routine barns-robot-arm barns-scheduler barns-oms barns-video-stream barns-dashboard barns-robot1 barns-robot2; do
            echo "  - Copying $image:latest..."
            docker save ${image}:latest | ssh ${SSH_USER}@${WORKER_HOST} "docker load"
        done
        
        print_status "Images copied to worker node"
        ;;
    2)
        echo ""
        read -p "Enter SSH user for worker node [default: root]: " SSH_USER
        SSH_USER=${SSH_USER:-root}
        read -p "Enter worker node IP or hostname: " WORKER_HOST
        
        print_status "Copying build script to worker node..."
        scp build-images-arm64.sh ${SSH_USER}@${WORKER_HOST}:/tmp/
        scp -r services/ shared/ data/ ${SSH_USER}@${WORKER_HOST}:/tmp/barns-build/ || {
            print_warning "Could not copy all files. Building on worker node may require manual setup."
        }
        
        print_status "Building images on worker node..."
        ssh ${SSH_USER}@${WORKER_HOST} "cd /tmp && chmod +x build-images-arm64.sh && ./build-images-arm64.sh"
        
        print_status "Images built on worker node"
        ;;
    3)
        print_status "Building images locally only..."
        ./build-images-arm64.sh
        print_warning "Images built locally. You need to manually copy them to the worker node."
        echo ""
        echo "To copy images manually, use:"
        echo "  docker save <image>:latest | ssh user@worker-node 'docker load'"
        echo ""
        echo "Or if using containerd on worker node:"
        echo "  docker save <image>:latest | ssh user@worker-node 'ctr -n k8s.io images import -'"
        exit 0
        ;;
    *)
        print_error "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "========================================="
echo "Restarting Kubernetes Deployments"
echo "========================================="
echo ""

# Force delete pods to ensure they use new images
print_status "Restarting all deployments..."
kubectl rollout restart deployment -n barns

echo ""
print_status "Waiting for pods to restart..."
sleep 10

echo ""
echo "Current pod status:"
kubectl get pods -n barns

echo ""
print_warning "Monitor pod status with: kubectl get pods -n barns -w"
print_warning "Check logs with: kubectl logs <pod-name> -n barns"

