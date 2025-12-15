#!/bin/bash

# Sync Docker images to containerd for Kubernetes
# Run this on the worker node (barns-nuc15crsu7)

set -e

echo "========================================="
echo "Syncing Docker Images to Containerd"
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

# Check if docker is available
if ! command -v docker &> /dev/null; then
    print_error "docker is not installed"
    exit 1
fi

# Check if ctr is available
if ! command -v ctr &> /dev/null; then
    print_error "ctr (containerd CLI) is not installed"
    exit 1
fi

print_status "docker and ctr found"

# List of images to sync
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

echo ""
echo "Step 1: Removing old images from containerd..."
for image in "${IMAGES[@]}"; do
    echo "  - Removing old ${image} from containerd..."
    sudo ctr -n k8s.io images rm "docker.io/library/${image}" 2>/dev/null || \
    sudo ctr -n k8s.io images rm "${image}" 2>/dev/null || \
    echo "    (not found, continuing...)"
done

print_status "Old images removed"

echo ""
echo "Step 2: Exporting images from Docker and importing to containerd..."
for image in "${IMAGES[@]}"; do
    echo ""
    echo "  - Syncing ${image}..."
    
    # Check if image exists in Docker
    if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image}$"; then
        print_warning "    Image ${image} not found in Docker, skipping..."
        continue
    fi
    
    # Export from Docker and import to containerd
    TEMP_FILE="/tmp/${image//\//_//\//_}.tar"
    TEMP_FILE="${TEMP_FILE//:/_}"
    
    echo "    Exporting from Docker..."
    docker save "${image}" -o "${TEMP_FILE}"
    
    echo "    Importing to containerd..."
    sudo ctr -n k8s.io images import "${TEMP_FILE}"
    
    echo "    Cleaning up..."
    rm -f "${TEMP_FILE}"
    
    print_status "    ${image} synced successfully"
done

echo ""
echo "Step 3: Verifying images in containerd..."
echo ""
sudo ctr -n k8s.io images ls | grep "barns-" || print_warning "No barns images found in containerd"

echo ""
print_status "Image sync complete!"
echo ""
print_warning "Now restart pods from control plane:"
echo "  kubectl rollout restart deployment -n barns"
echo ""
print_warning "Or delete specific pods:"
echo "  kubectl delete pod -l app=api-bridge -n barns"


