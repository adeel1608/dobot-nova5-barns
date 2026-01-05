#!/bin/bash

# BARNS Build and Sync Script
# This script builds all BARNS service images for ARM64 and syncs them to containerd
# Run from the BARNS root directory on the worker node

set -e

echo "========================================="
echo "BARNS Build and Sync Tool"
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

# Check for required tools
if ! command -v docker &> /dev/null; then
    print_error "docker is not installed"
    exit 1
fi

if ! command -v ctr &> /dev/null; then
    print_warning "ctr (containerd CLI) is not installed. Will only build images."
    SYNC_ENABLED=false
else
    SYNC_ENABLED=true
fi

# Ensure we're in the root directory
if [ ! -f "docker-compose.yml" ]; then
    print_error "docker-compose.yml not found. Please run this script from the BARNS root directory."
    exit 1
fi

# ----------------------------------------------------------------------
# Part 1: Build Images (from build-images-arm64.sh)
# ----------------------------------------------------------------------

echo ""
echo "----------------------------------------------------------------------"
echo "Part 1: Building Images (ARM64)"
echo "----------------------------------------------------------------------"
echo ""

# Check if buildx is available
USE_BUILDX=false
if docker buildx version &> /dev/null; then
    USE_BUILDX=true
    # Create/use builder
    BUILDER_NAME="barns-arm64-builder"
    if ! docker buildx ls | grep -q "$BUILDER_NAME"; then
        docker buildx create --name "$BUILDER_NAME" --use 2>/dev/null || {
            print_warning "Could not create buildx builder, using default"
            docker buildx use default 2>/dev/null || USE_BUILDX=false
        }
    else
        docker buildx use "$BUILDER_NAME" 2>/dev/null || USE_BUILDX=false
    fi
fi

# Determine build command
if [ "$USE_BUILDX" = true ] && docker buildx version &> /dev/null; then
    BUILD_CMD="docker buildx build --platform linux/arm64 --load"
    print_status "Using docker buildx"
else
    BUILD_CMD="docker build"
    print_warning "Using regular docker build (native arch)"
fi

# Define images and their Dockerfiles
# Format: "image_name|dockerfile_path"
IMAGES_TO_BUILD=(
    "barns-api-bridge:latest|services/api-bridge/Dockerfile"
    "barns-validation:latest|services/validation/Dockerfile.rabbitmq"
    "barns-automation:latest|services/automation/Dockerfile.rabbitmq"
    "barns-routine:latest|services/routine/Dockerfile.rabbitmq"
    "barns-robot-arm:latest|services/robot_arm/Dockerfile.rabbitmq"
    "barns-scheduler:latest|services/scheduler/Dockerfile.rabbitmq"
    "barns-oms:latest|services/oms/Dockerfile.rabbitmq"
    "barns-video-stream:latest|services/video-stream/Dockerfile"
    "barns-dashboard:latest|services/barns-dashboard/Dockerfile.rabbitmq"
)

for entry in "${IMAGES_TO_BUILD[@]}"; do
    IFS="|" read -r image dockerfile <<< "$entry"
    echo "Building $image..."
    $BUILD_CMD -t "$image" -f "$dockerfile" .
    if [ $? -eq 0 ]; then
        print_status "$image built"
    else
        print_error "$image build failed"
        exit 1
    fi
    echo ""
done

print_status "All images built successfully"

# ----------------------------------------------------------------------
# Part 2: Sync to Containerd (from sync-docker-to-containerd.sh)
# ----------------------------------------------------------------------

if [ "$SYNC_ENABLED" = false ]; then
    print_warning "Skipping sync check (ctr not found)"
    exit 0
fi

echo ""
echo "----------------------------------------------------------------------"
echo "Part 2: Syncing to Containerd"
echo "----------------------------------------------------------------------"
echo ""

# Images to sync (same list as built)
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

# Remove old images from containerd to force update
echo "Step 1: Removing old images from containerd..."
for image in "${IMAGES[@]}"; do
    # Try removing docker.io prefix first, then bare
    sudo ctr -n k8s.io images rm "docker.io/library/${image}" 2>/dev/null || true
    sudo ctr -n k8s.io images rm "${image}" 2>/dev/null || true
done
print_status "Old images cleanup attempt finished"

echo ""
echo "Step 2: Exporting from Docker and importing to containerd..."
for image in "${IMAGES[@]}"; do
    echo "  Processing ${image}..."
    
    # Export from Docker
    TEMP_FILE="/tmp/${image//\//_}.tar"
    TEMP_FILE="${TEMP_FILE//:/_}"
    
    docker save "${image}" -o "${TEMP_FILE}"
    
    # Import to containerd
    sudo ctr -n k8s.io images import "${TEMP_FILE}" > /dev/null
    
    # Cleanup
    rm -f "${TEMP_FILE}"
    
    print_status "  ${image} synced"
done

echo ""
echo "----------------------------------------------------------------------"
echo "Complete!"
echo "----------------------------------------------------------------------"
echo ""
print_warning "Now restart deployments to use new images:"
echo "kubectl rollout restart deployment -n barns"
