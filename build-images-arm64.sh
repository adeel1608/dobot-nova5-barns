#!/bin/bash

# BARNS Docker Image Build Script for ARM64
# This script builds all BARNS service images for ARM64 architecture
# Run from the BARNS root directory

set -e  # Exit on error

echo "========================================="
echo "BARNS ARM64 Image Build Script"
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

# Check if buildx is available, try to install if not
USE_BUILDX=false
if docker buildx version &> /dev/null; then
    USE_BUILDX=true
    print_status "docker buildx found"
    
    # Create buildx builder if it doesn't exist
    BUILDER_NAME="barns-arm64-builder"
    if ! docker buildx ls | grep -q "$BUILDER_NAME"; then
        echo "Creating buildx builder: $BUILDER_NAME"
        docker buildx create --name "$BUILDER_NAME" --use 2>/dev/null || {
            print_warning "Could not create buildx builder, using default"
            docker buildx use default 2>/dev/null || USE_BUILDX=false
        }
        if [ "$USE_BUILDX" = true ]; then
            print_status "Builder created"
        fi
    else
        echo "Using existing buildx builder: $BUILDER_NAME"
        docker buildx use "$BUILDER_NAME" 2>/dev/null || USE_BUILDX=false
    fi
else
    print_warning "docker buildx not found, attempting to install..."
    
    # Try to install buildx
    if command -v apt-get &> /dev/null; then
        echo "Installing docker-buildx-plugin..."
        sudo apt-get update -qq && sudo apt-get install -y docker-buildx-plugin 2>/dev/null && {
            USE_BUILDX=true
            print_status "docker buildx installed"
        } || {
            print_warning "Could not install buildx via apt, will use regular docker build"
        }
    elif [ -f ~/.docker/cli-plugins/docker-buildx ] || [ -f /usr/lib/docker/cli-plugins/docker-buildx ]; then
        print_warning "buildx plugin exists but not working, will use regular docker build"
    else
        print_warning "buildx not available, will use regular docker build (native ARM64)"
    fi
fi

# Determine build command
if [ "$USE_BUILDX" = true ] && docker buildx version &> /dev/null; then
    BUILD_CMD="docker buildx build --platform linux/arm64 --load"
    print_status "Using docker buildx for builds"
else
    BUILD_CMD="docker build"
    print_warning "Using regular docker build (will build for native ARM64 architecture)"
    print_warning "Note: If you're on ARM64, this will work. If not, install buildx."
fi

# Ensure we're in the root directory
if [ ! -f "docker-compose.yml" ]; then
    print_error "docker-compose.yml not found. Please run this script from the BARNS root directory."
    exit 1
fi

print_status "Build context verified"

# Build all images
echo ""
echo "Building images for linux/arm64..."
echo ""

# API Bridge
echo "Building api-bridge..."
$BUILD_CMD \
  -t barns-api-bridge:latest \
  -f services/api-bridge/Dockerfile \
  .
if [ $? -eq 0 ]; then
    print_status "api-bridge built successfully"
else
    print_error "api-bridge build failed"
    exit 1
fi

# Validation Service
echo ""
echo "Building validation-service..."
$BUILD_CMD \
  -t barns-validation:latest \
  -f services/validation/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "validation-service built successfully"
else
    print_error "validation-service build failed"
    exit 1
fi

# Automation Service
echo ""
echo "Building automation-service..."
$BUILD_CMD \
  -t barns-automation:latest \
  -f services/automation/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "automation-service built successfully"
else
    print_error "automation-service build failed"
    exit 1
fi

# Routine Service
echo ""
echo "Building routine-service..."
$BUILD_CMD \
  -t barns-routine:latest \
  -f services/routine/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "routine-service built successfully"
else
    print_error "routine-service build failed"
    exit 1
fi

# Robot Arm Service
echo ""
echo "Building robot-arm-service..."
$BUILD_CMD \
  -t barns-robot-arm:latest \
  -f services/robot_arm/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "robot-arm-service built successfully"
else
    print_error "robot-arm-service build failed"
    exit 1
fi

# Scheduler Service
echo ""
echo "Building scheduler-service..."
$BUILD_CMD \
  -t barns-scheduler:latest \
  -f services/scheduler/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "scheduler-service built successfully"
else
    print_error "scheduler-service build failed"
    exit 1
fi

# OMS Service
echo ""
echo "Building oms-service..."
$BUILD_CMD \
  -t barns-oms:latest \
  -f services/oms/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "oms-service built successfully"
else
    print_error "oms-service build failed"
    exit 1
fi

# Video Stream Service
echo ""
echo "Building video-stream-service..."
$BUILD_CMD \
  -t barns-video-stream:latest \
  -f services/video-stream/Dockerfile \
  .
if [ $? -eq 0 ]; then
    print_status "video-stream-service built successfully"
else
    print_error "video-stream-service build failed"
    exit 1
fi

# Dashboard
echo ""
echo "Building dashboard..."
$BUILD_CMD \
  -t barns-dashboard:latest \
  -f services/barns-dashboard/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "dashboard built successfully"
else
    print_error "dashboard build failed"
    exit 1
fi

# Robot1 Service
echo ""
echo "Building robot1-service..."
$BUILD_CMD \
  -t barns-robot1:latest \
  -f services/robot/Dockerfile.robot1 \
  .
if [ $? -eq 0 ]; then
    print_status "robot1-service built successfully"
else
    print_error "robot1-service build failed"
    exit 1
fi

# Robot2 Service
echo ""
echo "Building robot2-service..."
$BUILD_CMD \
  -t barns-robot2:latest \
  -f services/robot/Dockerfile.robot2 \
  .
if [ $? -eq 0 ]; then
    print_status "robot2-service built successfully"
else
    print_error "robot2-service build failed"
    exit 1
fi

echo ""
echo "========================================="
echo "Build Summary"
echo "========================================="
echo ""
print_status "All images built successfully for ARM64!"
echo ""
echo "Built images:"
docker images | grep "barns-" | grep "latest"
echo ""
print_warning "Note: Images are built locally. If using a registry, tag and push them:"
echo "  docker tag barns-api-bridge:latest <registry>/barns-api-bridge:latest"
echo "  docker push <registry>/barns-api-bridge:latest"
echo ""
print_warning "After building, restart your Kubernetes pods to use the new images:"
echo "  kubectl rollout restart deployment/api-bridge -n barns"
echo "  kubectl rollout restart deployment/automation-service -n barns"
echo "  kubectl rollout restart deployment/dashboard -n barns"
echo "  kubectl rollout restart deployment/oms-service -n barns"
echo "  kubectl rollout restart deployment/robot-arm-service -n barns"
echo "  kubectl rollout restart deployment/routine-service -n barns"
echo "  kubectl rollout restart deployment/scheduler-service -n barns"
echo "  kubectl rollout restart deployment/validation-service -n barns"
echo "  kubectl rollout restart deployment/video-stream-service -n barns"
echo ""
echo "  kubectl rollout restart deployment/robot1 -n barns"
echo "  kubectl rollout restart deployment/robot2 -n barns"
echo ""

