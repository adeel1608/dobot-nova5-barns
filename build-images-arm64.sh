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

# Check for Jetson/low-memory system
if [ -f /etc/nv_tegra_release ] 2>/dev/null; then
    print_warning "Jetson device detected!"
    print_warning "If you encounter OOM errors (exit code 137), try:"
    print_warning "  1. Add swap space: sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile"
    print_warning "  2. Set COLCON_PARALLEL_WORKERS=2 before building"
    print_warning "  3. Build images one at a time instead of all at once"
    echo ""
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
    
    # #region agent log
    LOG_FILE="d:\\D-Drive\\BARNS\\.cursor\\debug.log"
    TIMESTAMP=$(date +%s%3N)
    echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:robot1-build\",\"message\":\"Robot1 image built successfully\",\"data\":{\"image\":\"barns-robot1:latest\",\"dockerExists\":$(docker images barns-robot1:latest --format '{{.ID}}' | wc -l)},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}" >> "$LOG_FILE" 2>/dev/null || true
    # #endregion
    
    # Check if image exists in Docker
    if docker images barns-robot1:latest --format "{{.Repository}}:{{.Tag}}" | grep -q "barns-robot1:latest"; then
        DOCKER_IMAGE_ID=$(docker images barns-robot1:latest --format "{{.ID}}")
        
        # #region agent log
        TIMESTAMP=$(date +%s%3N)
        echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:docker-verify\",\"message\":\"Image verified in Docker\",\"data\":{\"image\":\"barns-robot1:latest\",\"id\":\"${DOCKER_IMAGE_ID}\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}" >> "$LOG_FILE" 2>/dev/null || true
        # #endregion
        
        # Check if containerd import is needed
        if command -v ctr &> /dev/null; then
            if ! ctr -n k8s.io images ls 2>/dev/null | grep -q "barns-robot1:latest"; then
                print_warning "Image built in Docker but not found in containerd"
                print_warning "Kubernetes uses containerd, so you need to import the image:"
                echo "  docker save barns-robot1:latest | sudo ctr -n k8s.io images import -"
                
                # #region agent log
                TIMESTAMP=$(date +%s%3N)
                echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:containerd-check\",\"message\":\"Image not in containerd - import needed\",\"data\":{\"image\":\"barns-robot1:latest\",\"dockerExists\":true,\"containerdExists\":false},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}" >> "$LOG_FILE" 2>/dev/null || true
                # #endregion
            else
                # #region agent log
                TIMESTAMP=$(date +%s%3N)
                CONTAINERD_REF=$(ctr -n k8s.io images ls 2>/dev/null | grep "barns-robot1:latest" | awk '{print $1}' | head -1)
                echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:containerd-check\",\"message\":\"Image found in containerd\",\"data\":{\"image\":\"barns-robot1:latest\",\"containerdRef\":\"${CONTAINERD_REF}\",\"containerdExists\":true},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}" >> "$LOG_FILE" 2>/dev/null || true
                # #endregion
            fi
        fi
    fi
else
    print_error "robot1-service build failed"
    exit 1
fi

# Robot2 Service
echo ""
echo "Building robot2-service..."
$BUILD_CMD \
  -t barns-robot2:latest \
  -f services/robot_container/docker/dev.Dockerfile \
  services/robot_container
if [ $? -eq 0 ]; then
    print_status "robot2-service built successfully"
else
    print_error "robot2-service build failed"
    exit 1
fi


# 2. Import to containerd on worker
# Create this script on worker:
cat > import-to-containerd.sh << 'EOF'
#!/bin/bash
set -e
GREEN='\033[0;32m'
NC='\033[0m'
print_status() { echo -e "${GREEN}[✓]${NC} $1"; }

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
    "barns-robot1:latest"
    "barns-robot2:latest"
)

for IMAGE in "${IMAGES[@]}"; do
    if docker images "$IMAGE" --format "{{.Repository}}:{{.Tag}}" | grep -q "$IMAGE"; then
        echo "Importing $IMAGE..."
        docker save "$IMAGE" | sudo ctr -n k8s.io images import - --all-platforms
        echo "✓ $IMAGE imported"
    fi
done
EOF

chmod +x import-to-containerd.sh
sudo ./import-to-containerd.sh

# 3. Verify images are available
sudo ctr -n k8s.io images ls | grep barns


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

