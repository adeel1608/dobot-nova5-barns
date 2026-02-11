#!/bin/bash

# BARNS Docker Image Build Script (multi-arch: amd64 + arm64)
# - Without REGISTRY_PATH: builds for current host (auto-detected amd64 or arm64), loads locally.
# - With REGISTRY_PATH: builds for both linux/amd64 and linux/arm64 and pushes to registry.
#   Example: export REGISTRY_PATH=me-central2-docker.pkg.dev/qss-development-project/barns
# Run from the BARNS root directory.

set -e  # Exit on error

echo "========================================="
echo "BARNS Image Build Script"
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
    BUILDER_NAME="barns-builder"
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

# Platform: auto-detect host so images run on current machine; multi-arch when pushing to registry
HOST_ARCH=$(uname -m)
case "$HOST_ARCH" in
    x86_64)  BUILD_PLATFORM="linux/amd64" ;;
    aarch64|arm64) BUILD_PLATFORM="linux/arm64" ;;
    *)       BUILD_PLATFORM="linux/amd64" ; print_warning "Unknown arch $HOST_ARCH, defaulting to linux/amd64" ;;
esac

MULTI_ARCH=false
if [ -n "${REGISTRY_PATH:-}" ] && [ "$USE_BUILDX" = true ]; then
    MULTI_ARCH=true
    BUILD_PLATFORM_OPT="--platform linux/amd64,linux/arm64"
    PUSH_OPT="--push"
    print_status "Multi-arch build (amd64 + arm64) will push to registry: $REGISTRY_PATH"
else
    BUILD_PLATFORM_OPT="--platform $BUILD_PLATFORM"
    if [ "$USE_BUILDX" = true ]; then
        PUSH_OPT="--load"
    else
        PUSH_OPT=""
    fi
    print_status "Building for current host: $BUILD_PLATFORM"
fi

tag_for_image() {
    if [ "$MULTI_ARCH" = true ]; then
        echo "${REGISTRY_PATH}/${1}:latest"
    else
        echo "${1}:latest"
    fi
}

# Determine build command
if [ "$USE_BUILDX" = true ] && docker buildx version &> /dev/null; then
    BUILD_CMD="docker buildx build"
    print_status "Using docker buildx for builds"
else
    BUILD_CMD="docker build"
    print_warning "Using regular docker build (native architecture only)"
fi

# Ensure we're in the root directory
if [ ! -f "docker-compose.yml" ]; then
    print_error "docker-compose.yml not found. Please run this script from the BARNS root directory."
    exit 1
fi

print_status "Build context verified"

# Build all images
echo ""
if [ "$MULTI_ARCH" = true ]; then
    echo "Building images for linux/amd64 + linux/arm64 (push to registry)..."
else
    echo "Building images for $BUILD_PLATFORM (local load)..."
fi
echo ""

# # API Bridge
# echo "Building api-bridge..."
# IMAGE_TAG=$(tag_for_image "barns-api-bridge")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/api-bridge/Dockerfile \
#   .
# if [ $? -eq 0 ]; then
#     print_status "api-bridge built successfully"
# else
#     print_error "api-bridge build failed"
#     exit 1
# fi

# # Validation Service
# echo ""
# echo "Building validation-service..."
# IMAGE_TAG=$(tag_for_image "barns-validation")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/validation/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "validation-service built successfully"
# else
#     print_error "validation-service build failed"
#     exit 1
# fi

# Automation Service
echo ""
echo "Building automation-service..."
IMAGE_TAG=$(tag_for_image "barns-automation")
$BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
  -f services/automation/Dockerfile.rabbitmq \
  .
if [ $? -eq 0 ]; then
    print_status "automation-service built successfully"
else
    print_error "automation-service build failed"
    exit 1
fi

# # Routine Service
# echo ""
# echo "Building routine-service..."
# IMAGE_TAG=$(tag_for_image "barns-routine")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/routine/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "routine-service built successfully"
# else
#     print_error "routine-service build failed"
#     exit 1
# fi

# # Robot Arm Service
# echo ""
# echo "Building robot-arm-service..."
# IMAGE_TAG=$(tag_for_image "barns-robot-arm")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/robot_arm/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "robot-arm-service built successfully"
# else
#     print_error "robot-arm-service build failed"
#     exit 1
# fi

# # Scheduler Service
# echo ""
# echo "Building scheduler-service..."
# IMAGE_TAG=$(tag_for_image "barns-scheduler")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/scheduler/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "scheduler-service built successfully"
# else
#     print_error "scheduler-service build failed"
#     exit 1
# fi

# # OMS Service
# echo ""
# echo "Building oms-service..."
# IMAGE_TAG=$(tag_for_image "barns-oms")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/oms/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "oms-service built successfully"
# else
#     print_error "oms-service build failed"
#     exit 1
# fi

# # Video Stream Service
# echo ""
# echo "Building video-stream-service..."
# IMAGE_TAG=$(tag_for_image "barns-video-stream")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/video-stream/Dockerfile \
#   .
# if [ $? -eq 0 ]; then
#     print_status "video-stream-service built successfully"
# else
#     print_error "video-stream-service build failed"
#     exit 1
# fi

# # Dashboard
# echo ""
# echo "Building dashboard..."
# IMAGE_TAG=$(tag_for_image "barns-dashboard")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/barns-dashboard/Dockerfile.rabbitmq \
#   .
# if [ $? -eq 0 ]; then
#     print_status "dashboard built successfully"
# else
#     print_error "dashboard build failed"
#     exit 1
# fi

# # Robot1 Service
# echo ""
# echo "Building robot1-service..."
# IMAGE_TAG=$(tag_for_image "barns-robot1")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/robot/Dockerfile.robot1 \
#   .
# if [ $? -eq 0 ]; then
#     print_status "robot1-service built successfully"
#     if [ "$MULTI_ARCH" != true ]; then
#     # #region agent log
#     LOG_FILE="d:\\D-Drive\\BARNS\\.cursor\\debug.log"
#     TIMESTAMP=$(date +%s%3N)
#     echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:robot1-build\",\"message\":\"Robot1 image built successfully\",\"data\":{\"image\":\"barns-robot1:latest\",\"dockerExists\":$(docker images barns-robot1:latest --format '{{.ID}}' | wc -l)},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}" >> "$LOG_FILE" 2>/dev/null || true
#     # #endregion
    
#     # Check if image exists in Docker
#     if docker images barns-robot1:latest --format "{{.Repository}}:{{.Tag}}" | grep -q "barns-robot1:latest"; then
#         DOCKER_IMAGE_ID=$(docker images barns-robot1:latest --format "{{.ID}}")
        
#         # #region agent log
#         TIMESTAMP=$(date +%s%3N)
#         echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:docker-verify\",\"message\":\"Image verified in Docker\",\"data\":{\"image\":\"barns-robot1:latest\",\"id\":\"${DOCKER_IMAGE_ID}\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}" >> "$LOG_FILE" 2>/dev/null || true
#         # #endregion
        
#         # Check if containerd import is needed
#         if command -v ctr &> /dev/null; then
#             if ! ctr -n k8s.io images ls 2>/dev/null | grep -q "barns-robot1:latest"; then
#                 print_warning "Image built in Docker but not found in containerd"
#                 print_warning "Kubernetes uses containerd, so you need to import the image:"
#                 echo "  docker save barns-robot1:latest | sudo ctr -n k8s.io images import -"
                
#                 # #region agent log
#                 TIMESTAMP=$(date +%s%3N)
#                 echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:containerd-check\",\"message\":\"Image not in containerd - import needed\",\"data\":{\"image\":\"barns-robot1:latest\",\"dockerExists\":true,\"containerdExists\":false},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}" >> "$LOG_FILE" 2>/dev/null || true
#                 # #endregion
#             else
#                 # #region agent log
#                 TIMESTAMP=$(date +%s%3N)
#                 CONTAINERD_REF=$(ctr -n k8s.io images ls 2>/dev/null | grep "barns-robot1:latest" | awk '{print $1}' | head -1)
#                 echo "{\"id\":\"log_${TIMESTAMP}_${RANDOM}\",\"timestamp\":${TIMESTAMP},\"location\":\"build-images-arm64.sh:containerd-check\",\"message\":\"Image found in containerd\",\"data\":{\"image\":\"barns-robot1:latest\",\"containerdRef\":\"${CONTAINERD_REF}\",\"containerdExists\":true},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}" >> "$LOG_FILE" 2>/dev/null || true
#                 # #endregion
#             fi
#         fi
#     fi
#     fi
# else
#     print_error "robot1-service build failed"
#     exit 1
# fi

# # Robot2 Service
# echo ""
# echo "Building robot2-service..."
# IMAGE_TAG=$(tag_for_image "barns-robot2")
# $BUILD_CMD $BUILD_PLATFORM_OPT -t "$IMAGE_TAG" $PUSH_OPT \
#   -f services/robot_container/docker/dev.Dockerfile \
#   services/robot_container
# if [ $? -eq 0 ]; then
#     print_status "robot2-service built successfully"
# else
#     print_error "robot2-service build failed"
#     exit 1
# fi


# Import to containerd only when images were built locally (single-arch)
if [ "$MULTI_ARCH" != true ]; then
    cat > import-to-containerd.sh << 'EOF'
#!/bin/bash
set -e
GREEN='\033[0;32m'
NC='\033[0m'
print_status() { echo -e "${GREEN}[✓]${NC} $1"; }

IMAGES=(
    # "barns-api-bridge:latest"
    # "barns-validation:latest"
    "barns-automation:latest"
    # "barns-routine:latest"
    # "barns-robot-arm:latest"
    # "barns-scheduler:latest"
    # "barns-oms:latest"
    # "barns-video-stream:latest"
    # "barns-dashboard:latest"
    # "barns-robot1:latest"
    # "barns-robot2:latest"
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
    sudo ctr -n k8s.io images ls | grep barns || true
fi

echo ""
echo "========================================="
echo "Build Summary"
echo "========================================="
echo ""
if [ "$MULTI_ARCH" = true ]; then
    print_status "All images built and pushed for linux/amd64 + linux/arm64!"
    echo ""
    echo "Images are in registry: $REGISTRY_PATH"
else
    print_status "All images built successfully for $BUILD_PLATFORM!"
    echo ""
    echo "Built images:"
    docker images | grep "barns-" | grep "latest" || true
    echo ""
    print_warning "To build for both amd64 and arm64 and push to registry, set REGISTRY_PATH:"
    echo "  export REGISTRY_PATH=me-central2-docker.pkg.dev/qss-development-project/barns"
    echo "  ./build-images-arm64.sh"
fi
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

