#!/usr/bin/env bash
###############################
# BARNS Robot 1 Docker Image Build Script
# Builds image locally without pushing to registry
###############################

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() {
    echo -e "${GREEN}[BUILD-ROBOT1]${NC} $*"
}

error() {
    echo -e "${RED}[ERROR]${NC} $*"
    exit 1
}

info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME=${IMAGE_NAME:-"barns-robot1"}
IMAGE_TAG=${IMAGE_TAG:-"latest1"}
NO_CACHE=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --no-cache)
            NO_CACHE=true
            ;;
        --help|-h)
            echo "Usage: $(basename "$0") [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-cache   Force a full rebuild with no Docker layer cache"
            echo "  --help, -h   Show this help message"
            echo ""
            echo "When only oms_v1 has changed, the default (cached) build will"
            echo "skip the expensive ROS workspace compilation and only re-copy"
            echo "the oms_v1 Python files (~seconds instead of ~15-30 min)."
            exit 0
            ;;
    esac
done

log "Building Robot 1 Docker Image"
log "Project Root: $PROJECT_ROOT"
log "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo

# Check if Dockerfile exists
if [ ! -f "${SCRIPT_DIR}/Dockerfile.robot1" ]; then
    error "Dockerfile.robot1 not found in ${SCRIPT_DIR}"
fi

# Check if required directories exist
info "Checking required directories..."
if [ ! -d "${PROJECT_ROOT}/services/robot_container/ros_ws/src" ]; then
    warn "ROS workspace source directory not found at ${PROJECT_ROOT}/services/robot_container/ros_ws/src"
    warn "The build may fail if this directory is required"
fi

if [ ! -d "${PROJECT_ROOT}/shared" ]; then
    warn "Shared directory not found at ${PROJECT_ROOT}/shared"
    warn "The build may fail if this directory is required"
fi

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    error "Docker is not installed or not in PATH"
fi

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    error "Docker daemon is not running"
fi

# Build the image
if [ "$NO_CACHE" = true ]; then
    log "Starting Docker build for Robot 1 (NO CACHE - full rebuild)..."
    log "This may take 15-30 minutes depending on your system..."
else
    log "Starting Docker build for Robot 1 (cached)..."
    log "If only oms_v1 changed, the ROS workspace build will be cached (~seconds)."
fi
echo

cd "${PROJECT_ROOT}"

DOCKER_BUILD_ARGS=(
    -f "${SCRIPT_DIR}/Dockerfile.robot1"
    -t "${IMAGE_NAME}:${IMAGE_TAG}"
    --build-arg BUILDKIT_INLINE_CACHE=1
)

if [ "$NO_CACHE" = true ]; then
    DOCKER_BUILD_ARGS+=(--no-cache)
fi

docker build "${DOCKER_BUILD_ARGS[@]}" .

if [ $? -eq 0 ]; then
    echo
    log "✓ Build completed successfully!"
    echo
    info "Image built: ${IMAGE_NAME}:${IMAGE_TAG}"
    echo
    info "Image details:"
    docker images "${IMAGE_NAME}:${IMAGE_TAG}"
    echo
    info "To run the container:"
    echo "  docker run --privileged --network host \\"
    echo "    -e RABBITMQ_URL=amqp://admin:admin123@<rabbitmq-host>:5672/ \\"
    echo "    -e IP_ADDRESS=192.168.200.249 \\"
    echo "    ${IMAGE_NAME}:${IMAGE_TAG}"
    echo
else
    error "Build failed!"
fi