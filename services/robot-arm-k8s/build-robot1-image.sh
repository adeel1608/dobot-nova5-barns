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
IMAGE_TAG=${IMAGE_TAG:-"latest"}

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
log "Starting Docker build for Robot 1..."
log "This may take 15-30 minutes depending on your system..."
echo

cd "${PROJECT_ROOT}"

docker build \
    -f "${SCRIPT_DIR}/Dockerfile.robot1" \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    --build-arg BUILDKIT_INLINE_CACHE=1 \
    .

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