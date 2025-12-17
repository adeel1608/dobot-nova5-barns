#!/bin/bash

# BARNS Docker Images Build Script
# Builds all BARNS service images on the worker node

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Docker Images Build v${BARNS_DEPLOY_VERSION}"

# Get configuration
WORKER_IP=$(get_config "ip" "auto")
WORKER_USER=$(get_config "ssh_user" "barns")
WORKER_HOST=$(get_config "hostname" "barns-nuc15crsu7")
BUILD_NO_CACHE=$(get_config "no_cache" "true")
REGISTRY=$(get_config "registry" "")
TAG=$(get_config "tag" "latest")

# Auto-detect worker IP if needed
if [ "$WORKER_IP" = "auto" ]; then
    print_info "Auto-detecting worker node IP..."
    # Try to get IP from kubectl
    if command_exists kubectl; then
        WORKER_IP=$(kubectl get nodes -o wide | grep "$WORKER_HOST" | awk '{print $6}')
    fi
    
    if [ -z "$WORKER_IP" ] || [ "$WORKER_IP" = "<none>" ]; then
        print_error "Could not auto-detect worker IP"
        read -p "Enter worker node IP address: " WORKER_IP
    fi
fi

print_info "Worker Node: ${WORKER_USER}@${WORKER_IP}"

# Check if we can SSH to worker
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "${WORKER_USER}@${WORKER_IP}" echo ok &> /dev/null; then
    print_warning "Cannot SSH to worker without password"
    print_info "You may be prompted for password"
fi

echo ""
echo "Build Configuration:"
echo "  Worker: ${WORKER_IP}"
echo "  Tag: ${TAG}"
echo "  No Cache: ${BUILD_NO_CACHE}"
echo "  Registry: ${REGISTRY:-<none>}"
echo ""

if ! ask_yes_no "Start building images?"; then
    print_warning "Build cancelled"
    exit 0
fi

# Define services and their Dockerfiles
declare -A SERVICES=(
    ["api-bridge"]="services/api-bridge/Dockerfile"
    ["validation"]="services/validation/Dockerfile.rabbitmq"
    ["automation"]="services/automation/Dockerfile.rabbitmq"
    ["routine"]="services/routine/Dockerfile.rabbitmq"
    ["robot-arm"]="services/robot_arm/Dockerfile.rabbitmq"
    ["scheduler"]="services/scheduler/Dockerfile.rabbitmq"
    ["oms"]="services/oms/Dockerfile.rabbitmq"
    ["video-stream"]="services/video-stream/Dockerfile"
    ["dashboard"]="services/barns-dashboard/Dockerfile.rabbitmq"
)

# Step 1: Find BARNS source directory on worker
print_header "Step 1: Locating BARNS Source"

BARNS_REMOTE_DIR=$(ssh "${WORKER_USER}@${WORKER_IP}" "
    if [ -d ~/git-BARNS/BARNS ]; then
        echo ~/git-BARNS/BARNS
    elif [ -d ~/BARNS ]; then
        echo ~/BARNS
    elif [ -d /opt/BARNS ]; then
        echo /opt/BARNS
    else
        echo ''
    fi
")

if [ -z "$BARNS_REMOTE_DIR" ]; then
    print_error "BARNS source directory not found on worker node"
    print_info "Please clone the repository first:"
    echo "  ssh ${WORKER_USER}@${WORKER_IP}"
    echo "  git clone <repo-url> ~/git-BARNS/BARNS"
    exit 1
fi

print_status "Found BARNS at: $BARNS_REMOTE_DIR"

# Step 2: Create build script
print_header "Step 2: Creating Build Script"

BUILD_SCRIPT=$(cat <<'EOFSCRIPT'
#!/bin/bash

set -e

BARNS_DIR="$1"
NO_CACHE="$2"
TAG="$3"

cd "$BARNS_DIR"

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[✓]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[!]${NC} $1"; }
print_error() { echo -e "${RED}[✗]${NC} $1"; }

# Check Docker
if ! command -v docker &> /dev/null; then
    print_error "Docker not found"
    exit 1
fi

print_status "Docker found: $(docker --version)"

# Verify architecture
ARCH=$(uname -m)
print_status "Architecture: $ARCH"

# Build options
BUILD_OPTS=""
if [ "$NO_CACHE" = "true" ]; then
    BUILD_OPTS="--no-cache"
fi

# Services to build
declare -A SERVICES=(
    ["api-bridge"]="services/api-bridge/Dockerfile"
    ["validation"]="services/validation/Dockerfile.rabbitmq"
    ["automation"]="services/automation/Dockerfile.rabbitmq"
    ["routine"]="services/routine/Dockerfile.rabbitmq"
    ["robot-arm"]="services/robot_arm/Dockerfile.rabbitmq"
    ["scheduler"]="services/scheduler/Dockerfile.rabbitmq"
    ["oms"]="services/oms/Dockerfile.rabbitmq"
    ["video-stream"]="services/video-stream/Dockerfile"
    ["dashboard"]="services/barns-dashboard/Dockerfile.rabbitmq"
)

BUILD_FAILED=0

for service in "${!SERVICES[@]}"; do
    dockerfile="${SERVICES[$service]}"
    image_name="barns-${service}:${TAG}"
    
    echo ""
    echo "========================================"
    echo "Building: $service"
    echo "========================================"
    
    # Clean old images
    print_status "Cleaning old images..."
    docker rmi -f "$image_name" 2>/dev/null || true
    sudo ctr -n k8s.io images rm "docker.io/library/${image_name}" 2>/dev/null || true
    
    # Build
    print_status "Building $image_name..."
    if docker build $BUILD_OPTS -t "$image_name" -f "$dockerfile" .; then
        print_status "Built successfully"
        
        # Verify architecture
        arch=$(docker inspect "$image_name" | grep Architecture | head -1 | awk '{print $2}' | tr -d '",')
        echo "  Architecture: $arch"
        
        # Import to containerd
        print_status "Importing to containerd..."
        docker save "$image_name" | sudo ctr -n k8s.io image import -
        print_status "Imported to containerd"
    else
        print_error "Build failed for $service"
        BUILD_FAILED=1
    fi
done

echo ""
echo "========================================"
echo "Build Summary"
echo "========================================"

if [ $BUILD_FAILED -eq 0 ]; then
    print_status "All images built successfully!"
else
    print_error "Some images failed to build"
    exit 1
fi

echo ""
echo "Docker Images:"
docker images | grep "barns-"

echo ""
echo "Containerd Images:"
sudo ctr -n k8s.io images ls | grep "barns-"

EOFSCRIPT
)

# Copy build script to worker
echo "$BUILD_SCRIPT" | ssh "${WORKER_USER}@${WORKER_IP}" "cat > /tmp/build-barns-images.sh && chmod +x /tmp/build-barns-images.sh"
print_status "Build script copied to worker"

# Step 3: Execute build on worker
print_header "Step 3: Building Images on Worker Node"

ssh -t "${WORKER_USER}@${WORKER_IP}" "/tmp/build-barns-images.sh '$BARNS_REMOTE_DIR' '$BUILD_NO_CACHE' '$TAG'"

if [ $? -eq 0 ]; then
    print_status "Build completed successfully"
else
    print_error "Build failed"
    exit 1
fi

# Step 4: Push to registry (if configured)
if [ -n "$REGISTRY" ]; then
    print_header "Step 4: Pushing Images to Registry"
    
    for service in "${!SERVICES[@]}"; do
        local_image="barns-${service}:${TAG}"
        remote_image="${REGISTRY}/barns-${service}:${TAG}"
        
        print_info "Tagging and pushing $service..."
        ssh "${WORKER_USER}@${WORKER_IP}" "
            docker tag '$local_image' '$remote_image'
            docker push '$remote_image'
        "
        
        if [ $? -eq 0 ]; then
            print_status "$service pushed"
        else
            print_error "Failed to push $service"
        fi
    done
fi

# Step 5: Verify images in containerd
print_header "Step 5: Verifying Images"

print_info "Checking images in containerd..."
ssh "${WORKER_USER}@${WORKER_IP}" "sudo ctr -n k8s.io images ls | grep barns-"

# Completion
print_header "Build Complete!"

echo ""
print_status "All BARNS images have been built and imported to containerd"
echo ""
echo "Images are ready for deployment on worker node: $WORKER_IP"
echo ""
echo "Next Steps:"
echo "1. Deploy to Kubernetes:"
echo "   ./deploy-k8s.sh"
echo ""
echo "2. Or restart existing pods:"
echo "   kubectl rollout restart deployment -n barns"
echo ""

log_message "INFO" "Image build completed successfully"

