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

print_status "Importing locally built images to containerd..."
for img in "${IMAGES[@]}"; do
    if docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${img}$"; then
        print_status "Loading ${img} into containerd..."
        docker save "${img}" | sudo ctr -n k8s.io image import -
    else
        echo "Warning: Image ${img} not found locally, skipping containerd import."
    fi
done
print_status "Containerd import complete."
