#!/bin/bash
# BARNS Robot Image Build Script
# Usage: ./build-robot-images.sh robot1   (on master)
#        ./build-robot-images.sh robot2   (on worker)

set -euo pipefail

ROBOT=$1

if [[ "$ROBOT" != "robot1" && "$ROBOT" != "robot2" ]]; then
    echo "Usage: $0 [robot1|robot2]"
    echo ""
    echo "Examples:"
    echo "  $0 robot1    # Build robot1 image (run on master node)"
    echo "  $0 robot2    # Build robot2 image (run on worker node)"
    exit 1
fi

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}BARNS Robot Image Builder${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo -e "${GREEN}Building barns-${ROBOT}:latest...${NC}"
echo ""

if docker build --no-cache -t barns-${ROBOT}:latest -f services/robot/Dockerfile.${ROBOT} .; then
    echo ""
    echo -e "${GREEN}âœ“ Image built successfully!${NC}"
else
    echo ""
    echo -e "${YELLOW}âœ— Build failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Importing to containerd...${NC}"

if docker save barns-${ROBOT}:latest | sudo ctr -n k8s.io images import -; then
    echo ""
    echo -e "${GREEN}âœ“ Image imported to containerd${NC}"
else
    echo ""
    echo -e "${YELLOW}âœ— Import failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}âœ“ Done! Image barns-${ROBOT}:latest is ready for Kubernetes${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo "Next steps:"
echo "1. Deploy: kubectl apply -f k8s/services/${ROBOT}-deployment.yaml"
echo "2. Check: kubectl get pods -n barns -l app=${ROBOT}"
echo "3. Logs: kubectl logs -f -n barns -l app=${ROBOT}"
echo ""
