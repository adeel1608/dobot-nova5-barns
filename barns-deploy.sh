#!/bin/bash
###############################
# BARNS Deployment Menu
# 1 - Update ConfigMaps (tasks.json / recipes.json changes)
# 2 - Build Docker images and push to GCP
###############################

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

log()   { echo -e "${GREEN}[DEPLOY]${NC} $*"; }
info()  { echo -e "${BLUE}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARNING]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
cd "$PROJECT_ROOT"

echo ""
echo "========================================="
echo "  BARNS Deployment"
echo "========================================="
echo ""
echo "  1) Update ConfigMaps  (tasks.json / recipes.json)"
echo "  2) Build & Push Images to GCP"
echo ""
read -rp "Select an option [1 or 2]: " choice

case "$choice" in

    1)
        echo ""
        log "Updating ConfigMaps..."
        echo ""

        if [ ! -x "./k8s/update-configmaps.sh" ]; then
            error "./k8s/update-configmaps.sh not found or not executable"
        fi

        bash k8s/update-configmaps.sh
        ;;

    2)
        echo ""
        log "Building Docker images and pushing to GCP..."
        echo ""

        echo "  a) All service images         (build-images-arm64.sh)"
        echo "  b) Robot 1 only               (build-robot1-image.sh)"
        echo "  c) Robot 2 only               (build-robot2-image.sh)"
        echo "  d) Robot 1 + Robot 2"
        echo "  e) All service images + Robot 1 + Robot 2"
        echo ""
        read -rp "What to build? [a/b/c/d/e]: " build_choice

        NO_CACHE_FLAG=""
        echo ""
        read -rp "Use --no-cache (full rebuild)? [y/N]: " no_cache
        if [[ "$no_cache" =~ ^[Yy]$ ]]; then
            NO_CACHE_FLAG="--no-cache"
        fi

        build_services() {
            log "Building all service images..."
            bash build-images-arm64.sh $NO_CACHE_FLAG
        }

        build_robot1() {
            log "Building Robot 1 image..."
            bash services/robot-arm-k8s/build-robot1-image.sh $NO_CACHE_FLAG
        }

        build_robot2() {
            log "Building Robot 2 image..."
            bash services/robot-arm-k8s/build-robot2-image.sh $NO_CACHE_FLAG
        }

        case "$build_choice" in
            a) build_services ;;
            b) build_robot1 ;;
            c) build_robot2 ;;
            d) build_robot1; build_robot2 ;;
            e) build_services; build_robot1; build_robot2 ;;
            *) error "Invalid choice: $build_choice" ;;
        esac

        echo ""
        read -rp "Push built images to GCP? [Y/n]: " push_choice
        if [[ ! "$push_choice" =~ ^[Nn]$ ]]; then
            log "Pushing images to GCP..."
            bash push-all-to-gcp.sh
        else
            info "Skipping push. Run ./push-all-to-gcp.sh when ready."
        fi
        ;;

    *)
        error "Invalid option: $choice. Please enter 1 or 2."
        ;;
esac

echo ""
log "Done."
