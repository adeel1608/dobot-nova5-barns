#!/bin/bash

# BARNS Complete Deployment Script
# Main entry point for BARNS Kubernetes deployment

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source common functions
source scripts/common.sh

print_header "BARNS Kubernetes Complete Deployment v${BARNS_DEPLOY_VERSION}"

echo "This script will guide you through the complete BARNS deployment process."
echo ""
echo "Prerequisites:"
echo "  ✓ Master and worker nodes prepared"
echo "  ✓ SSH access to both nodes"
echo "  ✓ Configuration updated in config/cluster-config.yaml"
echo ""

if ! ask_yes_no "Have you completed the prerequisites?"; then
    print_warning "Please complete prerequisites first"
    echo ""
    echo "See QUICKSTART.md for instructions"
    exit 0
fi

# Menu
print_header "Deployment Options"

echo "What would you like to do?"
echo ""
echo "  1) Complete deployment (Setup + Build + Deploy)"
echo "  2) Setup nodes only"
echo "  3) Build images only"
echo "  4) Deploy to Kubernetes only"
echo "  5) Fix database"
echo "  6) Validate deployment"
echo "  7) Cleanup deployment"
echo "  8) Exit"
echo ""

read -p "Enter your choice [1-8]: " choice

case $choice in
    1)
        print_header "Starting Complete Deployment"
        
        print_info "This will:"
        echo "  1. Setup master node (if needed)"
        echo "  2. Setup worker node (if needed)"
        echo "  3. Build Docker images"
        echo "  4. Deploy to Kubernetes"
        echo "  5. Initialize database"
        echo "  6. Validate deployment"
        echo ""
        
        if ! ask_yes_no "Continue?"; then
            exit 0
        fi
        
        # Get node info from config
        MASTER_IP=$(get_config "ip" "")
        MASTER_USER=$(get_config "ssh_user" "")
        WORKER_IP=$(get_config "ip" "")
        WORKER_USER=$(get_config "ssh_user" "")
        
        print_info "Master: ${MASTER_USER}@${MASTER_IP}"
        print_info "Worker: ${WORKER_USER}@${WORKER_IP}"
        
        echo ""
        print_warning "Note: You will need to run setup scripts on each node separately"
        print_info "Or use SSH to run them remotely (requires passwordless SSH)"
        echo ""
        
        if ask_yes_no "Setup master node now? (requires SSH to master)"; then
            print_header "Setting Up Master Node"
            scp -r scripts config "$MASTER_USER@$MASTER_IP:~/k8s-deployment/"
            ssh -t "$MASTER_USER@$MASTER_IP" "cd ~/k8s-deployment/scripts && sudo ./setup-master.sh"
            print_status "Master node setup complete"
        fi
        
        echo ""
        if ask_yes_no "Setup worker node now? (requires SSH to worker)"; then
            print_header "Setting Up Worker Node"
            scp -r scripts config "$WORKER_USER@$WORKER_IP:~/k8s-deployment/"
            ssh -t "$WORKER_USER@$WORKER_IP" "cd ~/k8s-deployment/scripts && sudo ./setup-worker.sh"
            print_status "Worker node setup complete"
        fi
        
        echo ""
        if ask_yes_no "Build images now?"; then
            cd scripts
            ./build-images.sh
            cd ..
        fi
        
        echo ""
        if ask_yes_no "Deploy to Kubernetes now?"; then
            cd scripts
            ./deploy-k8s.sh
            cd ..
        fi
        
        echo ""
        if ask_yes_no "Initialize database now?"; then
            cd scripts
            ./fix-database.sh
            cd ..
        fi
        
        echo ""
        if ask_yes_no "Validate deployment now?"; then
            cd scripts
            ./validate.sh
            cd ..
        fi
        
        print_header "Complete Deployment Finished!"
        ;;
        
    2)
        print_header "Setup Nodes"
        
        echo "Run these commands on respective nodes:"
        echo ""
        echo "Master Node:"
        echo "  cd k8s-deployment/scripts"
        echo "  sudo ./setup-master.sh"
        echo ""
        echo "Worker Node:"
        echo "  cd k8s-deployment/scripts"
        echo "  sudo ./setup-worker.sh"
        echo ""
        ;;
        
    3)
        print_header "Building Images"
        cd scripts
        ./build-images.sh
        ;;
        
    4)
        print_header "Deploying to Kubernetes"
        cd scripts
        ./deploy-k8s.sh
        ;;
        
    5)
        print_header "Fixing Database"
        cd scripts
        ./fix-database.sh
        ;;
        
    6)
        print_header "Validating Deployment"
        cd scripts
        ./validate.sh
        ;;
        
    7)
        print_header "Cleanup Deployment"
        cd scripts
        ./cleanup.sh
        ;;
        
    8)
        print_info "Exiting"
        exit 0
        ;;
        
    *)
        print_error "Invalid choice"
        exit 1
        ;;
esac

echo ""
print_info "For more information, see:"
echo "  - README.md for overview"
echo "  - QUICKSTART.md for quick start guide"
echo "  - docs/TROUBLESHOOTING.md for common issues"
echo ""

