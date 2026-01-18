#!/bin/bash

# BARNS Kubernetes Cleanup Script
# This script cleans up Kubernetes installation and optionally SSD data

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Cleanup Script"

# Check if running as root
if ! is_root; then
    print_error "This script must be run with sudo"
    exit 1
fi

# Get SSD mount point
SSD_MOUNT=$(get_config "ssd_mount" "/mnt/ssd")

echo ""
echo "This script will clean up Kubernetes on this node."
echo ""
print_warning "WARNING: This will:"
echo "  - Stop all Kubernetes services"
echo "  - Remove Kubernetes cluster configuration"
echo "  - Clean up network settings"
echo ""

if ! ask_yes_no "Continue with cleanup?"; then
    print_warning "Cleanup cancelled"
    exit 0
fi

# Step 1: Stop kubelet
print_header "Step 1: Stopping Kubernetes Services"
systemctl stop kubelet || true
print_status "kubelet stopped"

# Step 2: Reset kubeadm
print_header "Step 2: Resetting Kubernetes"
kubeadm reset -f || true
print_status "kubeadm reset complete"

# Step 3: Clean up directories
print_header "Step 3: Cleaning up Directories"

# Remove Kubernetes configuration
rm -rf /etc/kubernetes/
rm -rf ~/.kube/
print_status "Kubernetes config removed"

# Remove CNI configuration
rm -rf /etc/cni/net.d/
print_status "CNI config removed"

# Step 4: Clean up network
print_header "Step 4: Cleaning up Network"

# Clean up iptables
iptables -F 2>/dev/null || true
iptables -t nat -F 2>/dev/null || true
iptables -t mangle -F 2>/dev/null || true
iptables -X 2>/dev/null || true
print_status "iptables cleaned"

# Step 5: Ask about SSD data cleanup
print_header "Step 5: SSD Data Cleanup"

echo ""
print_warning "Do you want to clean Kubernetes data from SSD ($SSD_MOUNT)?"
echo "This will remove:"
echo "  - $SSD_MOUNT/var/lib/kubelet"
echo "  - $SSD_MOUNT/var/lib/containerd"
echo "  - $SSD_MOUNT/var/lib/etcd"
echo ""
print_info "Application data ($SSD_MOUNT/barns-data) will NOT be removed"
echo ""

if ask_yes_no "Clean Kubernetes data from SSD?"; then
    # Stop containerd first
    systemctl stop containerd || true
    systemctl stop docker || true
    
    # Clean Kubernetes data
    rm -rf "$SSD_MOUNT/var/lib/kubelet"
    rm -rf "$SSD_MOUNT/var/lib/containerd"
    rm -rf "$SSD_MOUNT/var/lib/etcd"
    
    print_status "Kubernetes data removed from SSD"
    
    # Restart containerd
    systemctl start containerd || true
    systemctl start docker || true
else
    print_info "Keeping Kubernetes data on SSD"
fi

# Step 6: Remove systemd overrides
print_header "Step 6: Cleaning Systemd Configuration"

rm -f /etc/systemd/system/kubelet.service.d/20-ssd-root.conf
rm -f /etc/systemd/system/kubelet.service.d/10-dynamic-ip.conf
systemctl daemon-reload
print_status "Systemd configuration cleaned"

# Step 7: Restart containerd
print_header "Step 7: Restarting Container Runtime"

systemctl restart containerd || true
systemctl restart docker || true
print_status "Container runtime restarted"

# Completion
print_header "Cleanup Complete!"

echo ""
echo "Kubernetes has been cleaned up from this node."
echo ""

if [ -d "$SSD_MOUNT/barns-data" ]; then
    print_info "Application data preserved at: $SSD_MOUNT/barns-data"
fi

echo ""
echo "To reinstall Kubernetes:"
echo "  Master node: sudo ./setup-master.sh"
echo "  Worker node: sudo ./setup-worker.sh"
echo ""

log_message "INFO" "Cleanup completed successfully"
