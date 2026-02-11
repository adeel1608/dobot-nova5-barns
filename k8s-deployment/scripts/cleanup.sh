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

# Step 1: Stop kubelet and disable it
print_header "Step 1: Stopping Kubernetes Services"
systemctl stop kubelet || true
systemctl disable kubelet || true
print_status "kubelet stopped and disabled"

# Kill any remaining Kubernetes processes
print_info "Killing remaining Kubernetes processes..."
pkill -9 kube-apiserver 2>/dev/null || true
pkill -9 etcd 2>/dev/null || true
pkill -9 kube-controller-manager 2>/dev/null || true
pkill -9 kube-scheduler 2>/dev/null || true
pkill -9 kube-proxy 2>/dev/null || true
pkill -9 kubelet 2>/dev/null || true
pkill -9 flanneld 2>/dev/null || true
pkill -9 coredns 2>/dev/null || true
print_status "Kubernetes processes killed"

# Step 2: Reset kubeadm
print_header "Step 2: Resetting Kubernetes"
kubeadm reset -f || true
print_status "kubeadm reset complete"

# Step 3: Clean up directories
print_header "Step 3: Cleaning up Directories"

# Remove Kubernetes configuration
rm -rf /etc/kubernetes/
rm -rf ~/.kube/
rm -rf /root/.kube/
# Remove kubeconfig for all users
for user_home in /home/*; do
    if [ -d "$user_home/.kube" ]; then
        rm -rf "$user_home/.kube"
        print_info "Removed kubeconfig for $(basename $user_home)"
    fi
done
print_status "Kubernetes config removed"

# Remove CNI configuration
rm -rf /etc/cni/net.d/
rm -rf /run/flannel/
rm -rf /var/lib/cni/
print_status "CNI config removed"

# Clean up internal storage directories (not SSD)
print_info "Cleaning up internal storage directories..."
rm -rf /var/lib/kubelet 2>/dev/null || true
rm -rf /var/lib/etcd 2>/dev/null || true
print_status "Internal storage directories removed"

# Clean up any backup directories
rm -rf /var/lib/kubelet.backup* 2>/dev/null || true
rm -rf /var/lib/containerd.backup* 2>/dev/null || true
rm -rf /var/lib/etcd.backup* 2>/dev/null || true
rm -rf /var/lib/docker.backup* 2>/dev/null || true
print_status "Backup directories removed"

# Step 4: Clean up network
print_header "Step 4: Cleaning up Network"

# Clean up network interfaces
print_info "Removing CNI network interfaces..."
ip link delete cni0 2>/dev/null || true
ip link delete flannel.1 2>/dev/null || true
ip link delete docker0 2>/dev/null || true
print_status "CNI interfaces removed"

# Clean up iptables
print_info "Cleaning up iptables rules..."
iptables -F 2>/dev/null || true
iptables -t nat -F 2>/dev/null || true
iptables -t mangle -F 2>/dev/null || true
iptables -t filter -F 2>/dev/null || true
iptables -X 2>/dev/null || true
iptables -t nat -X 2>/dev/null || true
iptables -t mangle -X 2>/dev/null || true
iptables -t filter -X 2>/dev/null || true

# Clean up ip6tables too
ip6tables -F 2>/dev/null || true
ip6tables -t nat -F 2>/dev/null || true
ip6tables -t mangle -F 2>/dev/null || true
ip6tables -X 2>/dev/null || true
print_status "iptables/ip6tables cleaned"

# Clean up IPVS rules (if using IPVS)
if command -v ipvsadm &> /dev/null; then
    print_info "Cleaning up IPVS rules..."
    ipvsadm --clear 2>/dev/null || true
    print_status "IPVS rules cleaned"
fi

# Clean up network routes
print_info "Cleaning up routes..."
ip route flush proto bird 2>/dev/null || true
print_status "Routes cleaned"

# Step 5: Ask about SSD data cleanup
print_header "Step 5: SSD Data Cleanup"

echo ""
print_warning "Do you want to clean Kubernetes data from SSD ($SSD_MOUNT)?"
echo "This will remove:"
echo "  - $SSD_MOUNT/var/lib/kubelet"
echo "  - $SSD_MOUNT/var/lib/containerd"
echo "  - $SSD_MOUNT/var/lib/etcd"
echo "  - $SSD_MOUNT/var/lib/docker"
echo ""
print_info "Application data ($SSD_MOUNT/barns-data) will NOT be removed"
echo ""

# Always remove symlinks first (before asking about SSD cleanup)
print_info "Removing symlinks from /var/lib..."
for path in /var/lib/kubelet /var/lib/containerd /var/lib/etcd /var/lib/docker; do
    if [ -L "$path" ]; then
        rm -f "$path"
        print_status "Removed symlink: $path"
    fi
done

if ask_yes_no "Clean Kubernetes data from SSD?"; then
    # Stop services first
    print_info "Stopping container services..."
    systemctl stop kubelet || true
    systemctl stop containerd || true
    systemctl stop docker || true
    sleep 3
    
    # Kill any remaining processes
    pkill -9 containerd || true
    pkill -9 containerd-shim || true
    pkill -9 dockerd || true
    pkill -9 docker-proxy || true
    sleep 2
    
    # Unmount any kubelet mounts
    print_info "Unmounting kubelet volumes..."
    for mount in $(mount | grep "/var/lib/kubelet" | awk '{print $3}'); do
        umount -f "$mount" 2>/dev/null || true
    done
    
    # Clean Kubernetes data from SSD
    rm -rf "$SSD_MOUNT/var/lib/kubelet"
    rm -rf "$SSD_MOUNT/var/lib/containerd"
    rm -rf "$SSD_MOUNT/var/lib/etcd"
    rm -rf "$SSD_MOUNT/var/lib/docker"
    
    print_status "Kubernetes data removed from SSD"
    
    print_info "Note: Container services will be configured during next setup"
else
    print_info "Keeping Kubernetes data on SSD"
    
    # Still clean up mounts
    print_info "Unmounting kubelet volumes..."
    for mount in $(mount | grep "/var/lib/kubelet" | awk '{print $3}'); do
        umount -f "$mount" 2>/dev/null || true
    done
fi

# Step 6: Remove systemd overrides and config backups
print_header "Step 6: Cleaning Systemd Configuration"

rm -f /etc/systemd/system/kubelet.service.d/20-ssd-root.conf
rm -f /etc/systemd/system/kubelet.service.d/10-dynamic-ip.conf
rm -f /etc/systemd/system/kubelet.service.d/10-exec-start.conf
rm -f /etc/systemd/system/kubelet.service.d/20-node-ip.conf
systemctl daemon-reload
print_status "Systemd configuration cleaned"

# Remove kubelet config backups
print_info "Removing kubelet config backups..."
rm -f /var/lib/kubelet/config.yaml.backup* 2>/dev/null || true
print_status "Config backups removed"

print_info "Container runtime will be configured during next setup"
print_info "Do not attempt to start containerd/docker until running setup script"

# Step 8: Kill processes using Kubernetes ports
print_header "Step 8: Cleaning up Port Usage"

print_info "Checking for processes using Kubernetes ports..."
sleep 2

# List of all Kubernetes-related ports
K8S_PORTS=(
    6443    # kube-apiserver
    2379    # etcd client
    2380    # etcd peer
    10250   # kubelet
    10251   # kube-scheduler (old)
    10252   # kube-controller-manager (old)
    10257   # kube-controller-manager
    10259   # kube-scheduler
    8472    # flannel vxlan
    8285    # flannel (alternative)
    10256   # kube-proxy
    9099    # calico (if used)
    30000-32767  # NodePort range (will check sample)
)

# Kill processes using these ports
print_info "Killing processes on Kubernetes ports..."

# Check if lsof is available, if not use fuser
if command -v lsof &> /dev/null; then
    for port in 6443 2379 2380 10250 10251 10252 10257 10259 8472 8285 10256 9099; do
        PIDS=$(lsof -ti :$port 2>/dev/null || true)
        if [ -n "$PIDS" ]; then
            print_info "Killing processes on port $port: $PIDS"
            kill -9 $PIDS 2>/dev/null || true
        fi
    done
elif command -v fuser &> /dev/null; then
    for port in 6443 2379 2380 10250 10251 10252 10257 10259 8472 8285 10256 9099; do
        fuser -k $port/tcp 2>/dev/null || true
    done
else
    print_warning "Neither lsof nor fuser available, skipping port-specific cleanup"
fi

sleep 2

# Verify ports are free
print_info "Verifying ports are free..."
PORTS_IN_USE=$(netstat -tulpn 2>/dev/null | grep -E ':(6443|2379|2380|10250|10251|10252|10257|10259)' || true)

if [ -n "$PORTS_IN_USE" ]; then
    print_warning "Some Kubernetes ports are still in use:"
    echo "$PORTS_IN_USE"
    echo ""
    print_info "Attempting aggressive cleanup..."
    
    # Get PIDs from netstat output and kill them
    PIDS=$(echo "$PORTS_IN_USE" | awk '{print $7}' | cut -d'/' -f1 | sort -u)
    for pid in $PIDS; do
        if [ -n "$pid" ] && [ "$pid" != "-" ]; then
            print_info "Killing PID: $pid"
            kill -9 $pid 2>/dev/null || true
        fi
    done
    
    sleep 3
    
    # Check again
    PORTS_IN_USE=$(netstat -tulpn 2>/dev/null | grep -E ':(6443|2379|2380|10250|10251|10252|10257|10259)' || true)
    if [ -n "$PORTS_IN_USE" ]; then
        print_warning "Some ports still in use. A reboot may be required."
        echo "$PORTS_IN_USE"
    else
        print_status "All Kubernetes ports are now free"
    fi
else
    print_status "All Kubernetes ports are free"
fi

# Step 9: Final Verification
print_header "Step 9: Final Verification"

# Count remaining issues
REMAINING_ISSUES=0

# Check for Kubernetes processes
K8S_PROCS=$(ps aux | grep -E 'kube|etcd|flannel' | grep -v grep | wc -l)
if [ "$K8S_PROCS" -gt 0 ]; then
    print_warning "Warning: $K8S_PROCS Kubernetes processes still running"
    REMAINING_ISSUES=$((REMAINING_ISSUES+1))
else
    print_status "No Kubernetes processes running"
fi

# Check for ports
K8S_PORTS_IN_USE=$(netstat -tulpn 2>/dev/null | grep -E ':(6443|2379|2380|10250)' | wc -l)
if [ "$K8S_PORTS_IN_USE" -gt 0 ]; then
    print_warning "Warning: $K8S_PORTS_IN_USE Kubernetes ports still in use"
    REMAINING_ISSUES=$((REMAINING_ISSUES+1))
else
    print_status "All Kubernetes ports are free"
fi

# Check for network interfaces
CNI_INTERFACES=$(ip link show | grep -E 'cni0|flannel' | wc -l)
if [ "$CNI_INTERFACES" -gt 0 ]; then
    print_warning "Warning: CNI network interfaces still exist"
    REMAINING_ISSUES=$((REMAINING_ISSUES+1))
else
    print_status "No CNI network interfaces"
fi

# Completion
print_header "Cleanup Complete!"

echo ""
if [ "$REMAINING_ISSUES" -eq 0 ]; then
    print_status "Kubernetes has been completely cleaned up from this node."
else
    print_warning "Cleanup completed with $REMAINING_ISSUES warnings (see above)"
    print_info "These may resolve after a reboot, or you can ignore them."
fi
echo ""

if [ -d "$SSD_MOUNT/barns-data" ]; then
    print_info "Application data preserved at: $SSD_MOUNT/barns-data"
fi

echo ""
echo "Cleanup Summary:"
echo "  ✓ Kubernetes processes stopped"
echo "  ✓ Configuration files removed"
echo "  ✓ Network settings cleaned"
echo "  ✓ Systemd configuration removed"
echo "  ✓ Port usage cleared"
echo ""

if [ "$REMAINING_ISSUES" -gt 0 ]; then
    print_warning "If issues persist, consider rebooting: sudo reboot"
    echo ""
fi

echo "To reinstall Kubernetes:"
echo "  1. Fix storage: sudo ./fix-containerd.sh"
echo "  2. Master node: sudo ./setup-master.sh"
echo "     OR"
echo "     Worker node: sudo ./setup-worker.sh"
echo ""

log_message "INFO" "Cleanup completed successfully with $REMAINING_ISSUES warnings"
