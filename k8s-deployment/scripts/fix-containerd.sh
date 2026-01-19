#!/bin/bash

# Quick fix for containerd symlink issue

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "Fixing containerd Storage Links"

# Check if running as root
if ! is_root; then
    print_error "This script must be run with sudo"
    exit 1
fi

SSD_MOUNT="/mnt/ssd"

# Stop containerd
print_info "Stopping containerd..."
systemctl stop containerd || true

# Remove existing directories/symlinks if they exist
print_info "Removing old directories/symlinks..."
for path in /var/lib/kubelet /var/lib/containerd /var/lib/docker /var/lib/etcd; do
    if [ -L "$path" ]; then
        rm -f "$path"
        print_status "Removed symlink: $path"
    elif [ -d "$path" ]; then
        rm -rf "$path"
        print_status "Removed directory: $path"
    fi
done

# Ensure SSD directories exist
print_info "Creating SSD directories..."
mkdir -p "$SSD_MOUNT/var/lib/kubelet"
mkdir -p "$SSD_MOUNT/var/lib/containerd"
mkdir -p "$SSD_MOUNT/var/lib/docker"
mkdir -p "$SSD_MOUNT/var/lib/etcd"
print_status "SSD directories created"

# Create the symlinks
print_info "Creating symlinks..."
ln -sf "$SSD_MOUNT/var/lib/kubelet" /var/lib/kubelet
ln -sf "$SSD_MOUNT/var/lib/containerd" /var/lib/containerd
ln -sf "$SSD_MOUNT/var/lib/docker" /var/lib/docker
ln -sf "$SSD_MOUNT/var/lib/etcd" /var/lib/etcd
print_status "Symlinks created"

# Verify symlinks
print_info "Verifying symlinks..."
for link in /var/lib/kubelet /var/lib/containerd /var/lib/docker /var/lib/etcd; do
    if [ -L "$link" ]; then
        target=$(readlink -f "$link")
        echo "  $link -> $target"
    fi
done

# Start containerd
print_info "Starting containerd..."
systemctl start containerd
sleep 2

# Check status
if systemctl is-active --quiet containerd; then
    print_status "containerd is running"
else
    print_error "containerd failed to start"
    print_info "Checking logs..."
    journalctl -xeu containerd.service -n 20 --no-pager
    exit 1
fi

print_header "Fix Complete!"
echo ""
print_status "containerd is now properly configured"
echo ""
print_info "You can now continue with: sudo ./setup-master.sh"
echo ""
