#!/bin/bash

# Fix kubelet --root-dir configuration
# Run this on both master and worker nodes if kubelet is not using SSD

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "Fix kubelet --root-dir Configuration"
echo "========================================="
echo ""

if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}[✗]${NC} Must run as root (use sudo)"
   exit 1
fi

# Check if kubelet is already using --root-dir
if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} kubelet is already using --root-dir=$SSD_MOUNT/var/lib/kubelet"
    echo "No fix needed!"
    exit 0
fi

echo -e "${YELLOW}[!]${NC} kubelet is NOT using --root-dir flag"
echo ""

# Check if 10-kubeadm.conf exists
if [ ! -f /etc/systemd/system/kubelet.service.d/10-kubeadm.conf ]; then
    echo -e "${RED}[✗]${NC} /etc/systemd/system/kubelet.service.d/10-kubeadm.conf not found"
    echo "This script should only run after kubeadm init or join"
    exit 1
fi

# Backup the file
echo -e "${YELLOW}[i]${NC} Backing up 10-kubeadm.conf..."
cp /etc/systemd/system/kubelet.service.d/10-kubeadm.conf \
   /etc/systemd/system/kubelet.service.d/10-kubeadm.conf.backup.$(date +%s)

# Show current ExecStart line
echo -e "${YELLOW}[i]${NC} Current ExecStart line:"
grep "^ExecStart" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf || echo "NOT FOUND"
echo ""

# Method 1: Try simple sed
echo -e "${YELLOW}[i]${NC} Attempting to add --root-dir flag..."

# Check if it already has the flag
if grep -q "root-dir=" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf; then
    echo -e "${GREEN}[✓]${NC} Flag already exists in config file"
else
    # Use sed to add the flag
    sed -i '/^ExecStart=\/usr\/bin\/kubelet/s/$/ --root-dir=\/mnt\/ssd\/var\/lib\/kubelet/' \
        /etc/systemd/system/kubelet.service.d/10-kubeadm.conf
    
    # Verify it was added
    if grep -q "root-dir=" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf; then
        echo -e "${GREEN}[✓]${NC} Flag added to config file"
    else
        echo -e "${RED}[✗]${NC} Failed to add flag with sed"
        echo ""
        echo "Please manually edit the file:"
        echo "  sudo nano /etc/systemd/system/kubelet.service.d/10-kubeadm.conf"
        echo ""
        echo "Add this to the end of the ExecStart line:"
        echo "  --root-dir=/mnt/ssd/var/lib/kubelet"
        exit 1
    fi
fi

# Show modified ExecStart line
echo ""
echo -e "${YELLOW}[i]${NC} Modified ExecStart line:"
grep "^ExecStart" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf
echo ""

# Reload systemd
echo -e "${YELLOW}[i]${NC} Reloading systemd daemon..."
systemctl daemon-reload

# Restart kubelet
echo -e "${YELLOW}[i]${NC} Restarting kubelet..."
systemctl restart kubelet

# Wait for kubelet to start
echo -e "${YELLOW}[i]${NC} Waiting for kubelet to start..."
sleep 8

# Verify
echo ""
echo "========================================="
echo "Verification"
echo "========================================="
echo ""

if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} SUCCESS! kubelet is now using --root-dir=$SSD_MOUNT/var/lib/kubelet"
    echo ""
    echo "Full kubelet command:"
    ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
    echo ""
    echo -e "${GREEN}[✓]${NC} Fix applied successfully!"
else
    echo -e "${RED}[✗]${NC} kubelet is still NOT using --root-dir flag"
    echo ""
    echo "Current kubelet command:"
    ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
    echo ""
    echo "Please check:"
    echo "  1. Systemd config: sudo cat /etc/systemd/system/kubelet.service.d/10-kubeadm.conf"
    echo "  2. Kubelet logs: sudo journalctl -u kubelet -n 50"
fi

echo ""
echo "Note: It may take a few minutes for data to migrate to SSD"
echo "Check disk usage with: df -h / /mnt/ssd"
echo ""
