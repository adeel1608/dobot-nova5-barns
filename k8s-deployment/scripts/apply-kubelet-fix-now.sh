#!/bin/bash

# Apply kubelet SSD fix to running cluster
# This script fixes the current cluster without tearing it down

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "Apply kubelet SSD Fix to Running Cluster"
echo "========================================="
echo ""

if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}[✗]${NC} Must run as root (use sudo)"
   exit 1
fi

# Detect node IP
NODE_IP=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K\S+' || hostname -I | awk '{print $1}')
echo -e "${BLUE}[i]${NC} Detected node IP: $NODE_IP"
echo ""

# Check if kubelet is already using --root-dir
if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} kubelet is already using --root-dir=$SSD_MOUNT/var/lib/kubelet"
    echo "No fix needed!"
    exit 0
fi

echo -e "${YELLOW}[!]${NC} kubelet is NOT using --root-dir flag"
echo ""

# Show current kubelet command
echo -e "${YELLOW}[i]${NC} Current kubelet command:"
ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
echo ""

# Backup existing drop-in files
echo -e "${YELLOW}[i]${NC} Backing up existing kubelet drop-in files..."
BACKUP_DIR="/etc/systemd/system/kubelet.service.d/backup-$(date +%s)"
mkdir -p "$BACKUP_DIR"

for file in /etc/systemd/system/kubelet.service.d/*.conf; do
    if [ -f "$file" ]; then
        cp "$file" "$BACKUP_DIR/"
    fi
done
echo -e "${GREEN}[✓]${NC} Backups saved to $BACKUP_DIR"
echo ""

# Create new drop-in with proper ExecStart override
echo -e "${YELLOW}[i]${NC} Creating kubelet ExecStart override..."

cat > /etc/systemd/system/kubelet.service.d/10-exec-start.conf <<EOF
[Service]
# Clear any previous ExecStart
ExecStart=
# Set ExecStart with SSD root-dir and node-ip
ExecStart=/usr/bin/kubelet --root-dir=$SSD_MOUNT/var/lib/kubelet --node-ip=$NODE_IP
EOF

echo -e "${GREEN}[✓]${NC} Created /etc/systemd/system/kubelet.service.d/10-exec-start.conf"
echo ""

# Show the new configuration
echo -e "${YELLOW}[i]${NC} New kubelet configuration:"
cat /etc/systemd/system/kubelet.service.d/10-exec-start.conf
echo ""

# Reload systemd
echo -e "${YELLOW}[i]${NC} Reloading systemd daemon..."
systemctl daemon-reload
echo -e "${GREEN}[✓]${NC} Systemd reloaded"
echo ""

# Restart kubelet
echo -e "${YELLOW}[i]${NC} Restarting kubelet..."
systemctl restart kubelet

# Wait for kubelet to start
echo -e "${YELLOW}[i]${NC} Waiting for kubelet to stabilize..."
sleep 8

# Check if kubelet is running
if ! systemctl is-active --quiet kubelet; then
    echo -e "${RED}[✗]${NC} kubelet failed to start!"
    echo ""
    echo "Check logs:"
    echo "  sudo journalctl -u kubelet -n 50 --no-pager"
    echo ""
    echo "To restore backup:"
    echo "  sudo cp $BACKUP_DIR/*.conf /etc/systemd/system/kubelet.service.d/"
    echo "  sudo systemctl daemon-reload"
    echo "  sudo systemctl restart kubelet"
    exit 1
fi

echo -e "${GREEN}[✓]${NC} kubelet restarted successfully"
echo ""

# Verify the fix
echo "========================================="
echo "Verification"
echo "========================================="
echo ""

# Check for --root-dir flag
if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} kubelet is using --root-dir=$SSD_MOUNT/var/lib/kubelet"
else
    echo -e "${RED}[✗]${NC} kubelet is NOT using --root-dir flag"
fi

# Check for --node-ip flag
if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "node-ip=$NODE_IP"; then
    echo -e "${GREEN}[✓]${NC} kubelet is using --node-ip=$NODE_IP"
else
    echo -e "${YELLOW}[!]${NC} kubelet is NOT using --node-ip flag"
fi

echo ""
echo -e "${YELLOW}[i]${NC} New kubelet command:"
ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
echo ""

echo "========================================="
echo "Fix Applied Successfully!"
echo "========================================="
echo ""

echo -e "${GREEN}[✓]${NC} kubelet is now configured to use SSD storage"
echo ""
echo "Next steps:"
echo "  1. Wait 2-3 minutes for node to stabilize"
echo "  2. Check node status: kubectl get nodes"
echo "  3. New pod volumes will use SSD going forward"
echo ""
echo "Disk usage will change over time as pods restart:"
echo "  df -h / /mnt/ssd"
echo ""
echo "Backups saved at: $BACKUP_DIR"
echo ""
