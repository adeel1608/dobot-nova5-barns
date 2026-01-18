#!/bin/bash

# Fix kubelet to use SSD - Working version for systems without 10-kubeadm.conf
# This modifies the main kubelet service to use --root-dir

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "Fix kubelet --root-dir (Alternative Method)"
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

# Show current drop-in files
echo -e "${YELLOW}[i]${NC} Current kubelet drop-in files:"
ls -la /etc/systemd/system/kubelet.service.d/
echo ""

# Check what's in the current drop-in files
echo -e "${YELLOW}[i]${NC} Content of 20-ssd-root.conf:"
cat /etc/systemd/system/kubelet.service.d/20-ssd-root.conf 2>/dev/null || echo "File not found"
echo ""

# Method: Create a drop-in that directly modifies ExecStart
echo -e "${YELLOW}[i]${NC} Creating kubelet override with direct ExecStart modification..."

# Backup existing file if it exists
if [ -f /etc/systemd/system/kubelet.service.d/20-ssd-root.conf ]; then
    cp /etc/systemd/system/kubelet.service.d/20-ssd-root.conf \
       /etc/systemd/system/kubelet.service.d/20-ssd-root.conf.backup.$(date +%s)
fi

# Get the original ExecStart from the main service
ORIGINAL_EXEC=$(grep "^ExecStart=" /lib/systemd/system/kubelet.service | head -1)

if [ -z "$ORIGINAL_EXEC" ]; then
    echo -e "${RED}[✗]${NC} Could not find ExecStart in /lib/systemd/system/kubelet.service"
    exit 1
fi

echo -e "${YELLOW}[i]${NC} Original ExecStart:"
echo "$ORIGINAL_EXEC"
echo ""

# Create new drop-in that overrides ExecStart completely
cat > /etc/systemd/system/kubelet.service.d/10-root-dir.conf <<EOF
[Service]
# Clear the original ExecStart
ExecStart=
# Add our ExecStart with --root-dir
ExecStart=/usr/bin/kubelet --root-dir=$SSD_MOUNT/var/lib/kubelet
EOF

echo -e "${GREEN}[✓]${NC} Created /etc/systemd/system/kubelet.service.d/10-root-dir.conf"
echo ""

# Show the new file
echo -e "${YELLOW}[i]${NC} New drop-in content:"
cat /etc/systemd/system/kubelet.service.d/10-root-dir.conf
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

# Check if kubelet is running
if ! systemctl is-active --quiet kubelet; then
    echo -e "${RED}[✗]${NC} kubelet is not running!"
    echo ""
    echo "Check logs:"
    echo "  sudo journalctl -u kubelet -n 50 --no-pager"
    exit 1
fi

if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} SUCCESS! kubelet is now using --root-dir=$SSD_MOUNT/var/lib/kubelet"
    echo ""
    echo "kubelet process:"
    ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
    echo ""
    echo -e "${GREEN}[✓]${NC} Fix applied successfully!"
else
    echo -e "${RED}[✗]${NC} kubelet started but is NOT using --root-dir flag"
    echo ""
    echo "Current kubelet command:"
    ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1
    echo ""
    echo "This might need the kubeconfig and other flags. Let me create a complete version..."
    
    # Get current kubelet flags
    CURRENT_FLAGS=$(ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | sed 's/.*\/usr\/bin\/kubelet//' | head -1)
    
    echo ""
    echo "Current kubelet flags:"
    echo "$CURRENT_FLAGS"
    echo ""
    
    # Create a more complete drop-in
    cat > /etc/systemd/system/kubelet.service.d/10-root-dir.conf <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/kubelet --root-dir=$SSD_MOUNT/var/lib/kubelet$CURRENT_FLAGS
EOF
    
    echo "Created enhanced drop-in with all current flags"
    systemctl daemon-reload
    systemctl restart kubelet
    sleep 8
    
    if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "root-dir=$SSD_MOUNT"; then
        echo -e "${GREEN}[✓]${NC} SUCCESS on second attempt!"
    else
        echo -e "${RED}[✗]${NC} Still not working. Manual intervention needed."
        echo "Check: sudo journalctl -u kubelet -n 100"
    fi
fi

echo ""
echo "Note: New pod volumes will use SSD going forward"
echo "Check disk usage with: df -h / /mnt/ssd"
echo ""
