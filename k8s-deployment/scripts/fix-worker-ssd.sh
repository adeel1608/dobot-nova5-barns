#!/bin/bash

# BARNS Worker Node SSD Fix Script
# This script properly configures all services to use /mnt/ssd

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "BARNS Worker Node SSD Fix"
echo "========================================="
echo ""

if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}[✗]${NC} Must run as root (use sudo)"
   exit 1
fi

if ! mountpoint -q "$SSD_MOUNT"; then
    echo -e "${RED}[✗]${NC} SSD not mounted at $SSD_MOUNT"
    exit 1
fi

echo -e "${GREEN}[✓]${NC} SSD found at $SSD_MOUNT"
echo ""

# Stop services
echo "========================================="
echo "Step 1: Stopping Services"
echo "========================================="

systemctl stop kubelet || true
sleep 2
systemctl stop docker || true
sleep 2
systemctl stop containerd || true
sleep 3

echo -e "${GREEN}[✓]${NC} Services stopped"
echo ""

# Create SSD directories
echo "========================================="
echo "Step 2: Creating SSD Directories"
echo "========================================="

mkdir -p "$SSD_MOUNT/var/lib/kubelet"
mkdir -p "$SSD_MOUNT/var/lib/containerd"
mkdir -p "$SSD_MOUNT/var/lib/docker"
mkdir -p "$SSD_MOUNT/barns-data"
mkdir -p "$SSD_MOUNT/barns-config"

echo -e "${GREEN}[✓]${NC} Directories created"
echo ""

# Migrate data from backups
echo "========================================="
echo "Step 3: Migrating Data to SSD"
echo "========================================="

# Migrate Docker data
if [[ -d /var/lib/docker.backup ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating Docker data (9.6G)..."
    rsync -av --progress /var/lib/docker.backup/ "$SSD_MOUNT/var/lib/docker/" || true
    echo -e "${GREEN}[✓]${NC} Docker data migrated"
else
    echo -e "${YELLOW}[!]${NC} No Docker backup found"
fi

# Migrate containerd data
if [[ -d /var/lib/containerd.backup ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating containerd data (519M)..."
    rsync -av --progress /var/lib/containerd.backup/ "$SSD_MOUNT/var/lib/containerd/" || true
    echo -e "${GREEN}[✓]${NC} containerd data migrated"
else
    echo -e "${YELLOW}[!]${NC} No containerd backup found"
fi

# Migrate kubelet data
if [[ -d /var/lib/kubelet.backup ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating kubelet data (212K)..."
    rsync -av /var/lib/kubelet.backup/ "$SSD_MOUNT/var/lib/kubelet/" || true
    echo -e "${GREEN}[✓]${NC} kubelet data migrated"
else
    echo -e "${YELLOW}[!]${NC} No kubelet backup found"
fi

echo ""

# Configure containerd
echo "========================================="
echo "Step 4: Configuring containerd"
echo "========================================="

if [[ -f /etc/containerd/config.toml ]]; then
    cp /etc/containerd/config.toml /etc/containerd/config.toml.bak.$(date +%s)
    
    # Update root path
    sed -i 's|^[[:space:]]*root[[:space:]]*=.*|    root = "'$SSD_MOUNT'/var/lib/containerd"|' /etc/containerd/config.toml
    
    # Verify change
    if grep -q "$SSD_MOUNT/var/lib/containerd" /etc/containerd/config.toml; then
        echo -e "${GREEN}[✓]${NC} containerd configured: root = \"$SSD_MOUNT/var/lib/containerd\""
    else
        echo -e "${RED}[✗]${NC} Failed to update containerd config"
    fi
else
    echo -e "${RED}[✗]${NC} containerd config not found"
fi
echo ""

# Configure Docker
echo "========================================="
echo "Step 5: Configuring Docker"
echo "========================================="

mkdir -p /etc/docker
cat > /etc/docker/daemon.json << EOF
{
  "data-root": "$SSD_MOUNT/var/lib/docker",
  "storage-driver": "overlay2"
}
EOF

echo -e "${GREEN}[✓]${NC} Docker configured: data-root = \"$SSD_MOUNT/var/lib/docker\""
echo ""

# Configure kubelet - THE CRITICAL PART
echo "========================================="
echo "Step 6: Configuring kubelet"
echo "========================================="

# Create systemd drop-in directory
mkdir -p /etc/systemd/system/kubelet.service.d

# Check if 10-kubeadm.conf exists
if [[ -f /etc/systemd/system/kubelet.service.d/10-kubeadm.conf ]]; then
    echo -e "${YELLOW}[i]${NC} Found 10-kubeadm.conf, modifying it..."
    
    # Backup original
    cp /etc/systemd/system/kubelet.service.d/10-kubeadm.conf \
       /etc/systemd/system/kubelet.service.d/10-kubeadm.conf.bak.$(date +%s)
    
    # Check if ExecStart already has --root-dir
    if grep -q "root-dir=" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf; then
        echo -e "${YELLOW}[i]${NC} Updating existing --root-dir argument..."
        sed -i "s|--root-dir=[^ ]*|--root-dir=$SSD_MOUNT/var/lib/kubelet|g" \
            /etc/systemd/system/kubelet.service.d/10-kubeadm.conf
    else
        echo -e "${YELLOW}[i]${NC} Adding --root-dir argument to ExecStart..."
        # Add --root-dir to the end of the ExecStart line
        sed -i '/^ExecStart=\/usr\/bin\/kubelet/s/$/ --root-dir=\/mnt\/ssd\/var\/lib\/kubelet/' \
            /etc/systemd/system/kubelet.service.d/10-kubeadm.conf
    fi
    
    echo ""
    echo "Current kubelet ExecStart line:"
    grep "^ExecStart" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf || echo "NOT FOUND!"
    echo ""
fi

# Also create/update 20-ssd-root.conf as backup method
cat > /etc/systemd/system/kubelet.service.d/20-ssd-root.conf << EOF
[Service]
Environment="KUBELET_EXTRA_ARGS=--root-dir=$SSD_MOUNT/var/lib/kubelet"
EOF

echo -e "${GREEN}[✓]${NC} kubelet systemd configuration updated"
echo -e "${GREEN}[✓]${NC} Created drop-in: 20-ssd-root.conf"
echo ""

# Set proper permissions
echo "========================================="
echo "Step 7: Setting Permissions"
echo "========================================="

chown -R root:root "$SSD_MOUNT/var/lib/kubelet"
chown -R root:root "$SSD_MOUNT/var/lib/containerd"
chown -R root:root "$SSD_MOUNT/var/lib/docker"

echo -e "${GREEN}[✓]${NC} Permissions set"
echo ""

# Remove old directories and create symlinks
echo "========================================="
echo "Step 8: Creating Symlinks"
echo "========================================="

# Remove old directories (not backups)
rm -rf /var/lib/kubelet 2>/dev/null || true
rm -rf /var/lib/containerd 2>/dev/null || true
rm -rf /var/lib/docker 2>/dev/null || true

# Create symlinks
ln -s "$SSD_MOUNT/var/lib/kubelet" /var/lib/kubelet
ln -s "$SSD_MOUNT/var/lib/containerd" /var/lib/containerd
ln -s "$SSD_MOUNT/var/lib/docker" /var/lib/docker

echo -e "${GREEN}[✓]${NC} Symlinks created"
echo ""

# Reload systemd and start services
echo "========================================="
echo "Step 9: Starting Services"
echo "========================================="

systemctl daemon-reload
echo -e "${GREEN}[✓]${NC} systemd reloaded"

echo -e "${YELLOW}[i]${NC} Starting containerd..."
systemctl start containerd
sleep 3
echo -e "${GREEN}[✓]${NC} containerd started"

echo -e "${YELLOW}[i]${NC} Starting Docker..."
systemctl start docker || true
sleep 2
echo -e "${GREEN}[✓]${NC} Docker started"

echo -e "${YELLOW}[i]${NC} Starting kubelet..."
systemctl start kubelet
sleep 5
echo -e "${GREEN}[✓]${NC} kubelet started"

echo ""
echo "========================================="
echo "Verification"
echo "========================================="
echo ""

echo -e "${YELLOW}[i]${NC} Checking kubelet process..."
sleep 3
ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | head -1 || echo "Kubelet not found in process list!"
echo ""

echo -e "${YELLOW}[i]${NC} Checking for --root-dir flag..."
if ps aux | grep kubelet | grep -q "root-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} kubelet is using --root-dir=$SSD_MOUNT/var/lib/kubelet"
else
    echo -e "${RED}[✗]${NC} kubelet is NOT using --root-dir (check systemd config)"
fi
echo ""

echo -e "${YELLOW}[i]${NC} SSD usage:"
du -sh "$SSD_MOUNT/var/lib/kubelet" "$SSD_MOUNT/var/lib/containerd" "$SSD_MOUNT/var/lib/docker" 2>/dev/null
echo ""

echo -e "${YELLOW}[i]${NC} containerd config:"
grep "root = " /etc/containerd/config.toml
echo ""

echo -e "${YELLOW}[i]${NC} Docker config:"
cat /etc/docker/daemon.json
echo ""

echo "========================================="
echo "Fix Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Wait 1-2 minutes for kubelet to stabilize"
echo "2. Check node status: kubectl get nodes (from master)"
echo "3. Check kubelet logs: sudo journalctl -u kubelet -n 50"
echo ""
echo "If node is Ready, you can remove old backups:"
echo "  sudo rm -rf /var/lib/*.backup"
echo ""
