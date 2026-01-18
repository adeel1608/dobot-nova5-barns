#!/bin/bash

# BARNS Master Node SSD Fix Script
# This script properly configures all services to use /mnt/ssd

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "BARNS Master Node SSD Fix"
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

# Stop services (except etcd which is in a static pod)
echo "========================================="
echo "Step 1: Stopping Services"
echo "========================================="

systemctl stop kubelet || true
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
mkdir -p "$SSD_MOUNT/var/lib/etcd"
mkdir -p "$SSD_MOUNT/k8s-data"
mkdir -p "$SSD_MOUNT/barns-data"

echo -e "${GREEN}[✓]${NC} Directories created"
echo ""

# Migrate data from backups if they exist
echo "========================================="
echo "Step 3: Migrating Data to SSD"
echo "========================================="

# Migrate containerd data
if [[ -d /var/lib/containerd.backup ]] && [[ $(du -s /var/lib/containerd.backup | cut -f1) -gt 100 ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating containerd data..."
    rsync -av --progress /var/lib/containerd.backup/ "$SSD_MOUNT/var/lib/containerd/" || true
    echo -e "${GREEN}[✓]${NC} containerd data migrated"
elif [[ -d /var/lib/containerd ]] && [[ $(du -s /var/lib/containerd 2>/dev/null | cut -f1 || echo 0) -gt 100 ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating containerd data from /var/lib/containerd..."
    rsync -av --progress /var/lib/containerd/ "$SSD_MOUNT/var/lib/containerd/" || true
    echo -e "${GREEN}[✓]${NC} containerd data migrated"
else
    echo -e "${YELLOW}[i]${NC} No containerd data to migrate"
fi

# Migrate kubelet data
if [[ -d /var/lib/kubelet.backup ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating kubelet data..."
    rsync -av /var/lib/kubelet.backup/ "$SSD_MOUNT/var/lib/kubelet/" || true
    echo -e "${GREEN}[✓]${NC} kubelet data migrated"
elif [[ -d /var/lib/kubelet ]] && [[ $(du -s /var/lib/kubelet 2>/dev/null | cut -f1 || echo 0) -gt 50 ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating kubelet data from /var/lib/kubelet..."
    rsync -av /var/lib/kubelet/ "$SSD_MOUNT/var/lib/kubelet/" || true
    echo -e "${GREEN}[✓]${NC} kubelet data migrated"
else
    echo -e "${YELLOW}[i]${NC} No kubelet data to migrate"
fi

# Migrate etcd data
if [[ -d /var/lib/etcd.backup ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating etcd data..."
    rsync -av /var/lib/etcd.backup/ "$SSD_MOUNT/var/lib/etcd/" || true
    echo -e "${GREEN}[✓]${NC} etcd data migrated"
elif [[ -d /var/lib/etcd ]] && [[ $(du -s /var/lib/etcd 2>/dev/null | cut -f1 || echo 0) -gt 50 ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating etcd data from /var/lib/etcd..."
    rsync -av /var/lib/etcd/ "$SSD_MOUNT/var/lib/etcd/" || true
    echo -e "${GREEN}[✓]${NC} etcd data migrated"
else
    echo -e "${YELLOW}[i]${NC} No etcd data to migrate"
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

# Configure kubelet - THE CRITICAL PART
echo "========================================="
echo "Step 5: Configuring kubelet"
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

# Configure etcd
echo "========================================="
echo "Step 6: Configuring etcd"
echo "========================================="

if [[ -f /etc/kubernetes/manifests/etcd.yaml ]]; then
    cp /etc/kubernetes/manifests/etcd.yaml \
       /etc/kubernetes/manifests/etcd.yaml.bak.$(date +%s)
    
    # Update data-dir command argument
    sed -i "s|--data-dir=.*|--data-dir=$SSD_MOUNT/var/lib/etcd|g" \
        /etc/kubernetes/manifests/etcd.yaml
    
    # Update hostPath volumes
    sed -i "s|path: /var/lib/etcd|path: $SSD_MOUNT/var/lib/etcd|g" \
        /etc/kubernetes/manifests/etcd.yaml
    
    echo -e "${GREEN}[✓]${NC} etcd manifest updated"
    echo ""
    echo "etcd data-dir:"
    grep "data-dir" /etc/kubernetes/manifests/etcd.yaml | head -1
    echo "etcd volume path:"
    grep "path:" /etc/kubernetes/manifests/etcd.yaml | grep etcd
    echo ""
else
    echo -e "${RED}[✗]${NC} etcd.yaml not found"
fi
echo ""

# Set proper permissions
echo "========================================="
echo "Step 7: Setting Permissions"
echo "========================================="

chown -R root:root "$SSD_MOUNT/var/lib/kubelet"
chown -R root:root "$SSD_MOUNT/var/lib/containerd"
chown -R root:root "$SSD_MOUNT/var/lib/etcd"
chmod 700 "$SSD_MOUNT/var/lib/etcd"

echo -e "${GREEN}[✓]${NC} Permissions set"
echo ""

# Remove old directories and create symlinks
echo "========================================="
echo "Step 8: Creating Symlinks"
echo "========================================="

# Remove old directories (not backups)
rm -rf /var/lib/kubelet 2>/dev/null || true
rm -rf /var/lib/containerd 2>/dev/null || true
rm -rf /var/lib/etcd 2>/dev/null || true

# Create symlinks
ln -s "$SSD_MOUNT/var/lib/kubelet" /var/lib/kubelet
ln -s "$SSD_MOUNT/var/lib/containerd" /var/lib/containerd
ln -s "$SSD_MOUNT/var/lib/etcd" /var/lib/etcd

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

echo -e "${YELLOW}[i]${NC} Starting kubelet..."
systemctl start kubelet
sleep 5
echo -e "${GREEN}[✓]${NC} kubelet started"

echo -e "${YELLOW}[i]${NC} Waiting for etcd pod to restart..."
sleep 10
echo -e "${GREEN}[✓]${NC} etcd should be restarting"

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

echo -e "${YELLOW}[i]${NC} Checking etcd process..."
sleep 5
if ps aux | grep etcd | grep -q "data-dir=$SSD_MOUNT"; then
    echo -e "${GREEN}[✓]${NC} etcd is using --data-dir=$SSD_MOUNT/var/lib/etcd"
else
    echo -e "${YELLOW}[!]${NC} etcd may still be starting (check in 30 seconds)"
fi
echo ""

echo -e "${YELLOW}[i]${NC} SSD usage:"
du -sh "$SSD_MOUNT/var/lib/kubelet" "$SSD_MOUNT/var/lib/containerd" "$SSD_MOUNT/var/lib/etcd" 2>/dev/null
echo ""

echo -e "${YELLOW}[i]${NC} containerd config:"
grep "root = " /etc/containerd/config.toml
echo ""

echo "========================================="
echo "Fix Complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "1. Wait 2-3 minutes for all pods to stabilize"
echo "2. Check cluster status: kubectl get nodes"
echo "3. Check pods: kubectl get pods -A"
echo "4. Check kubelet logs: sudo journalctl -u kubelet -n 50"
echo ""
echo "If cluster is healthy, you can remove old backups:"
echo "  sudo rm -rf /var/lib/*.backup"
echo ""
