#!/bin/bash

# BARNS Kubernetes SSD Storage Fix Script
# This script fixes the storage configuration to use /mnt/ssd

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SSD_MOUNT="/mnt/ssd"

echo "========================================="
echo "BARNS K8s SSD Storage Fix"
echo "========================================="
echo ""

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}[✗]${NC} This script must be run as root (use sudo)"
   exit 1
fi

# Check if SSD is mounted
if ! mountpoint -q "$SSD_MOUNT"; then
    echo -e "${RED}[✗]${NC} SSD is not mounted at $SSD_MOUNT"
    exit 1
fi

echo -e "${GREEN}[✓]${NC} SSD found at $SSD_MOUNT"
echo ""

# Detect node type
IS_MASTER=false
if [[ -f /etc/kubernetes/manifests/etcd.yaml ]]; then
    IS_MASTER=true
    echo -e "${BLUE}[i]${NC} Detected: MASTER node"
else
    echo -e "${BLUE}[i]${NC} Detected: WORKER node"
fi
echo ""

# Create necessary directories
echo "========================================="
echo "Step 1: Creating SSD Directories"
echo "========================================="
mkdir -p "$SSD_MOUNT/var/lib/kubelet"
mkdir -p "$SSD_MOUNT/var/lib/containerd"
mkdir -p "$SSD_MOUNT/var/lib/docker"
mkdir -p "$SSD_MOUNT/barns-data"
mkdir -p "$SSD_MOUNT/barns-config"

if [[ "$IS_MASTER" == true ]]; then
    mkdir -p "$SSD_MOUNT/var/lib/etcd"
    mkdir -p "$SSD_MOUNT/k8s-data"
fi

echo -e "${GREEN}[✓]${NC} Directories created"
echo ""

# Fix containerd configuration
echo "========================================="
echo "Step 2: Configuring containerd"
echo "========================================="

if [[ -f /etc/containerd/config.toml ]]; then
    # Backup original
    cp /etc/containerd/config.toml /etc/containerd/config.toml.backup.$(date +%s)
    
    # Update root path
    if grep -q '^[[:space:]]*root[[:space:]]*=' /etc/containerd/config.toml; then
        sed -i 's|^[[:space:]]*root[[:space:]]*=.*|    root = "'$SSD_MOUNT'/var/lib/containerd"|' /etc/containerd/config.toml
        echo -e "${GREEN}[✓]${NC} containerd config updated"
    else
        echo -e "${YELLOW}[!]${NC} Could not find root directive in containerd config"
    fi
    
    # Migrate existing data if needed
    if [[ -d /var/lib/containerd ]] && [[ $(du -s /var/lib/containerd | cut -f1) -gt 100 ]]; then
        echo -e "${YELLOW}[i]${NC} Migrating containerd data to SSD..."
        systemctl stop containerd || true
        rsync -av /var/lib/containerd/ "$SSD_MOUNT/var/lib/containerd/" || true
        mv /var/lib/containerd /var/lib/containerd.backup.$(date +%s)
        systemctl start containerd
        echo -e "${GREEN}[✓]${NC} containerd data migrated"
    fi
else
    echo -e "${RED}[✗]${NC} containerd config not found"
fi
echo ""

# Fix Docker configuration (if Docker is used)
echo "========================================="
echo "Step 3: Configuring Docker"
echo "========================================="

if command -v docker &> /dev/null; then
    mkdir -p /etc/docker
    
    # Create or update daemon.json
    cat > /etc/docker/daemon.json << EOF
{
  "data-root": "$SSD_MOUNT/var/lib/docker",
  "storage-driver": "overlay2"
}
EOF
    
    echo -e "${GREEN}[✓]${NC} Docker config updated"
    
    # Migrate existing data if needed
    if [[ -d /var/lib/docker ]] && [[ $(du -s /var/lib/docker 2>/dev/null | cut -f1 || echo 0) -gt 100 ]]; then
        echo -e "${YELLOW}[i]${NC} Migrating Docker data to SSD..."
        systemctl stop docker || true
        rsync -av /var/lib/docker/ "$SSD_MOUNT/var/lib/docker/" || true
        mv /var/lib/docker /var/lib/docker.backup.$(date +%s)
        systemctl start docker
        echo -e "${GREEN}[✓]${NC} Docker data migrated"
    fi
else
    echo -e "${YELLOW}[i]${NC} Docker not installed (skipping)"
fi
echo ""

# Fix kubelet configuration
echo "========================================="
echo "Step 4: Configuring kubelet"
echo "========================================="

# Create systemd drop-in directory
mkdir -p /etc/systemd/system/kubelet.service.d

# Modify the 10-kubeadm.conf file to include --root-dir
if [[ -f /etc/systemd/system/kubelet.service.d/10-kubeadm.conf ]]; then
    cp /etc/systemd/system/kubelet.service.d/10-kubeadm.conf \
       /etc/systemd/system/kubelet.service.d/10-kubeadm.conf.backup.$(date +%s)
    
    # Check if root-dir is already in the ExecStart line
    if ! grep -q "root-dir=" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf; then
        # Add --root-dir to the kubelet command
        sed -i '/^ExecStart=\/usr\/bin\/kubelet/s/$/ --root-dir=\/mnt\/ssd\/var\/lib\/kubelet/' \
            /etc/systemd/system/kubelet.service.d/10-kubeadm.conf
        echo -e "${GREEN}[✓]${NC} kubelet systemd config updated with --root-dir"
    else
        echo -e "${YELLOW}[i]${NC} kubelet already has --root-dir configured"
    fi
else
    echo -e "${YELLOW}[!]${NC} 10-kubeadm.conf not found, creating new drop-in file"
    cat > /etc/systemd/system/kubelet.service.d/20-ssd-root.conf << EOF
[Service]
Environment="KUBELET_EXTRA_ARGS=--root-dir=$SSD_MOUNT/var/lib/kubelet"
EOF
fi

# Migrate kubelet data if needed
if [[ -d /var/lib/kubelet ]] && [[ $(du -s /var/lib/kubelet | cut -f1) -gt 50 ]]; then
    echo -e "${YELLOW}[i]${NC} Migrating kubelet data to SSD..."
    systemctl stop kubelet || true
    rsync -av /var/lib/kubelet/ "$SSD_MOUNT/var/lib/kubelet/" || true
    mv /var/lib/kubelet /var/lib/kubelet.backup.$(date +%s)
    ln -s "$SSD_MOUNT/var/lib/kubelet" /var/lib/kubelet
    echo -e "${GREEN}[✓]${NC} kubelet data migrated"
fi

echo -e "${GREEN}[✓]${NC} kubelet configured"
echo ""

# Fix etcd configuration (master only)
if [[ "$IS_MASTER" == true ]]; then
    echo "========================================="
    echo "Step 5: Configuring etcd (Master Only)"
    echo "========================================="
    
    if [[ -f /etc/kubernetes/manifests/etcd.yaml ]]; then
        cp /etc/kubernetes/manifests/etcd.yaml \
           /etc/kubernetes/manifests/etcd.yaml.backup.$(date +%s)
        
        # Update data-dir in etcd.yaml
        sed -i "s|--data-dir=.*|--data-dir=$SSD_MOUNT/var/lib/etcd|g" \
            /etc/kubernetes/manifests/etcd.yaml
        
        # Update hostPath volumes
        sed -i "s|path: /var/lib/etcd|path: $SSD_MOUNT/var/lib/etcd|g" \
            /etc/kubernetes/manifests/etcd.yaml
        
        echo -e "${GREEN}[✓]${NC} etcd config updated"
        
        # Migrate etcd data if needed
        if [[ -d /var/lib/etcd ]] && [[ $(du -s /var/lib/etcd 2>/dev/null | cut -f1 || echo 0) -gt 50 ]]; then
            echo -e "${YELLOW}[i]${NC} Migrating etcd data to SSD..."
            rsync -av /var/lib/etcd/ "$SSD_MOUNT/var/lib/etcd/" || true
            mv /var/lib/etcd /var/lib/etcd.backup.$(date +%s)
            echo -e "${GREEN}[✓]${NC} etcd data migrated"
            echo -e "${YELLOW}[!]${NC} etcd pod will restart automatically"
        fi
    else
        echo -e "${YELLOW}[!]${NC} etcd.yaml not found"
    fi
    echo ""
fi

# Reload and restart services
echo "========================================="
echo "Step 6: Restarting Services"
echo "========================================="

systemctl daemon-reload
echo -e "${GREEN}[✓]${NC} systemd reloaded"

# Restart containerd
echo -e "${YELLOW}[i]${NC} Restarting containerd..."
systemctl restart containerd
sleep 3
echo -e "${GREEN}[✓]${NC} containerd restarted"

# Restart Docker if installed
if command -v docker &> /dev/null; then
    echo -e "${YELLOW}[i]${NC} Restarting Docker..."
    systemctl restart docker || true
    sleep 2
    echo -e "${GREEN}[✓]${NC} Docker restarted"
fi

# Restart kubelet
echo -e "${YELLOW}[i]${NC} Restarting kubelet..."
systemctl restart kubelet
sleep 5
echo -e "${GREEN}[✓]${NC} kubelet restarted"

echo ""
echo "========================================="
echo "Fix Complete!"
echo "========================================="
echo ""
echo -e "${GREEN}[✓]${NC} All services configured to use $SSD_MOUNT"
echo ""
echo "Next steps:"
echo "1. Wait 1-2 minutes for services to stabilize"
echo "2. Run verification: sudo ./verify-ssd-usage.sh"
echo "3. Check node status: kubectl get nodes"
echo ""
echo "Note: Old data has been backed up with timestamp suffix"
echo "      You can remove backups after verifying everything works:"
echo "      sudo rm -rf /var/lib/*.backup.*"
echo ""
