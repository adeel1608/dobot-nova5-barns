#!/bin/bash

# BARNS Kubernetes SSD Usage Verification Script
# This script verifies that all Kubernetes components are using /mnt/ssd for storage

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========================================="
echo "BARNS K8s SSD Usage Verification"
echo "========================================="
echo ""

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}[✗]${NC} This script must be run as root (use sudo)"
   exit 1
fi

echo -e "${YELLOW}[i]${NC} Checking kubelet configuration..."
KUBELET_ROOT=$(ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -o -- '--root-dir=[^ ]*' || echo "NOT SET")
if [[ "$KUBELET_ROOT" == *"/mnt/ssd"* ]]; then
    echo -e "${GREEN}[✓]${NC} kubelet root-dir: $KUBELET_ROOT"
else
    echo -e "${RED}[✗]${NC} kubelet root-dir: $KUBELET_ROOT"
    echo "    Expected: --root-dir=/mnt/ssd/var/lib/kubelet"
fi

echo ""
echo -e "${YELLOW}[i]${NC} Checking kubelet systemd configuration..."
if [[ -f /etc/systemd/system/kubelet.service.d/10-kubeadm.conf ]]; then
    if grep -q "root-dir=/mnt/ssd" /etc/systemd/system/kubelet.service.d/10-kubeadm.conf 2>/dev/null; then
        echo -e "${GREEN}[✓]${NC} kubelet systemd config includes SSD root-dir"
    else
        echo -e "${RED}[✗]${NC} kubelet systemd config missing SSD root-dir"
        echo "    File: /etc/systemd/system/kubelet.service.d/10-kubeadm.conf"
    fi
fi

echo ""
echo -e "${YELLOW}[i]${NC} Checking containerd configuration..."
if [[ -f /etc/containerd/config.toml ]]; then
    CONTAINERD_ROOT=$(grep '^[[:space:]]*root[[:space:]]*=' /etc/containerd/config.toml | head -1 || echo "NOT FOUND")
    if [[ "$CONTAINERD_ROOT" == *"/mnt/ssd"* ]]; then
        echo -e "${GREEN}[✓]${NC} containerd: $CONTAINERD_ROOT"
    else
        echo -e "${RED}[✗]${NC} containerd: $CONTAINERD_ROOT"
        echo "    Expected: root = \"/mnt/ssd/var/lib/containerd\""
    fi
fi

echo ""
echo -e "${YELLOW}[i]${NC} Checking Docker configuration..."
if [[ -f /etc/docker/daemon.json ]]; then
    DOCKER_ROOT=$(grep 'data-root' /etc/docker/daemon.json | grep -o '"/[^"]*"' || echo "NOT FOUND")
    if [[ "$DOCKER_ROOT" == *"/mnt/ssd"* ]]; then
        echo -e "${GREEN}[✓]${NC} docker: data-root = $DOCKER_ROOT"
    else
        echo -e "${RED}[✗]${NC} docker: data-root = $DOCKER_ROOT"
        echo "    Expected: \"data-root\": \"/mnt/ssd/var/lib/docker\""
    fi
else
    echo -e "${YELLOW}[i]${NC} Docker not configured (may not be needed for K8s)"
fi

echo ""
echo -e "${YELLOW}[i]${NC} Checking etcd configuration (master node only)..."
if [[ -f /etc/kubernetes/manifests/etcd.yaml ]]; then
    ETCD_DATA=$(grep 'data-dir' /etc/kubernetes/manifests/etcd.yaml | grep -o '/[^"]*' || echo "NOT FOUND")
    if [[ "$ETCD_DATA" == *"/mnt/ssd"* ]]; then
        echo -e "${GREEN}[✓]${NC} etcd: data-dir = $ETCD_DATA"
    else
        echo -e "${RED}[✗]${NC} etcd: data-dir = $ETCD_DATA"
        echo "    Expected: --data-dir=/mnt/ssd/var/lib/etcd"
    fi
else
    echo -e "${YELLOW}[i]${NC} Not a master node (etcd not found)"
fi

echo ""
echo "========================================="
echo "Disk Usage Analysis"
echo "========================================="
echo ""

echo -e "${YELLOW}[i]${NC} Internal Storage Usage:"
du -sh /var/lib/kubelet 2>/dev/null || echo "  /var/lib/kubelet: Not found"
du -sh /var/lib/containerd 2>/dev/null || echo "  /var/lib/containerd: Not found"
du -sh /var/lib/etcd 2>/dev/null || echo "  /var/lib/etcd: Not found (normal for worker)"
du -sh /var/lib/docker 2>/dev/null || echo "  /var/lib/docker: Not found"

echo ""
echo -e "${YELLOW}[i]${NC} SSD Storage Usage:"
du -sh /mnt/ssd/var/lib/kubelet 2>/dev/null || echo "  /mnt/ssd/var/lib/kubelet: Not found"
du -sh /mnt/ssd/var/lib/containerd 2>/dev/null || echo "  /mnt/ssd/var/lib/containerd: Not found"
du -sh /mnt/ssd/var/lib/etcd 2>/dev/null || echo "  /mnt/ssd/var/lib/etcd: Not found (normal for worker)"
du -sh /mnt/ssd/var/lib/docker 2>/dev/null || echo "  /mnt/ssd/var/lib/docker: Not found"

echo ""
echo "========================================="
echo "Summary"
echo "========================================="
echo ""
echo "If any checks failed (marked with [✗]), run the fix script:"
echo "  sudo ./fix-ssd-storage.sh"
echo ""
