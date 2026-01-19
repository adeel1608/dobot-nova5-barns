#!/bin/bash

# BARNS Kubernetes Network Update Script
# This script updates Kubernetes configuration when the network/IP changes

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Network Update Script"

# Check if running as root
if ! is_root; then
    print_error "This script must be run with sudo"
    exit 1
fi

# Detect current IP
NEW_IP=$(detect_ip)
print_info "Detected current IP: $NEW_IP"

# Check if kubeconfig exists
if [ ! -f /etc/kubernetes/admin.conf ]; then
    print_error "Kubernetes not initialized. Run setup-master.sh first."
    exit 1
fi

# Get old IP from kubeconfig
OLD_IP=$(grep "server:" /etc/kubernetes/admin.conf | awk -F'//' '{print $2}' | cut -d':' -f1 | head -n1)
print_info "Old IP from config: $OLD_IP"

if [ "$OLD_IP" == "$NEW_IP" ]; then
    print_status "IP has not changed. No update needed."
    exit 0
fi

echo ""
print_warning "Network IP has changed from $OLD_IP to $NEW_IP"
echo ""
echo "This script will:"
echo "  1. Update kubeconfig files"
echo "  2. Update kubelet configuration"
echo "  3. Restart Kubernetes services"
echo "  4. Update API server certificate"
echo ""

if ! ask_yes_no "Continue with network update?"; then
    print_warning "Update cancelled"
    exit 0
fi

# Step 1: Stop kubelet
print_header "Step 1: Stopping kubelet"
systemctl stop kubelet
print_status "kubelet stopped"

# Step 2: Update kubeconfig files
print_header "Step 2: Updating kubeconfig files"

for config_file in /etc/kubernetes/admin.conf /etc/kubernetes/kubelet.conf /etc/kubernetes/controller-manager.conf /etc/kubernetes/scheduler.conf; do
    if [ -f "$config_file" ]; then
        print_info "Updating $config_file..."
        sed -i "s|https://${OLD_IP}:6443|https://${NEW_IP}:6443|g" "$config_file"
        print_status "Updated $config_file"
    fi
done

# Update user kubeconfig
if [ -f "$HOME/.kube/config" ]; then
    print_info "Updating $HOME/.kube/config..."
    sed -i "s|https://${OLD_IP}:6443|https://${NEW_IP}:6443|g" "$HOME/.kube/config"
    print_status "Updated $HOME/.kube/config"
fi

# Update root kubeconfig
if [ -f /root/.kube/config ]; then
    print_info "Updating /root/.kube/config..."
    sed -i "s|https://${OLD_IP}:6443|https://${NEW_IP}:6443|g" /root/.kube/config
    print_status "Updated /root/.kube/config"
fi

# Step 3: Update kubelet node-ip configuration
print_header "Step 3: Updating kubelet configuration"

# Update kubelet extra args
mkdir -p /etc/systemd/system/kubelet.service.d
cat > /etc/systemd/system/kubelet.service.d/20-node-ip.conf <<EOF
[Service]
Environment="KUBELET_EXTRA_ARGS=--node-ip=${NEW_IP}"
EOF

systemctl daemon-reload
print_status "kubelet configuration updated"

# Step 4: Update API server certificate
print_header "Step 4: Updating API server certificate"

# Backup existing certs
cp -r /etc/kubernetes/pki /etc/kubernetes/pki.backup.$(date +%Y%m%d_%H%M%S)

# Remove old API server cert and key
rm -f /etc/kubernetes/pki/apiserver.crt
rm -f /etc/kubernetes/pki/apiserver.key

# Get cluster configuration
CLUSTER_NAME=$(kubectl config view -o jsonpath='{.clusters[0].name}' 2>/dev/null || echo "kubernetes")
POD_CIDR=$(kubectl get nodes -o jsonpath='{.items[0].spec.podCIDR}' 2>/dev/null || echo "10.244.0.0/16")

# Regenerate API server certificate with new IP
kubeadm init phase certs apiserver --apiserver-advertise-address="${NEW_IP}" --apiserver-cert-extra-sans="${NEW_IP}"

print_status "API server certificate updated"

# Step 5: Update kubeadm ConfigMap
print_header "Step 5: Updating kubeadm ConfigMap"

# Wait for API server to be accessible
print_info "Waiting for API server to be accessible..."
sleep 5

# Update the kubeadm-config ConfigMap
kubectl -n kube-system get cm kubeadm-config -o yaml | \
    sed "s|advertiseAddress:.*|advertiseAddress: ${NEW_IP}|g" | \
    sed "s|bindPort:.*|bindPort: 6443|g" | \
    kubectl apply -f - || print_warning "Could not update kubeadm-config ConfigMap (this may be ok)"

print_status "kubeadm ConfigMap updated"

# Step 6: Restart Kubernetes services
print_header "Step 6: Restarting Kubernetes services"

# Start kubelet
systemctl start kubelet
print_status "kubelet started"

# Wait for API server to be ready
print_info "Waiting for API server to be ready..."
MAX_RETRIES=30
RETRY=0
while [ $RETRY -lt $MAX_RETRIES ]; do
    if kubectl get nodes &> /dev/null; then
        print_status "API server is ready"
        break
    fi
    sleep 2
    RETRY=$((RETRY+1))
done

if [ $RETRY -eq $MAX_RETRIES ]; then
    print_error "API server did not become ready in time"
    print_info "You may need to run: sudo kubeadm init with the new IP"
    exit 1
fi

# Step 7: Update node internal IP
print_header "Step 7: Updating node internal IP"

# Get the node name
NODE_NAME=$(hostname)

# Patch the node to update its internal IP
kubectl patch node "$NODE_NAME" -p "{\"status\":{\"addresses\":[{\"address\":\"${NEW_IP}\",\"type\":\"InternalIP\"},{\"address\":\"${NODE_NAME}\",\"type\":\"Hostname\"}]}}" || print_warning "Could not patch node (this may be ok)"

print_status "Node internal IP updated"

# Completion
print_header "Network Update Complete!"

echo ""
echo "Kubernetes has been updated to use the new IP: $NEW_IP"
echo ""
print_info "To join worker nodes, run: sudo kubeadm token create --print-join-command"
echo ""

# Display cluster info
print_info "Verifying cluster status..."
kubectl get nodes
echo ""

log_message "INFO" "Network update completed successfully"
