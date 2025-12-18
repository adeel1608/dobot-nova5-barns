#!/bin/bash

# BARNS Kubernetes Master Node Setup Script
# This script sets up a Kubernetes master node with network-agnostic configuration

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Master Node Setup v${BARNS_DEPLOY_VERSION}"

# Check if running as root
if ! is_root; then
    print_error "This script must be run with sudo"
    exit 1
fi

# Check requirements
check_requirements "master"

# Detect node IP
NODE_IP=$(detect_ip)
print_info "Detected node IP: $NODE_IP"

# Get configuration
POD_CIDR=$(get_config "pod_cidr" "10.244.0.0/16")
SERVICE_CIDR=$(get_config "service_cidr" "10.96.0.0/12")
CNI=$(get_config "cni" "flannel")
CLUSTER_NAME=$(get_config "name" "barns-cluster")

echo ""
echo "Configuration:"
echo "  Node IP: $NODE_IP"
echo "  Pod CIDR: $POD_CIDR"
echo "  Service CIDR: $SERVICE_CIDR"
echo "  CNI: $CNI"
echo ""

if ! ask_yes_no "Continue with installation?"; then
    print_warning "Installation cancelled"
    exit 0
fi

# Step 1: Update system
print_header "Step 1: Updating System"
apt-get update -qq
print_status "System updated"

# Step 2: Install prerequisites
print_header "Step 2: Installing Prerequisites"

packages=(
    apt-transport-https
    ca-certificates
    curl
    gnupg
    lsb-release
    software-properties-common
)

for pkg in "${packages[@]}"; do
    if ! dpkg -l | grep -q "^ii  $pkg"; then
        apt-get install -y "$pkg" > /dev/null 2>&1
        print_status "Installed $pkg"
    else
        print_info "$pkg already installed"
    fi
done

# Step 3: Disable swap
print_header "Step 3: Configuring System"

if [ $(swapon --show | wc -l) -gt 0 ]; then
    swapoff -a
    sed -i '/ swap / s/^/#/' /etc/fstab
    print_status "Swap disabled"
else
    print_info "Swap already disabled"
fi

# Enable kernel modules
cat > /etc/modules-load.d/k8s.conf <<EOF
overlay
br_netfilter
EOF

modprobe overlay
modprobe br_netfilter
print_status "Kernel modules loaded"

# Configure sysctl
cat > /etc/sysctl.d/k8s.conf <<EOF
net.bridge.bridge-nf-call-iptables  = 1
net.bridge.bridge-nf-call-ip6tables = 1
net.ipv4.ip_forward                 = 1
EOF

sysctl --system > /dev/null 2>&1
print_status "Sysctl configured"

# Step 4: Install containerd
print_header "Step 4: Installing containerd"

if ! command_exists containerd; then
    apt-get install -y containerd > /dev/null 2>&1
    
    # Configure containerd
    mkdir -p /etc/containerd
    containerd config default > /etc/containerd/config.toml
    sed -i 's/SystemdCgroup = false/SystemdCgroup = true/' /etc/containerd/config.toml
    
    systemctl restart containerd
    systemctl enable containerd > /dev/null 2>&1
    
    print_status "containerd installed and configured"
else
    print_info "containerd already installed"
fi

# Step 5: Install Kubernetes components
print_header "Step 5: Installing Kubernetes Components"

if ! is_k8s_installed; then

    # Create keyrings directory if it doesn't exist
    mkdir -p /etc/apt/keyrings
    
    # Add Kubernetes repo
    print_info "Adding Kubernetes repository..."
    if curl -fsSL https://pkgs.k8s.io/core:/stable:/v1.30/deb/Release.key | gpg --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg; then
        echo 'deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/v1.30/deb/ /' > /etc/apt/sources.list.d/kubernetes.list
        print_status "Kubernetes repository added"
    else
        print_error "Failed to add Kubernetes repository"
        print_error "Check your internet connection"
        exit 1
    fi
    apt-get update -qq
    apt-get install -y kubelet kubeadm kubectl > /dev/null 2>&1
    apt-mark hold kubelet kubeadm kubectl > /dev/null 2>&1
    
    print_status "Kubernetes components installed"
else
    print_info "Kubernetes already installed: $(get_k8s_version)"
fi

# Step 6: Initialize Kubernetes cluster
print_header "Step 6: Initializing Kubernetes Cluster"

if is_node_in_cluster; then
    print_info "Node already in cluster"
    
    if ask_yes_no "Reinitialize cluster? (This will reset everything)"; then
        print_warning "Resetting cluster..."
        kubeadm reset -f > /dev/null 2>&1
        rm -rf /etc/kubernetes
        rm -rf ~/.kube
        print_status "Cluster reset"
    else
        print_info "Skipping cluster initialization"
        # Ensure kubectl config exists
        if [ ! -f ~/.kube/config ]; then
            mkdir -p ~/.kube
            cp /etc/kubernetes/admin.conf ~/.kube/config
            chown $(id -u):$(id -g) ~/.kube/config
        fi
        exit 0
    fi
fi

print_info "Initializing cluster with IP $NODE_IP..."

# Create kubeadm config
cat > /tmp/kubeadm-config.yaml <<EOF
apiVersion: kubeadm.k8s.io/v1beta3
kind: InitConfiguration
localAPIEndpoint:
  advertiseAddress: ${NODE_IP}
  bindPort: 6443
---
apiVersion: kubeadm.k8s.io/v1beta3
kind: ClusterConfiguration
clusterName: ${CLUSTER_NAME}
networking:
  podSubnet: ${POD_CIDR}
  serviceSubnet: ${SERVICE_CIDR}
---
apiVersion: kubelet.config.k8s.io/v1beta1
kind: KubeletConfiguration
cgroupDriver: systemd
EOF

# Initialize cluster
kubeadm init --config=/tmp/kubeadm-config.yaml | tee /tmp/kubeadm-init.log

print_status "Cluster initialized"

# Configure kubectl for root
export KUBECONFIG=/etc/kubernetes/admin.conf
print_status "kubectl configured for root"

# Configure kubectl for regular user
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(eval echo ~$SUDO_USER)
    mkdir -p "$USER_HOME/.kube"
    cp /etc/kubernetes/admin.conf "$USER_HOME/.kube/config"
    chown -R $SUDO_USER:$SUDO_USER "$USER_HOME/.kube"
    print_status "kubectl configured for $SUDO_USER"
fi

# Step 7: Install CNI plugin
print_header "Step 7: Installing CNI Plugin ($CNI)"

export KUBECONFIG=/etc/kubernetes/admin.conf

case "$CNI" in
    flannel)
        kubectl apply -f https://github.com/flannel-io/flannel/releases/latest/download/kube-flannel.yml
        print_status "Flannel CNI installed"
        ;;
    calico)
        kubectl create -f https://raw.githubusercontent.com/projectcalico/calico/v3.26.1/manifests/tigera-operator.yaml
        kubectl create -f https://raw.githubusercontent.com/projectcalico/calico/v3.26.1/manifests/custom-resources.yaml
        print_status "Calico CNI installed"
        ;;
    *)
        print_warning "Unknown CNI: $CNI, skipping installation"
        ;;
esac

# Step 8: Wait for cluster to be ready
print_header "Step 8: Waiting for Cluster to be Ready"

wait_for_condition "kubectl get nodes | grep -q Ready" 120 "Waiting for node to be ready..."

if kubectl get nodes | grep -q Ready; then
    print_status "Master node is ready"
else
    print_error "Master node not ready after timeout"
    kubectl get nodes
    exit 1
fi

# Step 9: Save join command
print_header "Step 9: Generating Worker Join Command"

JOIN_COMMAND=$(kubeadm token create --print-join-command)
echo "$JOIN_COMMAND" > /tmp/k8s-join-command.sh
chmod +x /tmp/k8s-join-command.sh

print_status "Join command saved to /tmp/k8s-join-command.sh"

# Step 10: Configure network reconfiguration
print_header "Step 10: Configuring Network Reconfiguration"

# Create update-network script
cat > /usr/local/bin/k8s-update-network.sh <<'EOF'
#!/bin/bash
# Update Kubernetes network configuration
NEW_IP=$(ip route get 8.8.8.8 | grep -oP 'src \K\S+')
OLD_IP=$(grep "advertise-address" /etc/kubernetes/manifests/kube-apiserver.yaml | awk -F'=' '{print $2}')

if [ "$NEW_IP" != "$OLD_IP" ]; then
    echo "Network change detected: $OLD_IP -> $NEW_IP"
    echo "Updating API server configuration..."
    
    sed -i "s/--advertise-address=${OLD_IP}/--advertise-address=${NEW_IP}/" /etc/kubernetes/manifests/kube-apiserver.yaml
    sed -i "s/advertiseAddress: ${OLD_IP}/advertiseAddress: ${NEW_IP}/" /etc/kubernetes/kubeadm-config.yaml
    
    echo "Restarting kubelet..."
    systemctl restart kubelet
    
    echo "Network configuration updated"
fi
EOF

chmod +x /usr/local/bin/k8s-update-network.sh
print_status "Network reconfiguration script installed"

# Completion
print_header "Master Node Setup Complete!"

echo ""
echo "Cluster Information:"
echo "  Master IP: $NODE_IP"
echo "  API Server: https://${NODE_IP}:6443"
echo ""
echo "Next Steps:"
echo ""
echo "1. Copy the join command to your worker node:"
echo "   scp /tmp/k8s-join-command.sh user@worker-node:/tmp/"
echo ""
echo "2. On worker node, run:"
echo "   sudo bash /tmp/k8s-join-command.sh"
echo ""
echo "3. Verify cluster status:"
echo "   kubectl get nodes"
echo ""
echo "Join Command:"
echo "---"
cat /tmp/k8s-join-command.sh
echo "---"
echo ""

print_info "If network changes, run: sudo /usr/local/bin/k8s-update-network.sh"

# Save configuration
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(eval echo ~$SUDO_USER)
    cat > "$USER_HOME/k8s-master-info.txt" <<EOF
Master Node Information
=======================
Node IP: $NODE_IP
API Server: https://${NODE_IP}:6443
Setup Date: $(date)
Pod CIDR: $POD_CIDR
Service CIDR: $SERVICE_CIDR
CNI: $CNI

Join Command:
$JOIN_COMMAND
EOF
    chown $SUDO_USER:$SUDO_USER "$USER_HOME/k8s-master-info.txt"
    print_status "Configuration saved to $USER_HOME/k8s-master-info.txt"
fi

log_message "INFO" "Master node setup completed successfully"

