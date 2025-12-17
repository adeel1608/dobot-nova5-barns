#!/bin/bash

# BARNS Kubernetes Worker Node Setup Script
# This script sets up a Kubernetes worker node with Docker and containerd

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Worker Node Setup v${BARNS_DEPLOY_VERSION}"

# Check if running as root
if ! is_root; then
    print_error "This script must be run with sudo"
    exit 1
fi

# Check requirements
check_requirements "worker"

# Detect node IP
NODE_IP=$(detect_ip)
print_info "Detected node IP: $NODE_IP"

# Get configuration
STORAGE_BASE=$(get_config "base_path" "/mnt/barns-data")
CONFIG_PATH=$(get_config "path" "/mnt/barns-config")

echo ""
echo "Configuration:"
echo "  Node IP: $NODE_IP"
echo "  Storage Base: $STORAGE_BASE"
echo "  Config Path: $CONFIG_PATH"
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
    git
    dos2unix
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

# Step 4: Install Docker
print_header "Step 4: Installing Docker"

if ! is_docker_installed; then
    # Add Docker repo
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" > /etc/apt/sources.list.d/docker.list
    
    apt-get update -qq
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin > /dev/null 2>&1
    
    # Configure Docker daemon
    cat > /etc/docker/daemon.json <<EOF
{
    "exec-opts": ["native.cgroupdriver=systemd"],
    "log-driver": "json-file",
    "log-opts": {
        "max-size": "100m",
        "max-file": "3"
    },
    "storage-driver": "overlay2",
    "dns": ["8.8.8.8", "8.8.4.4", "1.1.1.1"]
}
EOF
    
    systemctl daemon-reload
    systemctl restart docker
    systemctl enable docker > /dev/null 2>&1
    
    print_status "Docker installed and configured"
else
    print_info "Docker already installed"
    
    # Ensure DNS configuration
    if [ -f /etc/docker/daemon.json ]; then
        if ! grep -q '"dns"' /etc/docker/daemon.json; then
            print_warning "Updating Docker DNS configuration..."
            backup_file /etc/docker/daemon.json
            jq '. + {"dns": ["8.8.8.8", "8.8.4.4", "1.1.1.1"]}' /etc/docker/daemon.json > /tmp/daemon.json
            mv /tmp/daemon.json /etc/docker/daemon.json
            systemctl restart docker
            print_status "Docker DNS configured"
        fi
    fi
fi

# Add user to docker group
if [ -n "$SUDO_USER" ]; then
    usermod -aG docker $SUDO_USER
    print_status "Added $SUDO_USER to docker group"
fi

# Step 5: Install containerd
print_header "Step 5: Configuring containerd"

# Configure containerd for Kubernetes
mkdir -p /etc/containerd
containerd config default > /etc/containerd/config.toml
sed -i 's/SystemdCgroup = false/SystemdCgroup = true/' /etc/containerd/config.toml

systemctl restart containerd
systemctl enable containerd > /dev/null 2>&1

print_status "containerd configured"

# Step 6: Install Kubernetes components
print_header "Step 6: Installing Kubernetes Components"

if ! is_k8s_installed; then
    # Add Kubernetes repo
    curl -fsSL https://pkgs.k8s.io/core:/stable:/v1.30/deb/Release.key | gpg --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg
    echo 'deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/v1.30/deb/ /' > /etc/apt/sources.list.d/kubernetes.list
    
    apt-get update -qq
    apt-get install -y kubelet kubeadm kubectl > /dev/null 2>&1
    apt-mark hold kubelet kubeadm kubectl > /dev/null 2>&1
    
    print_status "Kubernetes components installed"
else
    print_info "Kubernetes already installed: $(get_k8s_version)"
fi

# Step 7: Create storage directories
print_header "Step 7: Creating Storage Directories"

directories=(
    "$STORAGE_BASE"
    "$STORAGE_BASE/postgres"
    "$STORAGE_BASE/postgres/pgdata"
    "$STORAGE_BASE/rabbitmq"
    "$STORAGE_BASE/influxdb"
    "$STORAGE_BASE/redis"
    "$STORAGE_BASE/cup_models"
    "$CONFIG_PATH"
)

for dir in "${directories[@]}"; do
    ensure_directory "$dir"
done

print_status "Storage directories created"

# Set permissions
if [ -n "$SUDO_USER" ]; then
    chown -R $SUDO_USER:$SUDO_USER "$STORAGE_BASE"
    chown -R $SUDO_USER:$SUDO_USER "$CONFIG_PATH"
fi

# Step 8: Configure network reconfiguration
print_header "Step 8: Configuring Network Reconfiguration"

# Create kubelet configuration with dynamic node-ip
cat > /etc/default/kubelet <<'EOF'
KUBELET_EXTRA_ARGS="--node-ip=$(ip route get 8.8.8.8 | grep -oP 'src \K\S+')"
EOF

print_status "Kubelet configured for dynamic IP"

# Step 9: Check for join command
print_header "Step 9: Checking for Join Command"

if [ -f /tmp/k8s-join-command.sh ]; then
    print_info "Found join command at /tmp/k8s-join-command.sh"
    
    if ask_yes_no "Join the cluster now?"; then
        print_info "Joining cluster..."
        bash /tmp/k8s-join-command.sh
        print_status "Joined cluster"
        
        # Wait for node to be ready
        print_info "Waiting for node to be ready..."
        sleep 10
        print_status "Worker node joined successfully"
    else
        print_info "Skipping cluster join. Run manually later:"
        echo "  sudo bash /tmp/k8s-join-command.sh"
    fi
else
    print_warning "Join command not found at /tmp/k8s-join-command.sh"
    print_info "Copy join command from master node:"
    echo "  scp master-node:/tmp/k8s-join-command.sh /tmp/"
    echo "  sudo bash /tmp/k8s-join-command.sh"
fi

# Step 10: Install additional tools
print_header "Step 10: Installing Additional Tools"

# Install ffmpeg for camera testing
if ! command_exists ffmpeg; then
    apt-get install -y ffmpeg > /dev/null 2>&1
    print_status "ffmpeg installed"
else
    print_info "ffmpeg already installed"
fi

# Install python3 and pip (for database scripts)
if ! command_exists python3; then
    apt-get install -y python3 python3-pip > /dev/null 2>&1
    print_status "Python3 installed"
else
    print_info "Python3 already installed"
fi

# Completion
print_header "Worker Node Setup Complete!"

echo ""
echo "Node Information:"
echo "  Worker IP: $NODE_IP"
echo "  Storage Base: $STORAGE_BASE"
echo "  Docker Version: $(docker --version)"
echo "  Kubernetes Version: $(get_k8s_version)"
echo ""

if is_node_in_cluster; then
    print_status "Node is part of the cluster!"
else
    print_warning "Node is NOT yet part of the cluster"
    echo ""
    echo "To join the cluster:"
    echo "1. Get the join command from master node:"
    echo "   scp master-node:/tmp/k8s-join-command.sh /tmp/"
    echo ""
    echo "2. Run the join command:"
    echo "   sudo bash /tmp/k8s-join-command.sh"
    echo ""
fi

echo "Next Steps:"
echo "1. Clone BARNS repository (if not done):"
echo "   git clone <repo-url> ~/git-BARNS/BARNS"
echo ""
echo "2. Build Docker images:"
echo "   cd ~/k8s-deployment/scripts"
echo "   ./build-images.sh"
echo ""
echo "3. Deploy to Kubernetes (from master):"
echo "   cd ~/k8s-deployment/scripts"
echo "   ./deploy-k8s.sh"
echo ""

# Save configuration
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(eval echo ~$SUDO_USER)
    cat > "$USER_HOME/k8s-worker-info.txt" <<EOF
Worker Node Information
========================
Node IP: $NODE_IP
Setup Date: $(date)
Storage Base: $STORAGE_BASE
Config Path: $CONFIG_PATH
Docker Version: $(docker --version)
Kubernetes Version: $(get_k8s_version)
EOF
    chown $SUDO_USER:$SUDO_USER "$USER_HOME/k8s-worker-info.txt"
    print_status "Configuration saved to $USER_HOME/k8s-worker-info.txt"
fi

log_message "INFO" "Worker node setup completed successfully"

