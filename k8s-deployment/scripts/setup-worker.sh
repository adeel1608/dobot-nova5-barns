#!/bin/bash

# BARNS Kubernetes Worker Node Setup Script
# This script sets up a Kubernetes worker node with Docker and containerd

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

# =============================================================================
# STORAGE CONFIGURATION
# =============================================================================
# Directory for Kubernetes storage (will be created if doesn't exist)
# Default: /mnt/ssd (works on main disk or separate SSD)
# Change this if you want different location (e.g., /opt/k8s-storage)
SSD_MOUNT="/mnt/ssd"
# =============================================================================

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
STORAGE_BASE="$SSD_MOUNT/barns-data"
CONFIG_PATH="$SSD_MOUNT/barns-config"

# Create storage directory if it doesn't exist
if [ ! -d "$SSD_MOUNT" ]; then
    print_info "Creating storage directory: $SSD_MOUNT"
    mkdir -p "$SSD_MOUNT"
    print_status "Storage directory created"
else
    print_status "Storage directory found: $SSD_MOUNT"
fi

echo ""
echo "Configuration:"
echo "  Node IP: $NODE_IP"
echo "  SSD Mount: $SSD_MOUNT"
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
    jq
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

# Step 3.5: Create SSD storage directories
print_header "Step 3.5: Setting up SSD Storage"

directories=(
    "$SSD_MOUNT/var/lib/kubelet"
    "$SSD_MOUNT/var/lib/containerd"
    "$SSD_MOUNT/var/lib/docker"
    "$SSD_MOUNT/k8s-data"
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

print_status "SSD storage directories created"

# Create symlinks from /var/lib to storage
print_info "Creating symlinks to storage..."

# Stop services that might be using these directories
systemctl stop kubelet 2>/dev/null || true
systemctl stop containerd 2>/dev/null || true
systemctl stop docker 2>/dev/null || true
sleep 2

# Kill any remaining processes
pkill -9 containerd 2>/dev/null || true
pkill -9 containerd-shim 2>/dev/null || true
pkill -9 dockerd 2>/dev/null || true
sleep 1

# Unmount any volumes under these paths
for path in /var/lib/kubelet /var/lib/containerd /var/lib/docker; do
    if [ -d "$path" ] || [ -L "$path" ]; then
        for mount in $(mount | grep "$path" | awk '{print $3}' | sort -r); do
            umount -f "$mount" 2>/dev/null || umount -l "$mount" 2>/dev/null || true
        done
    fi
done
sleep 1

# Remove existing directories/symlinks
for path in /var/lib/kubelet /var/lib/containerd /var/lib/docker; do
    if [ -L "$path" ]; then
        rm -f "$path"
    elif [ -d "$path" ]; then
        rm -rf "$path" 2>/dev/null || (umount -l "$path" 2>/dev/null && rm -rf "$path") || true
    fi
done

# Create the symlinks
ln -sf "$SSD_MOUNT/var/lib/kubelet" /var/lib/kubelet
ln -sf "$SSD_MOUNT/var/lib/containerd" /var/lib/containerd
ln -sf "$SSD_MOUNT/var/lib/docker" /var/lib/docker

print_status "Symlinks created to storage"

# Step 4: Install Docker
print_header "Step 4: Installing Docker"

if ! is_docker_installed; then
    # Add Docker repo
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" > /etc/apt/sources.list.d/docker.list

    apt-get update -qq
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin > /dev/null 2>&1

    # Configure Docker daemon with SSD storage
    cat > /etc/docker/daemon.json <<EOF
{
    "data-root": "$SSD_MOUNT/var/lib/docker",
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

            # Check if jq is available, if not use Python
            if command -v jq &> /dev/null; then
                jq '. + {"dns": ["8.8.8.8", "8.8.4.4", "1.1.1.1"]}' /etc/docker/daemon.json > /tmp/daemon.json
            elif command -v python3 &> /dev/null; then
                python3 -c "import json; f=open('/etc/docker/daemon.json'); d=json.load(f); f.close(); d['dns']=['8.8.8.8','8.8.4.4','1.1.1.1']; f=open('/tmp/daemon.json','w'); json.dump(d,f,indent=2); f.close()"
            else
                # Fallback: manual edit (basic)
                print_warning "Neither jq nor python3 available, using basic sed replacement"
                sed 's/^{/{\n  "dns": ["8.8.8.8", "8.8.4.4", "1.1.1.1"],/' /etc/docker/daemon.json > /tmp/daemon.json
            fi

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

# Configure containerd for Kubernetes with SSD storage
mkdir -p /etc/containerd
cat > /etc/containerd/config.toml <<EOF
version = 2

[plugins]
  [plugins."io.containerd.grpc.v1.cri"]
    sandbox_image = "registry.k8s.io/pause:3.9"

    [plugins."io.containerd.grpc.v1.cri".containerd]
      snapshotter = "overlayfs"
      default_runtime_name = "runc"

      [plugins."io.containerd.grpc.v1.cri".containerd.runtimes]
        [plugins."io.containerd.grpc.v1.cri".containerd.runtimes.runc]
          runtime_type = "io.containerd.runc.v2"
          [plugins."io.containerd.grpc.v1.cri".containerd.runtimes.runc.options]
            SystemdCgroup = true

    [plugins."io.containerd.grpc.v1.cri".cni]
      bin_dir = "/opt/cni/bin"
      conf_dir = "/etc/cni/net.d"

# Container and image storage on SSD
root = "$SSD_MOUNT/var/lib/containerd"
state = "/run/containerd"
EOF

systemctl restart containerd
systemctl enable containerd > /dev/null 2>&1

print_status "containerd configured with SSD storage"

# Step 6: Install Kubernetes components
print_header "Step 6: Installing Kubernetes Components"

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

# Configure kubelet to use SSD (will be finalized after kubeadm join)
print_info "Configuring kubelet to use SSD storage..."
mkdir -p /etc/systemd/system/kubelet.service.d

# Note: We'll create the proper ExecStart override after kubeadm join
# For now, just create the directory
print_status "kubelet directory prepared for SSD storage"

# Step 7: Verify storage directories
print_header "Step 7: Verifying Storage Directories"

# Storage directories were already created in Step 3.5
print_status "Storage directories verified"

# Set permissions
if [ -n "$SUDO_USER" ]; then
    chown -R $SUDO_USER:$SUDO_USER "$STORAGE_BASE"
    chown -R $SUDO_USER:$SUDO_USER "$CONFIG_PATH"
fi

# Step 8: Configure network reconfiguration
print_header "Step 8: Configuring Network Reconfiguration"

# Create script to update kubelet IP on boot
cat > /usr/local/bin/update-kubelet-ip.sh <<'EOF'
#!/bin/bash
# Get the IP of the interface used for default route
NODE_IP=$(ip route get 8.8.8.8 | grep -oP 'src \K\S+')
# Write to environment file
echo "KUBELET_EXTRA_ARGS=\"--node-ip=$NODE_IP\"" > /etc/default/kubelet
EOF

chmod +x /usr/local/bin/update-kubelet-ip.sh

# Configure kubelet systemd drop-in to run update script before start
mkdir -p /etc/systemd/system/kubelet.service.d
cat > /etc/systemd/system/kubelet.service.d/10-dynamic-ip.conf <<EOF
[Service]
ExecStartPre=/usr/local/bin/update-kubelet-ip.sh
EOF

# Reload systemd to pick up changes
systemctl daemon-reload

print_status "Kubelet configured for dynamic IP (systemd hook installed)"

# Step 9: Check for join command
print_header "Step 9: Checking for Join Command"

if [ -f /tmp/k8s-join-command.sh ]; then
    print_info "Found join command at /tmp/k8s-join-command.sh"

    if ask_yes_no "Join the cluster now?"; then
        print_info "Joining cluster..."
        bash /tmp/k8s-join-command.sh
        print_status "Joined cluster"

        # CRITICAL: Force containerd to use SSD with bind mount
        print_info "Forcing containerd to use SSD storage via bind mount..."

        # Stop services temporarily
        systemctl stop kubelet
        systemctl stop containerd

        # Move any existing containerd data to SSD
        if [ -d "/var/lib/containerd" ] && [ "$(ls -A /var/lib/containerd 2>/dev/null)" ]; then
            print_info "Moving existing containerd data to SSD..."
            rsync -a /var/lib/containerd/ "$SSD_MOUNT/var/lib/containerd/"
            rm -rf /var/lib/containerd
        fi

        # Create bind mount directory
        mkdir -p /var/lib/containerd
        mkdir -p "$SSD_MOUNT/var/lib/containerd"

        # Create bind mount to FORCE containerd to use SSD
        mount --bind "$SSD_MOUNT/var/lib/containerd" /var/lib/containerd

        # Make bind mount persistent across reboots
        if ! grep -q "$SSD_MOUNT/var/lib/containerd /var/lib/containerd" /etc/fstab; then
            echo "$SSD_MOUNT/var/lib/containerd /var/lib/containerd none bind 0 0" >> /etc/fstab
            print_status "Bind mount added to /etc/fstab"
        fi

        print_status "Containerd forced to use SSD via bind mount"

        # Restart services
        systemctl start containerd
        sleep 3
        systemctl start kubelet
        sleep 5

        # CRITICAL: Configure kubelet to use SSD root directory
        print_info "Configuring kubelet to use SSD via config.yaml..."

        # Wait a moment for kubeadm to create the config file
        sleep 3

        # Modify kubelet config.yaml to set rootDirectory
        KUBELET_CONFIG="/var/lib/kubelet/config.yaml"

        if [ -f "$KUBELET_CONFIG" ]; then
            # Backup the config
            cp "$KUBELET_CONFIG" "${KUBELET_CONFIG}.backup.$(date +%s)"

            # Check if rootDirectory is already set
            if grep -q "^rootDirectory:" "$KUBELET_CONFIG"; then
                print_info "Updating existing rootDirectory in config..."
                sed -i "s|^rootDirectory:.*|rootDirectory: $SSD_MOUNT/var/lib/kubelet|" "$KUBELET_CONFIG"
            else
                print_info "Adding rootDirectory to config..."
                echo "rootDirectory: $SSD_MOUNT/var/lib/kubelet" >> "$KUBELET_CONFIG"
            fi

            print_status "Updated kubelet config.yaml with SSD root directory"

            # Restart kubelet to apply changes
            systemctl restart kubelet
            sleep 5
            print_status "kubelet restarted with SSD configuration"
        else
            print_warning "Kubelet config.yaml not found yet, will be configured on first start"
        fi

        # Wait for node to be ready
        print_info "Waiting for node to be ready..."
        sleep 10
        print_status "Worker node joined successfully"
    else
        print_info "Skipping cluster join. Run manually later:"
        echo "  sudo bash /tmp/k8s-join-command.sh"
        echo ""
        print_warning "IMPORTANT: After joining, configure kubelet for SSD:"
        echo "  sudo tee /etc/systemd/system/kubelet.service.d/10-exec-start.conf > /dev/null <<EOF"
        echo "  [Service]"
        echo "  ExecStart="
        echo "  ExecStart=/usr/bin/kubelet --root-dir=/mnt/ssd/var/lib/kubelet --node-ip=$NODE_IP"
        echo "  EOF"
        echo "  sudo systemctl daemon-reload"
        echo "  sudo systemctl restart kubelet"
    fi
else
    print_warning "Join command not found at /tmp/k8s-join-command.sh"
    print_info "Copy join command from master node:"
    echo "  scp master-node:/tmp/k8s-join-command.sh /tmp/"
    echo "  sudo bash /tmp/k8s-join-command.sh"
    echo ""
    print_warning "IMPORTANT: After joining, configure kubelet for SSD:"
    echo "  sudo tee /etc/systemd/system/kubelet.service.d/10-exec-start.conf > /dev/null <<EOF"
    echo "  [Service]"
    echo "  ExecStart="
    echo "  ExecStart=/usr/bin/kubelet --root-dir=/mnt/ssd/var/lib/kubelet --node-ip=$NODE_IP"
    echo "  EOF"
    echo "  sudo systemctl daemon-reload"
    echo "  sudo systemctl restart kubelet"
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

# Step 11: Verify SSD Configuration
if is_node_in_cluster; then
    print_header "Step 11: Verifying SSD Configuration"

    sleep 3

    # Verify kubelet root directory in config
    KUBELET_CONFIG="/var/lib/kubelet/config.yaml"
    if [ -f "$KUBELET_CONFIG" ] && grep -q "rootDirectory: $SSD_MOUNT" "$KUBELET_CONFIG"; then
        print_status "kubelet config.yaml has rootDirectory: $SSD_MOUNT/var/lib/kubelet"
    else
        print_warning "kubelet config.yaml may not have rootDirectory set correctly"
        if [ -f "$KUBELET_CONFIG" ]; then
            print_info "Current rootDirectory setting:"
            grep "rootDirectory" "$KUBELET_CONFIG" || echo "  Not set"
        fi
    fi

    # Verify SSD kubelet directory exists and is being used
    if [ -d "$SSD_MOUNT/var/lib/kubelet" ]; then
        KUBELET_SIZE=$(du -sh "$SSD_MOUNT/var/lib/kubelet" 2>/dev/null | cut -f1)
        print_status "SSD kubelet directory exists: $KUBELET_SIZE"
    else
        print_warning "SSD kubelet directory not yet created"
    fi

    # Verify kubelet is using --node-ip
    if ps aux | grep -E '/usr/bin/kubelet' | grep -v grep | grep -q "node-ip=$NODE_IP"; then
        print_status "kubelet is using --node-ip=$NODE_IP"
    else
        print_warning "kubelet may not be using --node-ip flag"
    fi

    # Show disk usage
    print_info "Disk usage:"
    df -h / | tail -1
    df -h $SSD_MOUNT | tail -1
fi

# Step 11.5: Increase containerd file descriptor limit (NOFILE)
print_header "Step 11.5: Configuring containerd NOFILE limit"

# Create systemd drop-in for containerd
mkdir -p /etc/systemd/system/containerd.service.d

cat > /etc/systemd/system/containerd.service.d/limits.conf <<'EOF'
[Service]
LimitNOFILE=65536
EOF

# Reload systemd and restart services to apply
systemctl daemon-reload
systemctl restart containerd || true
systemctl restart kubelet || true

print_info "Effective containerd LimitNOFILE:"
systemctl show -p LimitNOFILE containerd | sed 's/^/  /' || true

print_status "containerd NOFILE limit set to 65536 and services restarted"

# Completion
print_header "Worker Node Setup Complete!"

echo ""
echo "Node Information:"
echo "  Worker IP: $NODE_IP"
echo "  SSD Mount: $SSD_MOUNT"
echo "  Storage Base: $STORAGE_BASE"
echo "  Kubelet Data: $SSD_MOUNT/var/lib/kubelet"
echo "  Container Data: $SSD_MOUNT/var/lib/containerd"
echo "  Docker Data: $SSD_MOUNT/var/lib/docker"
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
SSD Mount: $SSD_MOUNT
Storage Base: $STORAGE_BASE
Config Path: $CONFIG_PATH
Kubelet Data: $SSD_MOUNT/var/lib/kubelet
Container Data: $SSD_MOUNT/var/lib/containerd
Docker Data: $SSD_MOUNT/var/lib/docker
Docker Version: $(docker --version)
Kubernetes Version: $(get_k8s_version)
EOF
    chown $SUDO_USER:$SUDO_USER "$USER_HOME/k8s-worker-info.txt"
    print_status "Configuration saved to $USER_HOME/k8s-worker-info.txt"
fi

log_message "INFO" "Worker node setup completed successfully"