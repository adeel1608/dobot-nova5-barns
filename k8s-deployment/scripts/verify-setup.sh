#!/bin/bash

# BARNS Kubernetes Setup Verification Script
# Verifies that storage, symlinks, and services are properly configured

set -e

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

print_header "BARNS Kubernetes Setup Verification"

SSD_MOUNT="/mnt/ssd"
ERRORS=0
WARNINGS=0

# Function to check and report
check_status() {
    local test_name="$1"
    local result="$2"
    local message="$3"
    local severity="${4:-error}"  # error or warning
    
    if [ "$result" -eq 0 ]; then
        print_status "$test_name: OK"
    else
        if [ "$severity" == "error" ]; then
            print_error "$test_name: FAILED - $message"
            ERRORS=$((ERRORS+1))
        else
            print_warning "$test_name: WARNING - $message"
            WARNINGS=$((WARNINGS+1))
        fi
    fi
}

# Check 1: SSD is mounted
print_header "Storage Checks"
if [ -d "$SSD_MOUNT" ] && mountpoint -q "$SSD_MOUNT" 2>/dev/null; then
    check_status "SSD Mount" 0 ""
else
    check_status "SSD Mount" 1 "SSD not mounted at $SSD_MOUNT"
fi

# Check 2: SSD directories exist
for dir in "$SSD_MOUNT/var/lib/kubelet" "$SSD_MOUNT/var/lib/containerd"; do
    if [ -d "$dir" ]; then
        check_status "Directory $(basename $dir)" 0 ""
    else
        check_status "Directory $(basename $dir)" 1 "Directory $dir does not exist" "warning"
    fi
done

# Check 3: Symlinks are correct
print_header "Symlink Checks"
for link in /var/lib/kubelet /var/lib/containerd; do
    if [ -L "$link" ]; then
        target=$(readlink -f "$link")
        if [[ "$target" == "$SSD_MOUNT"* ]]; then
            print_status "$(basename $link): $link -> $target"
        else
            check_status "$(basename $link)" 1 "Symlink points to wrong location: $target"
        fi
    elif [ -d "$link" ]; then
        check_status "$(basename $link)" 1 "Should be symlink but is directory. Run fix-containerd.sh"
    else
        check_status "$(basename $link)" 1 "Symlink missing. Run fix-containerd.sh" "warning"
    fi
done

# Check 4: Container runtime status
print_header "Service Checks"
if command_exists containerd; then
    if systemctl is-active --quiet containerd; then
        print_status "containerd: Running"
    else
        check_status "containerd" 1 "Not running. Try: sudo systemctl start containerd" "warning"
    fi
else
    check_status "containerd" 1 "Not installed" "warning"
fi

# Check 5: Kubernetes components
if command_exists kubeadm; then
    print_status "Kubernetes tools: Installed ($(kubeadm version -o short))"
else
    check_status "Kubernetes tools" 1 "Not installed" "warning"
fi

# Check 6: Network configuration
print_header "Network Checks"
NODE_IP=$(detect_ip)
print_info "Detected IP: $NODE_IP"

# Check if kubeconfig exists and what IP it points to
if [ -f ~/.kube/config ]; then
    CONFIG_IP=$(grep "server:" ~/.kube/config | awk -F'//' '{print $2}' | cut -d':' -f1 | head -n1)
    if [ -n "$CONFIG_IP" ]; then
        print_info "Kubeconfig points to: $CONFIG_IP"
        if [ "$CONFIG_IP" != "$NODE_IP" ]; then
            check_status "IP Match" 1 "Kubeconfig IP ($CONFIG_IP) != Current IP ($NODE_IP). Run update-network.sh" "warning"
        else
            print_status "IP Match: OK"
        fi
    fi
fi

# Check 7: Port availability
print_header "Port Checks"
for port in 6443 2379 2380 10250; do
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        print_info "Port $port: In use"
    else
        print_info "Port $port: Available"
    fi
done

# Check 8: System resources
print_header "Resource Checks"
TOTAL_MEM=$(free -g | awk '/^Mem:/{print $2}')
AVAILABLE_MEM=$(free -g | awk '/^Mem:/{print $7}')
print_info "Memory: ${AVAILABLE_MEM}GB available / ${TOTAL_MEM}GB total"
if [ "$AVAILABLE_MEM" -lt 4 ]; then
    check_status "Memory" 1 "Less than 4GB available" "warning"
fi

DISK_FREE=$(df -BG "$SSD_MOUNT" | awk 'NR==2 {print $4}' | sed 's/G//')
print_info "SSD Space: ${DISK_FREE}GB available"
if [ "$DISK_FREE" -lt 20 ]; then
    check_status "Disk Space" 1 "Less than 20GB available on SSD" "warning"
fi

# Check 9: Swap status
if [ $(swapon --show | wc -l) -gt 0 ]; then
    check_status "Swap" 1 "Swap is enabled. Kubernetes requires it to be disabled" "warning"
else
    print_status "Swap: Disabled (correct)"
fi

# Summary
print_header "Verification Summary"
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    print_status "All checks passed! ✓"
    echo ""
    echo "You can proceed with:"
    echo "  - Master node: sudo ./setup-master.sh"
    echo "  - Worker node: sudo ./setup-worker.sh"
elif [ $ERRORS -eq 0 ]; then
    print_warning "$WARNINGS warning(s) found"
    echo ""
    echo "You can proceed, but review the warnings above."
else
    print_error "$ERRORS error(s) and $WARNINGS warning(s) found"
    echo ""
    echo "Please fix the errors before proceeding:"
    echo ""
    echo "Common fixes:"
    echo "  - For symlink/storage errors: sudo ./fix-containerd.sh"
    echo "  - For network/IP errors: sudo ./update-network.sh"
    echo "  - For service errors: sudo systemctl status <service>"
    exit 1
fi

echo ""
print_info "Current node IP: $NODE_IP"
print_info "Current date: $(date)"
echo ""
