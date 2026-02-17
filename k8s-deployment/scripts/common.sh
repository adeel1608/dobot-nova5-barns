#!/bin/bash

# BARNS Kubernetes Deployment - Common Functions
# Source this file in other scripts: source "$(dirname "$0")/common.sh"

# Version
BARNS_DEPLOY_VERSION="1.0.0"

# Color codes
export GREEN='\033[0;32m'
export YELLOW='\033[1;33m'
export RED='\033[0;31m'
export BLUE='\033[0;34m'
export CYAN='\033[0;36m'
export NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="$ROOT_DIR/config"
TEMPLATES_DIR="$ROOT_DIR/templates"

# Configuration file
CONFIG_FILE="$CONFIG_DIR/cluster-config.yaml"

# Print functions
print_header() {
    echo -e "${BLUE}"
    echo "========================================="
    echo "$1"
    echo "========================================="
    echo -e "${NC}"
}

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_info() {
    echo -e "${CYAN}[i]${NC} $1"
}

# Ask yes/no question
ask_yes_no() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

# Check if command exists
command_exists() {
    command -v "$1" &> /dev/null
}

# Check if running as root
is_root() {
    [ "$(id -u)" -eq 0 ]
}

# Require root
require_root() {
    if ! is_root; then
        print_error "This script must be run as root or with sudo"
        exit 1
    fi
}

# Get value from YAML config
get_config() {
    local key="$1"
    local default="${2:-}"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "$default"
        return
    fi
    
    # Simple YAML parser (handles basic key: value format)
    local value=$(grep "^[[:space:]]*${key}:" "$CONFIG_FILE" | head -1 | sed 's/.*:[[:space:]]*//' | sed 's/[[:space:]]*$//' | tr -d '"' | tr -d "'")
    
    if [ -z "$value" ] || [ "$value" = "auto" ]; then
        echo "$default"
    else
        echo "$value"
    fi
}

# Detect node IP address
detect_ip() {
    local ip=""
    
    # Try to get IP from default route interface
    local interface=$(ip route | grep default | awk '{print $5}' | head -1)
    if [ -n "$interface" ]; then
        ip=$(ip addr show "$interface" | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
    fi
    
    # Fallback: get any non-loopback IP
    if [ -z "$ip" ]; then
        ip=$(hostname -I | awk '{print $1}')
    fi
    
    echo "$ip"
}

# Check system requirements
check_requirements() {
    local node_type="$1"  # master or worker
    
    print_info "Checking system requirements..."
    
    # Check OS
    if [ ! -f /etc/os-release ]; then
        print_error "Cannot detect OS"
        return 1
    fi
    
    . /etc/os-release
    print_status "OS: $NAME $VERSION"
    
    # Check architecture
    local arch=$(uname -m)
    print_status "Architecture: $arch"
    
    # Check CPU
    local cpus=$(nproc)
    local min_cpus=2
    [ "$node_type" = "worker" ] && min_cpus=4
    
    if [ "$cpus" -lt "$min_cpus" ]; then
        print_warning "CPU cores: $cpus (recommended: $min_cpus+)"
    else
        print_status "CPU cores: $cpus"
    fi
    
    # Check memory
    local mem_gb=$(free -g | awk '/^Mem:/{print $2}')
    local min_mem=4
    [ "$node_type" = "worker" ] && min_mem=8
    
    if [ "$mem_gb" -lt "$min_mem" ]; then
        print_warning "Memory: ${mem_gb}GB (recommended: ${min_mem}GB+)"
    else
        print_status "Memory: ${mem_gb}GB"
    fi
    
    # Check disk space
    local disk_gb=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
    local min_disk=20
    [ "$node_type" = "worker" ] && min_disk=50
    
    if [ "$disk_gb" -lt "$min_disk" ]; then
        print_warning "Disk space: ${disk_gb}GB (recommended: ${min_disk}GB+)"
    else
        print_status "Disk space: ${disk_gb}GB"
    fi
    
    return 0
}

# Wait for condition with timeout
wait_for_condition() {
    local condition="$1"
    local timeout="${2:-300}"
    local message="${3:-Waiting...}"
    
    print_info "$message"
    
    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if eval "$condition"; then
            print_status "Condition met after ${elapsed}s"
            return 0
        fi
        sleep 5
        elapsed=$((elapsed + 5))
        echo -n "."
    done
    
    echo ""
    print_warning "Timeout after ${timeout}s"
    return 1
}

# Retry command with backoff
retry_command() {
    local max_attempts="${1}"
    local delay="${2}"
    local command="${@:3}"
    
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        if $command; then
            return 0
        fi
        
        if [ $attempt -lt $max_attempts ]; then
            print_warning "Attempt $attempt failed. Retrying in ${delay}s..."
            sleep "$delay"
        fi
        
        attempt=$((attempt + 1))
    done
    
    print_error "Command failed after $max_attempts attempts"
    return 1
}

# Check network connectivity
check_network() {
    local host="${1:-8.8.8.8}"
    
    if ping -c 1 -W 2 "$host" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

# Create directory if not exists
ensure_directory() {
    local dir="$1"
    local owner="${2:-}"
    
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        print_status "Created directory: $dir"
    fi
    
    if [ -n "$owner" ]; then
        chown -R "$owner" "$dir"
    fi
}

# Backup file
backup_file() {
    local file="$1"
    
    if [ -f "$file" ]; then
        local backup="${file}.backup.$(date +%Y%m%d_%H%M%S)"
        cp "$file" "$backup"
        print_status "Backed up: $file -> $backup"
    fi
}

# Check if Kubernetes is installed
is_k8s_installed() {
    command_exists kubectl && command_exists kubeadm && command_exists kubelet
}

# Check if Docker is installed
is_docker_installed() {
    command_exists docker && systemctl is-active --quiet docker
}

# Get Kubernetes version
get_k8s_version() {
    if command_exists kubectl; then
        kubectl version --client --short 2>/dev/null | grep "Client Version" | awk '{print $3}'
    fi
}

# Check if node is master
is_master_node() {
    [ -f /etc/kubernetes/admin.conf ]
}

# Check if node is in cluster
is_node_in_cluster() {
    if is_master_node && command_exists kubectl; then
        kubectl get nodes &> /dev/null
        return $?
    fi
    return 1
}

# Generate random password
generate_password() {
    local length="${1:-16}"
    openssl rand -base64 "$length" | tr -d "=+/" | cut -c1-"$length"
}

# Update YAML value
update_yaml_value() {
    local file="$1"
    local key="$2"
    local value="$3"
    
    if [ -f "$file" ]; then
        backup_file "$file"
        sed -i "s|^[[:space:]]*${key}:.*|  ${key}: ${value}|" "$file"
        print_status "Updated $key in $file"
    fi
}

# Show deployment summary
show_summary() {
    local worker_ip="$1"
    
    print_header "Deployment Summary"
    
    echo "Access your BARNS deployment:"
    echo ""
    echo "  🌐 Dashboard:        http://${worker_ip}:30003"
    echo "  🔌 API Bridge:       http://${worker_ip}:30000"
    echo "  📹 Video Stream:     http://${worker_ip}:30001"
    echo "  📦 OMS Service:      http://${worker_ip}:30002"
    echo "  🐰 RabbitMQ Mgmt:    http://${worker_ip}:30672"
    echo "  📊 InfluxDB:         http://${worker_ip}:30086"
    echo ""
    echo "Default Credentials:"
    echo "  RabbitMQ: guest / guest"
    echo ""
    echo "Useful Commands:"
    echo "  kubectl get pods -n barns"
    echo "  kubectl logs <pod-name> -n barns"
    echo "  kubectl describe pod <pod-name> -n barns"
    echo ""
}

# Log message to file
log_message() {
    local level="$1"
    local message="$2"
    local log_file="${3:-$ROOT_DIR/deployment.log}"
    
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] [$level] $message" >> "$log_file"
}

# Export functions
export -f print_header
export -f print_status
export -f print_warning
export -f print_error
export -f print_info
export -f ask_yes_no
export -f command_exists
export -f get_config
export -f detect_ip
export -f check_requirements
export -f wait_for_condition
export -f check_network
export -f ensure_directory
export -f backup_file
export -f show_summary
export -f log_message

# Print version
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "BARNS Deployment Common Functions v${BARNS_DEPLOY_VERSION}"
    echo "This file should be sourced, not executed directly"
fi

