#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
log() {
    echo -e "${GREEN}[BARNS]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[BARNS]${NC} $1"
}

error() {
    echo -e "${RED}[BARNS]${NC} $1"
}

# Function to start BARNS services
start_barns() {
    log "Setting up X11 forwarding for Docker containers..."
    xhost +local:root

    log "Stopping existing BARNS services..."
    docker compose -f docker-compose.arms.yml down

    log "Starting BARNS services..."
    # Try to start with existing images first, only build if needed
    docker compose -f docker-compose.arms.yml up -d --no-build || {
        warn "Some services need to be built. Building now..."
        docker compose -f docker-compose.arms.yml up -d --build
    }

    log "BARNS services started successfully!"
    echo ""
    log "Dashboard: http://localhost:3000"
    log "RabbitMQ Management: http://localhost:15672 (admin/admin123)"
}

# Function to stop BARNS services
stop_barns() {
    log "Stopping BARNS services..."
    docker compose -f docker-compose.arms.yml down
    
    log "BARNS services stopped successfully!"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 {start|stop}"
    echo ""
    echo "Commands:"
    echo "  start    Stop existing services and start BARNS stack"
    echo "  stop     Stop all BARNS services"
    echo ""
    echo "Examples:"
    echo "  $0 start"
    echo "  $0 stop"
}

# Main script logic
case "$1" in
    start)
        start_barns
        ;;
    stop)
        stop_barns
        ;;
    *)
        error "Invalid command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac 