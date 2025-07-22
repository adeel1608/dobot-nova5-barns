#!/bin/bash
###############################
# BARNS Startup Script for Robot 1 PC
# Starts Docker services and optionally Robot 1
###############################

set -e

# Configuration
START_ROBOT=${START_ROBOT:-true}
WORKSPACE_DIR=${WORKSPACE_DIR:-$HOME/barns_robot_ws}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[BARNS-ROBOT1]${NC} $*"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

error() {
    echo -e "${RED}[ERROR]${NC} $*"
    exit 1
}

info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

# Check prerequisites
check_prerequisites() {
    log "Checking prerequisites..."
    
    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        error "Docker is not installed. Please install Docker first."
    fi
    
    # Check if Docker Compose file exists
    if [ ! -f "docker-compose.arms.yml" ]; then
        error "docker-compose.arms.yml not found. Please run this script from the BARNS project directory."
    fi
    
    # Check if robot workspace exists (if starting robot)
    if [ "$START_ROBOT" = "true" ]; then
        if [ ! -d "$WORKSPACE_DIR" ]; then
            error "Robot workspace not found at $WORKSPACE_DIR. Please run install-robot-dependencies.sh first."
        fi
        
        if [ ! -f "robot1-startup.sh" ]; then
            error "robot1-startup.sh not found. Please ensure the robot startup script is available."
        fi
    fi
    
    log "Prerequisites check passed"
}

# Start Docker services
start_docker_services() {
    log "Setting up X11 forwarding for Docker containers..."
    xhost +local:root 2>/dev/null || warn "Could not set X11 forwarding (xhost not available)"

    log "Stopping existing BARNS services..."
    docker compose -f docker-compose.arms.yml down 2>/dev/null || true

    log "Starting BARNS Docker services..."
    docker compose -f docker-compose.arms.yml up -d --build

    # Wait for services to be ready
    log "Waiting for core services to be ready..."
    local max_wait=90
    local count=0
    
    # Wait for RabbitMQ
    while ! docker exec barns-rabbitmq rabbitmq-diagnostics ping &>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt $max_wait ]; then
            error "RabbitMQ failed to start within $max_wait seconds"
        fi
        echo -n "."
    done
    echo
    
    # Wait a bit more for other services
    sleep 10
    
    log "Docker services started successfully!"
    info "Dashboard: http://localhost:3000"
    info "RabbitMQ Management: http://localhost:15672 (admin/admin123)"
}

# Start Robot 1 in background
start_robot_background() {
    log "Starting Robot 1 in background..."
    
    # Make robot startup script executable
    chmod +x robot1-startup.sh
    
    # Start robot in background with robot-only mode
    nohup ./robot1-startup.sh robot-only > robot1.log 2>&1 &
    local robot_pid=$!
    
    # Save PID for later cleanup
    echo $robot_pid > robot1.pid
    
    log "Robot 1 started with PID: $robot_pid"
    info "Robot 1 logs: tail -f robot1.log"
}

# Monitor services
monitor_services() {
    log "BARNS services are running!"
    echo
    info "Available endpoints:"
    info "  - Dashboard: http://localhost:3000"
    info "  - RabbitMQ Management: http://localhost:15672 (admin/admin123)"
    echo
    info "Service status:"
    info "  - Docker services: Running"
    
    if [ "$START_ROBOT" = "true" ]; then
        if [ -f robot1.pid ] && kill -0 $(cat robot1.pid) 2>/dev/null; then
            info "  - Robot 1: Running (PID: $(cat robot1.pid))"
            info "  - Robot 1 logs: tail -f robot1.log"
        else
            warn "  - Robot 1: Failed to start or stopped unexpectedly"
        fi
    else
        info "  - Robot 1: Not started (use START_ROBOT=true to auto-start)"
    fi
    
    echo
    info "To manually start Robot 1: ./robot1-startup.sh"
    info "To stop all services: ./start-barns-robot1.sh stop"
    echo
    log "Press Ctrl+C to stop all services"
    
    # Wait for interrupt
    trap cleanup INT
    while true; do
        sleep 5
        # Check if Docker services are still running
        if ! docker ps | grep -q barns-rabbitmq; then
            error "Docker services stopped unexpectedly"
        fi
    done
}

# Cleanup function
cleanup() {
    log "Stopping BARNS services..."
    
    # Stop robot if running
    if [ -f robot1.pid ]; then
        local robot_pid=$(cat robot1.pid)
        if kill -0 $robot_pid 2>/dev/null; then
            log "Stopping Robot 1 (PID: $robot_pid)..."
            kill $robot_pid
            # Wait for graceful shutdown
            local count=0
            while kill -0 $robot_pid 2>/dev/null && [ $count -lt 10 ]; do
                sleep 1
                count=$((count + 1))
            done
            # Force kill if still running
            if kill -0 $robot_pid 2>/dev/null; then
                kill -9 $robot_pid 2>/dev/null || true
            fi
        fi
        rm -f robot1.pid
    fi
    
    # Stop Docker services
    log "Stopping Docker services..."
    docker compose -f docker-compose.arms.yml down
    
    log "All services stopped successfully!"
    exit 0
}

# Stop services function
stop_services() {
    cleanup
}

# Main function
main() {
    log "Starting BARNS for Robot 1 PC"
    log "Start Robot: $START_ROBOT"
    echo
    
    check_prerequisites
    start_docker_services
    
    if [ "$START_ROBOT" = "true" ]; then
        start_robot_background
        sleep 5  # Give robot time to start
    fi
    
    monitor_services
}

# Handle command line arguments
case "${1:-start}" in
    start)
        main
        ;;
    stop)
        stop_services
        ;;
    docker-only)
        log "Starting Docker services only..."
        START_ROBOT=false
        check_prerequisites
        start_docker_services
        log "Docker services started. Robot 1 not started."
        info "To start Robot 1: ./robot1-startup.sh"
        ;;
    status)
        log "Checking service status..."
        echo
        if docker ps | grep -q barns-rabbitmq; then
            info "✅ Docker services: Running"
        else
            info "❌ Docker services: Not running"
        fi
        
        if [ -f robot1.pid ] && kill -0 $(cat robot1.pid) 2>/dev/null; then
            info "✅ Robot 1: Running (PID: $(cat robot1.pid))"
        else
            info "❌ Robot 1: Not running"
        fi
        ;;
    logs)
        case "${2:-docker}" in
            docker)
                log "Showing Docker logs..."
                docker compose -f docker-compose.arms.yml logs -f
                ;;
            robot)
                if [ -f robot1.log ]; then
                    log "Showing Robot 1 logs..."
                    tail -f robot1.log
                else
                    warn "Robot 1 log file not found"
                fi
                ;;
            *)
                error "Unknown log type: $2. Use 'docker' or 'robot'"
                ;;
        esac
        ;;
    --help|-h)
        echo "BARNS Startup Script for Robot 1 PC"
        echo
        echo "Usage: $0 [start|stop|docker-only|status|logs|--help]"
        echo
        echo "Commands:"
        echo "  start        Start Docker services and Robot 1 (default)"
        echo "  stop         Stop all services"
        echo "  docker-only  Start only Docker services"
        echo "  status       Show service status"
        echo "  logs         Show logs (docker|robot)"
        echo "  --help       Show this help"
        echo
        echo "Environment variables:"
        echo "  START_ROBOT      Start Robot 1 automatically (default: true)"
        echo "  WORKSPACE_DIR    Robot workspace (default: \$HOME/barns_robot_ws)"
        echo
        echo "Examples:"
        echo "  $0                          # Start everything"
        echo "  START_ROBOT=false $0        # Start only Docker services"
        echo "  $0 logs robot               # Show Robot 1 logs"
        echo "  $0 status                   # Check service status"
        ;;
    *)
        error "Unknown command: $1. Use --help for usage information."
        ;;
esac 