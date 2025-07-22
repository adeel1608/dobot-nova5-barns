# BARNS External Robot Setup Guide

This guide explains how to run BARNS robots outside of Docker containers for improved performance. The new configuration supports:

- **Robot 1**: Runs on a separate PC that also hosts the Docker services
- **Robot 2**: Runs on the same PC as the Docker services

## Architecture Overview

```
┌─────────────────────────────────────┐    ┌─────────────────────────────────────┐
│           Robot 1 PC                │    │           Robot 2 PC                │
│                                     │    │                                     │
│  ┌─────────────────────────────────┐│    │  ┌─────────────────────────────────┐│
│  │        Docker Services          ││    │  │        Docker Services          ││
│  │  • RabbitMQ                     ││    │  │  (connects to Robot 1 PC)       ││
│  │  • PostgreSQL                   ││    │  │                                 ││
│  │  • Redis                        ││    │  │                                 ││
│  │  • Validation Service           ││    │  │                                 ││
│  │  • OMS Service                  ││    │  │                                 ││
│  │  • Dashboard                    ││    │  │                                 ││
│  │  • etc.                         ││    │  │                                 ││
│  └─────────────────────────────────┘│    │  └─────────────────────────────────┘│
│                                     │    │                                     │
│  ┌─────────────────────────────────┐│    │  ┌─────────────────────────────────┐│
│  │        Robot 1 Process          ││    │  │        Robot 2 Process          ││
│  │  • ROS 2 (Domain 0)             ││    │  │  • ROS 2 (Domain 1)             ││
│  │  • Dobot Nova5                  ││    │  │  • Dobot Nova5                  ││
│  │  • Orbbec Camera                ││    │  │  • Orbbec Camera                ││
│  │  • MoveIt                       ││    │  │  • MoveIt                       ││
│  │  • Perception                   ││    │  │  • Perception                   ││
│  └─────────────────────────────────┘│    │  └─────────────────────────────────┘│
│                                     │    │                                     │
│  IP: 192.168.200.249 (Robot)       │    │  IP: 192.168.200.248 (Robot)       │
│  Camera: CP1Z842000YW               │    │  Camera: CP1Z842000F6               │
└─────────────────────────────────────┘    └─────────────────────────────────────┘
```

## Installation Process

### 1. Install Robot Dependencies

Run this on **both** Robot 1 PC and Robot 2 PC:

```bash
# Make the installation script executable
chmod +x install-robot-dependencies.sh

# Install all robot dependencies
./install-robot-dependencies.sh

# This will:
# - Install ROS 2 Humble
# - Install required APT packages
# - Install Python dependencies
# - Install Orbbec SDK
# - Build the robot workspace
# - Create environment setup script
```

### 2. Copy Robot Source Code

The installation script will attempt to copy robot source code automatically. If it fails:

```bash
# Manually copy robot source from the repository
cp -r services/robot_container/ros_ws/src/* $HOME/barns_robot_ws/src/

# Rebuild the workspace
cd $HOME/barns_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --continue-on-error
```

## Running the System

### Robot 1 PC (Primary PC with Docker Services)

#### Option 1: Start Everything Together
```bash
# Start Docker services AND Robot 1
./start-barns-robot1.sh

# Or with explicit environment
START_ROBOT=true ./start-barns-robot1.sh
```

#### Option 2: Start Services Separately
```bash
# Start only Docker services
./start-barns-robot1.sh docker-only

# In another terminal, start Robot 1
./robot1-startup.sh
```

#### Option 3: Use Individual Scripts
```bash
# Start Docker services
./start-barns.sh

# Start Robot 1 (robot-only mode)
./robot1-startup.sh robot-only
```

### Robot 2 PC (Secondary PC)

**Prerequisites**: Ensure Robot 1 PC Docker services are running first.

```bash
# Check if Robot 1 PC services are accessible
./robot2-startup.sh check

# Start Robot 2
./robot2-startup.sh

# Or check status
./robot2-startup.sh status
```

## Configuration

### Environment Variables

#### Robot 1 Configuration
```bash
export WORKSPACE_DIR="$HOME/barns_robot_ws"           # Robot workspace
export IP_ADDRESS="192.168.200.249"                   # Robot IP address
export CAMERA_SERIAL_NUMBER="CP1Z842000YW"            # Camera serial
export USB_PORT="1-5-7"                               # Camera USB port
export ROS_DOMAIN_ID=0                                # ROS domain
```

#### Robot 2 Configuration
```bash
export WORKSPACE_DIR="$HOME/barns_robot_ws"           # Robot workspace
export DOCKER_HOST_IP="192.168.1.100"                # Robot 1 PC IP (update as needed)
export IP_ADDRESS="192.168.200.248"                   # Robot IP address
export CAMERA_SERIAL_NUMBER="CP1Z842000F6"            # Camera serial
export USB_PORT="2-7-6"                               # Camera USB port
export ROS_DOMAIN_ID=1                                # ROS domain
export ROBOT_STARTUP_DELAY=15                         # Delay to avoid USB conflicts
```

### RabbitMQ Connection

Robots connect to RabbitMQ using these URLs:
- **Robot 1**: `amqp://admin:admin123@localhost:5672/` (local Docker)
- **Robot 2**: `amqp://admin:admin123@<ROBOT1_PC_IP>:5672/` (remote Docker)

## Monitoring and Management

### Service Status
```bash
# Check Robot 1 PC status
./start-barns-robot1.sh status

# Check Robot 2 status
./robot2-startup.sh check
```

### View Logs
```bash
# Robot 1 PC - Docker logs
./start-barns-robot1.sh logs docker

# Robot 1 PC - Robot logs
./start-barns-robot1.sh logs robot
# OR
tail -f robot1.log

# Robot 2 - No built-in log aggregation, use terminal output
```

### Stop Services
```bash
# Stop everything on Robot 1 PC
./start-barns-robot1.sh stop

# Stop Robot 2
./robot2-startup.sh stop
# OR
pkill -f "ros2 launch"
pkill -f "ros2 run"
```

## Troubleshooting

### Common Issues

#### 1. RabbitMQ Connection Failed
```bash
# Check if RabbitMQ is accessible from Robot 2 PC
nc -z <ROBOT1_PC_IP> 5672

# Check Docker services on Robot 1 PC
docker ps | grep barns-rabbitmq
```

#### 2. Camera Not Detected
```bash
# List USB devices
lsusb

# List Orbbec devices
ros2 run orbbec_camera list_devices_node

# Check USB permissions
ls -la /dev/bus/usb/
```

#### 3. Robot Service Not Starting
```bash
# Check environment setup
source $HOME/barns_robot_ws/setup_robot_env.sh
env | grep ROS

# Check workspace build
cd $HOME/barns_robot_ws
source /opt/ros/humble/setup.bash
colcon build --continue-on-error
```

#### 4. USB Device Conflicts
```bash
# Clean up device locks
rm -f /dev/shm/orbbec_device_lock

# Restart with delay (Robot 2)
ROBOT_STARTUP_DELAY=30 ./robot2-startup.sh
```

### Performance Optimization

#### 1. USB Buffer Settings
```bash
# Increase USB buffer for cameras
echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb
```

#### 2. ROS 2 Domain Isolation
- Robot 1 uses `ROS_DOMAIN_ID=0`
- Robot 2 uses `ROS_DOMAIN_ID=1`
- This prevents ROS 2 topic conflicts between robots

#### 3. Logging Optimization
- Most ROS nodes run with reduced logging (`__log_level:=fatal`)
- Critical services maintain `info` level logging
- Docker services use standard logging

## Network Configuration

### Firewall Settings (Robot 1 PC)
```bash
# Allow RabbitMQ access from Robot 2 PC
sudo ufw allow from <ROBOT2_PC_IP> to any port 5672
sudo ufw allow from <ROBOT2_PC_IP> to any port 15672  # Management UI (optional)
```

### Port Usage
- `5672`: RabbitMQ AMQP
- `15672`: RabbitMQ Management UI
- `3000`: Dashboard
- `8000`: API Bridge
- `8001`: Video Stream

## Migration from Docker Containers

### Changes Made
1. **Removed Docker containers**: `robot1` and `robot2` services removed from `docker-compose.arms.yml`
2. **Updated networking**: RabbitMQ accessible to external processes
3. **Created installation script**: `install-robot-dependencies.sh` replicates Dockerfile setup
4. **Created startup scripts**: Individual scripts for each robot
5. **Enhanced start script**: `start-barns-robot1.sh` for Robot 1 PC

### Benefits
- **Improved performance**: No Docker overhead for robot processes
- **Better hardware access**: Direct USB and camera access
- **Easier debugging**: Direct access to robot processes
- **Flexible deployment**: Robots can run on separate machines
- **Reduced resource usage**: Lower memory and CPU overhead

## File Summary

| File | Purpose | Usage |
|------|---------|-------|
| `install-robot-dependencies.sh` | Install all robot dependencies | Run once on each robot PC |
| `robot1-startup.sh` | Start Robot 1 process | Robot 1 PC |
| `robot2-startup.sh` | Start Robot 2 process | Robot 2 PC |
| `start-barns-robot1.sh` | Start Docker + Robot 1 | Robot 1 PC (replaces start-barns.sh) |
| `start-barns.sh` | Original Docker-only startup | Robot 2 PC (Docker services only) |

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review log files (`robot1.log`, Docker logs)
3. Verify network connectivity between PCs
4. Ensure all dependencies are properly installed

The external robot setup provides better performance and flexibility while maintaining the same functionality as the original Docker-based system. 