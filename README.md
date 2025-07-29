# BARNS - Business Automation & Robotics Network System

![BARNS Logo](BARNS%20Logo.png)

A comprehensive microservices-based automation platform for business operations, robotics control, and real-time monitoring with AI integration.

## Overview

BARNS is an event-driven microservices platform that orchestrates complex robotic operations through intelligent task decomposition, real-time monitoring, and scalable service communication.

**Core Capabilities:**
- **Intelligent Order Processing** - Complete order lifecycle management with smart scheduling
- **Robotic Task Orchestration** - Multi-arm coordination with dependency resolution
- **Real-time Monitoring** - Live dashboard with WebSocket updates and camera feeds
- **Event-Driven Architecture** - RabbitMQ-based reliable message delivery
- **Horizontal Scalability** - Docker-based microservices with independent scaling
- **Flexible Robot Deployment** - Support for both containerized and external robot processes

**Deployment Options:**
- **Container-Based Robots** - Full Docker deployment with robot arm containers
- **External Robot Processes** - High-performance native robot processes with Docker services
- **Hybrid Architecture** - Mix of containerized services and external robot hardware

---

## 📋 Table of Contents

- [Architecture Overview](#architecture-overview)
- [Communication Patterns](#communication-patterns)
- [Service Data Flows](#service-data-flows)
- [Quick Start](#quick-start)
- [External Robot Setup](#external-robot-setup)
- [Services Detailed](#services-detailed)
- [Scalability & Deployment](#scalability--deployment)
- [Development](#development)
- [Troubleshooting](#troubleshooting)

---

## 🏗️ Architecture Overview

### High-Level System Flow

```
┌─────────────────┐    HTTP/WebSocket     ┌─────────────────┐
│   Dashboard     │ ◄───────────────────► │   API Bridge    │
│   (Frontend)    │                       │  (HTTP→RabbitMQ)│
└─────────────────┘                       └─────────────────┘
                                                   │
                                          RabbitMQ Messages
                                                   ▼
                                      ┌─────────────────┐
                                      │    RabbitMQ     │
                                      │  Message Broker │
                                      └─────────────────┘
                                               │
                     ┌─────────────────────────┼─────────────────────────┐
                     │                         │                         │
                ┌────▼────┐               ┌────▼────┐               ┌────▼────┐
                │   OMS   │               │Scheduler│               │Routine  │
                │ Service │               │ Service │               │ Service │
                └─────────┘               └─────────┘               └─────────┘
                     │                         │                         │
                ┌────▼────┐               ┌────▼────┐               ┌────▼────┐
                │ Order   │               │ Task    │               │ Robot   │
                │ Queue   │               │Breakdown│               │ Arms    │
                └─────────┘               └─────────┘               └─────────┘
```

### Deployment Architecture Options

#### Option 1: Container-Based (docker-compose.arms.yml)
```
┌─────────────────────────────────────────────────────────────┐
│                    Single Docker Host                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐  │
│  │ Core Services   │  │   Robot 1       │  │  Robot 2    │  │
│  │ • RabbitMQ      │  │   Container     │  │  Container  │  │
│  │ • PostgreSQL    │  │ • ROS 2 (Dom 0) │  │ • ROS 2 (1) │  │
│  │ • Dashboard     │  │ • Nova5 Robot   │  │ • Nova5     │  │
│  │ • All Services  │  │ • Camera/Vision │  │ • Camera    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

#### Option 2: External Robot Processes (Recommended for Production)
```
┌─────────────────────────────────────┐    ┌─────────────────────────────────────┐
│           Robot 1 PC                │    │           Robot 2 PC                │
│                                     │    │                                     │
│  ┌─────────────────────────────────┐│    │  ┌─────────────────────────────────┐│
│  │        Docker Services          ││    │  │        Docker Services          ││
│  │  • RabbitMQ                     ││    │  │  (connects to Robot 1 PC)       ││
│  │  • PostgreSQL                   ││    │  │                                 ││
│  │  • Dashboard                    ││    │  │                                 ││
│  │  • All Core Services            ││    │  │                                 ││
│  └─────────────────────────────────┘│    │  └─────────────────────────────────┘│
│                                     │    │                                     │
│  ┌─────────────────────────────────┐│    │  ┌─────────────────────────────────┐│
│  │        Robot 1 Process          ││    │  │        Robot 2 Process          ││
│  │  • Native ROS 2 (Domain 0)      ││    │  │  • Native ROS 2 (Domain 1)      ││
│  │  • Direct Hardware Access       ││    │  │  • Direct Hardware Access       ││
│  │  • High Performance             ││    │  │  • High Performance             ││
│  └─────────────────────────────────┘│    │  └─────────────────────────────────┘│
└─────────────────────────────────────┘    └─────────────────────────────────────┘
```

### Core Principles

1. **Event-Driven Communication** - All services communicate via RabbitMQ events and messages
2. **Service Isolation** - Each service encapsulates specific business logic
3. **Data Consistency** - PostgreSQL for persistence, Redis for queuing, RabbitMQ for messaging
4. **Real-time Updates** - WebSocket connections for live dashboard updates
5. **Fault Tolerance** - Message queuing ensures reliability and recovery

---

## 🔄 Communication Patterns

### 1. Request-Response Pattern (Synchronous)
```
Dashboard → API Bridge → RabbitMQ → Target Service
                                          │
Dashboard ← API Bridge ← RabbitMQ ← Response
```

**Used for:** Order creation, status queries, system commands

### 2. Event Broadcasting Pattern (Asynchronous)
```
Service → RabbitMQ Event → All Interested Services
                      │
                      └─→ Dashboard (via WebSocket)
```

**Used for:** Order status updates, system alerts, progress notifications

### 3. Task Delegation Pattern (Producer-Consumer)
```
Scheduler → RabbitMQ Queue → Routine Service → Robot Arms
                      │
Feedback ←────────────┘
```

**Used for:** Task execution, robotic arm coordination

### 4. Service Coordination Pattern (Synchronous Multi-Service)
```
Routine Service → RabbitMQ → Validation Service
                │                     │
                ├─ RabbitMQ → Automation Service  
                │                     │
                └─ Responses ←────────┘
```

**Used for:** Multi-step task execution requiring validation and automation coordination

---

## 📊 Service Data Flows

### Order Processing Flow (Complete Lifecycle)

#### Phase 1: Order Creation & Queuing
```mermaid
sequenceDiagram
    participant D as Dashboard
    participant AB as API Bridge
    participant OMS as OMS Service
    participant PG as PostgreSQL
    participant R as Redis

    D->>AB: POST /api/orders/create
    AB->>OMS: create_order (RabbitMQ)
    OMS->>PG: Store order data
    OMS->>R: Add to order queue
    OMS->>AB: Success response
    AB->>D: HTTP 201 Created
    OMS->>AB: order_created event
    AB->>D: WebSocket update
```

#### Phase 2: Order Scheduling & Task Breakdown
```mermaid
sequenceDiagram
    participant D as Dashboard
    participant AB as API Bridge
    participant OMS as OMS Service
    participant S as Scheduler
    participant RT as Routine

    D->>AB: POST /api/orders/{id}/start
    AB->>OMS: start_order (RabbitMQ)
    OMS->>S: process_order (RabbitMQ)
    S->>S: Load recipe & break down tasks
    S->>RT: submit_task (per arm/task)
    RT->>S: task_queued response
    S->>OMS: order_processing_started event
    OMS->>AB: Status update event
    AB->>D: WebSocket update
```

#### Phase 3: Task Execution & Feedback
```mermaid
sequenceDiagram
    participant RT as Routine Service
    participant ARM as Robot Arm
    participant S as Scheduler
    participant OMS as OMS Service
    participant D as Dashboard

    RT->>ARM: Execute task
    ARM->>RT: Task completion
    RT->>S: task_completed event
    S->>S: Check dependencies
    alt All tasks complete
        S->>OMS: order_completed event
        OMS->>D: Order finished (WebSocket)
    else More tasks pending
        S->>RT: Next task
    end
```

#### Phase 4: Task Execution with Service Coordination
```mermaid
sequenceDiagram
    participant RT as Routine Service
    participant V as Validation Service
    participant A as Automation Service
    participant R as Robot Arm Service
    participant S as Scheduler

    Note over RT: Process task steps sequentially
    
    RT->>V: validate (check_ingredient_availability)
    V->>RT: {passed: true, details: "sufficient ingredients"}
    
    RT->>R: robot_action (grind_beans, arm_id: 1)
    R->>RT: {success: true, message: "beans ground"}
    
    RT->>A: automate (heat_water)
    A->>RT: {success: true, details: "water heated to 93°C"}
    
    RT->>R: robot_action (move_to_position, arm_id: 1)
    R->>RT: {success: true, message: "moved to position"}
    
    RT->>V: validate (update_inventory)
    V->>RT: {passed: true, details: "inventory updated"}
    
    RT->>S: task_completed event
    S->>S: Update order progress
```

### Real-time Dashboard Updates

```
Service Event → RabbitMQ → API Bridge → WebSocket → Dashboard
     │                                                  │
     └─ order_started                                   └─ Live UI Update
     └─ task_completed                                  └─ Progress Bar
     └─ system_alert                                    └─ Notification
     └─ inventory_warning                               └─ Alert Panel
     └─ validation.failed                               └─ Error Dialog
     └─ automation.error                                └─ Equipment Alert
```

### Service Coordination Patterns

#### Routine ↔ Validation Communication
```
Routine Service                    Validation Service
     │                                     │
     ├─ validation step execution          │
     ├─ send_request("validation", ...)────┤
     │                                     ├─ check_ingredient_availability()
     │                                     ├─ update_inventory()
     │                                     ├─ validate_test1/test2()
     ├─ await response ◄───────────────────┤
     ├─ continue/abort based on result     │
```

#### Routine ↔ Automation Communication  
```
Routine Service                    Automation Service
     │                                     │
     ├─ automation step execution          │
     ├─ send_request("automation", ...)────┤
     │                                     ├─ heat_water()
     │                                     ├─ dispense_milk()
     │                                     ├─ automation_test1/test2()
     ├─ await response ◄───────────────────┤
     ├─ continue/abort based on result     │
```

#### Routine ↔ Robot Arm Communication
```
Routine Service                    Robot Arm Service                    Robot Container
     │                                     │                                 │
     ├─ robot step execution               │                                 │
     ├─ send_request("robot_arm", ...)─────┤                                 │
     │                                     ├─ Test Functions:               │
     │                                     │   ├─ robot_test1/test2()       │
     │                                     │   └─ (executed locally)        │
     │                                     ├─ Real Robot Functions:         │
     │                                     │   ├─ RabbitMQ Bridge ─────────►│
     │                                     │   └─ send_request(...)         ├─ ACTION_MAP functions:
     │                                     │                                 │   ├─ home()
     │                                     │                                 │   ├─ move_to_position()
     │                                     │                                 │   ├─ pick_and_place()
     ├─ await response ◄───────────────────┤◄─────────────────────────────────┤   ├─ sequences.home.*()
     ├─ continue/abort based on result     │                                 │   ├─ sequences.cups.*()
     ├─ arm_id specific targeting          │                                 │   └─ sequences.espresso.*()
```

### Complete System Communication Flow

```
┌─────────────┐   HTTP    ┌─────────────┐   RabbitMQ   ┌─────────────┐
│  Dashboard  │◄─────────►│ API Bridge  │◄────────────►│     OMS     │
└─────────────┘           └─────────────┘              └─────────────┘
                                │                              │
                          WebSocket Events                RabbitMQ Messages
                                │                              │
                                ▼                              ▼
                         Real-time Updates              ┌─────────────┐
                                                        │  Scheduler  │
                                                        └─────────────┘
                                                               │
                                                        RabbitMQ Messages
                                                               │
                                                               ▼
                                                        ┌─────────────┐
                                                        │   Routine   │
                                                        └─────────────┘
                                                        │             │
                                              RabbitMQ Messages      Robot
                                                        │           Control
                                                        ▼             │
                                                                            ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
                                    │ Validation  │   │ Automation  │   │ Robot Arm   │
                                    └─────────────┘   └─────────────┘   └─────────────┘
                                    │             │   │             │   │             │
                               Inventory       Test   Equipment    Test   Physical     Test
                              Management    Functions   Control  Functions Movement Functions
                                    │                               │   │             │
                                    └─────────────┬─────────────────┘   │             │
                                                  │                     │             │
                                         RabbitMQ Coordination          │             │
                                                  │                     │             │
                                                  └─────────────────────┴─────────────┘
                                                                        │
                                                               Physical Robot Arms
                                                                 (Optional Hardware)
                                                                ┌─────────────────┐
                                                                │ Robot 1 & 2     │
                                                                │ (Dobot Nova5)   │
                                                                │ VNC: 5901/5902  │
                                                                └─────────────────┘
```

**Message Flow Summary**:
1. **User Interaction**: Dashboard → API Bridge (HTTP)
2. **Order Management**: API Bridge → OMS (RabbitMQ)
3. **Task Orchestration**: OMS → Scheduler (RabbitMQ)
4. **Task Execution**: Scheduler → Routine (RabbitMQ)
5. **Service Coordination**: Routine ↔ Validation/Automation (RabbitMQ)
6. **Real-time Updates**: All Services → API Bridge → Dashboard (WebSocket)

---

## 🚀 Quick Start

### Prerequisites
- Docker Desktop with 8GB+ RAM
- Docker Compose v2.0+

### 1. Launch the System

#### Option A: Software-Only Deployment (No Physical Robot Arms)
```bash
# Clone repository
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS

# Start all services (RabbitMQ-based architecture)
./barns.sh start

# Or using docker-compose directly
docker-compose up --build -d

# Verify services are healthy
docker-compose ps
```

#### Option B: Container-Based Robot Deployment
```bash
# Clone repository
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS

# Set environment variables for robot containers
export UID=$(id -u)
export GID=$(id -g)
export DISPLAY=:0

# Start all services including robot containers
docker-compose -f docker-compose.arms.yml up --build -d

# Verify all services including robot arms are running
docker-compose -f docker-compose.arms.yml ps
```

#### Option C: External Robot Processes (Production Recommended)
```bash
# Clone repository
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS

# Install robot dependencies (run on each robot PC)
chmod +x install-robot-dependencies.sh
./install-robot-dependencies.sh

# Single PC Setup - Robot 1 PC: Start everything
./robot1-startup.sh

# OR Distributed Setup:
# Robot 1 PC: Start Docker services + Robot 1
./robot1-startup.sh docker-only
./robot1-startup.sh robot-only

# Robot 2 PC: Start Robot 2 (connects to Robot 1 PC)
export DOCKER_HOST_IP="<ROBOT1_PC_IP>"
./robot2-startup.sh robot-only

# OR Services-only approach:
./barns.sh start  # Docker services only
./robot1-startup.sh robot-only
./robot2-startup.sh robot-only
```

### 2. Access Interfaces

#### Core System Interfaces
- **Dashboard**: http://localhost:3000 (Main interface)
- **RabbitMQ Management**: http://localhost:15672 (admin/admin123)
- **API Bridge**: http://localhost:8000 (REST API)
- **Video Streams**: http://localhost:8001 (Camera feeds)

#### Robot Arm Interfaces (Only available with docker-compose.arms.yml)
- **Robot 1 VNC**: http://localhost:5901 (Direct robot arm 1 control)
- **Robot 2 VNC**: http://localhost:5902 (Direct robot arm 2 control)

#### Robot Arm Prerequisites
```bash
# Install VNC viewer to access robot interfaces
sudo apt-get install vinagre  # Ubuntu/Debian
brew install vnc-viewer       # macOS

# Ensure robot hardware is connected:
# - Dobot Nova5 robots at IP addresses 192.168.100.249 and 192.168.100.248
# - USB connections available at /dev/ttyUSB0 and /dev/ttyUSB1
# - Camera devices available at /dev/video0-7 and /dev/video10-17
```

### 3. Test Order Flow
```bash
# Check system health
curl http://localhost:8000/api/health

# Create test order
curl -X POST http://localhost:8000/api/orders/create \
  -H "Content-Type: application/json" \
  -d '{"cups":[{"type":"latte","size":"regular","addons":[]}]}'

# Monitor via dashboard at http://localhost:3000
```

---

## 🤖 External Robot Setup

For production deployments or improved performance, robots can run as external processes instead of Docker containers. This provides better hardware access and reduced overhead.

### Architecture Benefits

**External Robot Advantages:**
- **Improved Performance** - No Docker overhead for robot processes
- **Better Hardware Access** - Direct USB and camera access
- **Easier Debugging** - Direct access to robot processes
- **Flexible Deployment** - Robots can run on separate machines
- **Reduced Resource Usage** - Lower memory and CPU overhead

### Installation Process

#### 1. Install Robot Dependencies

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

#### 2. Copy Robot Source Code

The installation script will attempt to copy robot source code automatically. If it fails:

```bash
# Manually copy robot source from the repository
cp -r services/robot_container/ros_ws/src/* $HOME/barns_robot_ws/src/

# Rebuild the workspace
cd $HOME/barns_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --continue-on-error
```

### Running the System

#### Robot 1 PC (Primary PC with Docker Services)

**Option 1: Start Everything Together**
```bash
# Start Docker services AND Robot 1
./robot1-startup.sh

# This starts both Docker services and Robot 1 process
```

**Option 2: Start Services Separately**
```bash
# Start only Docker services
./robot1-startup.sh docker-only

# In another terminal, start Robot 1
./robot1-startup.sh robot-only
```

**Option 3: Use Individual Scripts**
```bash
# Start Docker services
./barns.sh start

# Start Robot 1 (robot-only mode)
./robot1-startup.sh robot-only
```

#### Robot 2 PC (Secondary PC)

**Prerequisites**: Ensure Robot 1 PC Docker services are running first.

**Option 1: Robot 2 PC as Main System (with Docker services)**
```bash
# Start Docker services AND Robot 2
./robot2-startup.sh

# This starts both Docker services and Robot 2 process
```

**Option 2: Robot 2 PC connecting to Remote Docker Services**
```bash
# Set Docker host IP (Robot 1 PC IP)
export DOCKER_HOST_IP="192.168.1.100"  # Update with actual Robot 1 PC IP

# Start only Robot 2 (connects to remote Docker)
./robot2-startup.sh robot-only
```

**Option 3: Start Services Separately on Robot 2 PC**
```bash
# Start only Docker services
./robot2-startup.sh docker-only

# In another terminal, start Robot 2
./robot2-startup.sh robot-only
```

### Configuration

#### Environment Variables

**Robot 1 Configuration**
```bash
export WORKSPACE_DIR="$HOME/barns_robot_ws"           # Robot workspace
export IP_ADDRESS="192.168.200.249"                   # Robot IP address
export CAMERA_SERIAL_NUMBER="CP1Z842000YW"            # Camera serial
export USB_PORT="1-5-7"                               # Camera USB port
export ROS_DOMAIN_ID=0                                # ROS domain
```

**Robot 2 Configuration**
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

### Monitoring and Management

#### Service Status
```bash
# Check Docker services
docker compose -f docker-compose.arms.yml ps

# Check Robot processes
ps aux | grep -E "(dobot_bringup|orbbec_camera|aruco_perception)"

# Check RabbitMQ status
curl -u admin:admin123 http://localhost:15672/api/overview
```

#### View Logs
```bash
# Docker service logs
docker compose -f docker-compose.arms.yml logs -f

# Specific service logs
docker logs barns-oms -f
docker logs barns-routine -f
docker logs barns-rabbitmq -f

# Robot process logs (terminal output where robot was started)
# Robot startup scripts output to terminal directly

# ROS 2 logs
tail -f ~/.ros/log/latest/*.log
```

#### Stop Services
```bash
# Stop Robot 1 (includes Docker if started together)
./robot1-startup.sh stop

# Stop Robot 2 (includes Docker if started together)
./robot2-startup.sh stop

# Stop Docker services only
./barns.sh stop

# Stop robot processes manually
pkill -f "dobot_bringup"
pkill -f "orbbec_camera"
pkill -f "aruco_perception"
pkill -f "ros2"
```

### Network Configuration

#### Firewall Settings (Robot 1 PC)
```bash
# Allow RabbitMQ access from Robot 2 PC
sudo ufw allow from <ROBOT2_PC_IP> to any port 5672
sudo ufw allow from <ROBOT2_PC_IP> to any port 15672  # Management UI (optional)
```

#### Port Usage
- `5672`: RabbitMQ AMQP
- `15672`: RabbitMQ Management UI
- `3000`: Dashboard
- `8000`: API Bridge
- `8001`: Video Stream

### Performance Optimization

#### USB Buffer Settings
```bash
# Increase USB buffer for cameras
echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb
```

#### ROS 2 Domain Isolation
- Robot 1 uses `ROS_DOMAIN_ID=0`
- Robot 2 uses `ROS_DOMAIN_ID=1`
- This prevents ROS 2 topic conflicts between robots

#### Logging Optimization
- Most ROS nodes run with reduced logging (`__log_level:=fatal`)
- Critical services maintain `info` level logging
- Docker services use standard logging

---

## 🔧 Services Detailed

### Core Service Communication Matrix

| Service | Listens To | Sends To | Purpose |
|---------|-----------|----------|---------|
| **OMS** | `api-bridge`, `scheduler.*`, `validation.*`, `automation.*`, `routine.*` | `scheduler`, `dashboard.*` | Order lifecycle management |
| **Scheduler** | `oms`, `routine.*` | `routine`, `oms.*` | Task orchestration & dependency resolution |
| **Routine** | `scheduler` | `validation`, `automation`, `robot_arm`, `scheduler.*` | Task execution & service coordination |
| **Validation** | `routine`, `inventory.*` | `routine.*`, `oms.*`, `alerts.*` | Ingredient validation & inventory monitoring |
| **Automation** | `routine`, `system.*` | `routine.*`, `system.*` | Equipment control & automation functions |
| **Robot Arm** | `routine`, `system.*` | `routine.*`, `system.*` | Physical robot control & movement operations |
| **API Bridge** | `dashboard`, `all_services.*` | `all_services` | HTTP ↔ RabbitMQ translation |

### OMS (Order Management Service)
**Port**: Internal (RabbitMQ only)  
**Database**: PostgreSQL + Redis  
**Role**: Central order coordinator

```python
# Key Responsibilities
- Order creation and persistence
- Queue management (Redis-based)
- Status tracking through complete lifecycle
- Dashboard update broadcasting
- System-wide event coordination
```

**Message Handlers**:
- `create_order` → Store order, add to queue
- `start_order` → Send to Scheduler for processing
- `update_order_status` → Update DB and broadcast
- `complete_order` → Finalize order, notify dashboard

**Events Published**:
- `oms.order_created`, `oms.order_started`, `oms.order_completed`

### Scheduler Service
**Port**: Internal (RabbitMQ only)  
**Data**: Recipe configurations (JSON)  
**Role**: Task orchestration engine

```python
# Key Responsibilities
- Recipe loading and management
- Order decomposition into executable tasks
- Dependency resolution between tasks
- Multi-arm coordination and load balancing
- Progress tracking and status reporting
```

**Processing Logic**:
1. **Recipe Lookup**: Load drink recipe from `data/recipes.json`
2. **Task Generation**: Create individual tasks with dependencies
3. **Arm Assignment**: Distribute tasks across available robotic arms
4. **Execution Coordination**: Submit tasks to Routine service
5. **Progress Monitoring**: Track completion and handle failures

**Sample Recipe Structure**:
```json
{
  "latte": [
    {
      "action": "grind_beans",
      "assigned_arm": "Arm1",
      "depends_on": []
    },
    {
      "action": "brew_espresso", 
      "assigned_arm": "Arm1",
      "depends_on": ["grind_beans"]
    },
    {
      "action": "steam_milk",
      "assigned_arm": "Arm2", 
      "depends_on": []
    },
    {
      "action": "combine",
      "assigned_arm": "Arm1",
      "depends_on": ["brew_espresso", "steam_milk"]
    }
  ]
}
```

### Routine Service
**Port**: Internal (RabbitMQ only)  
**Role**: Task execution coordinator & service orchestrator

```python
# Key Responsibilities
- Task queue management per robotic arm
- Multi-step task execution with service coordination
- Validation and automation service integration
- Robotic arm coordination and status
- Feedback provision to Scheduler
- Error handling and recovery
```

**Architecture**:
- **Per-Arm Queues**: Separate async queues for each robotic arm (Arm1, Arm2)
- **Worker Processes**: Dedicated workers per arm for parallel execution
- **Task Configurations**: JSON-based step definitions with service calls
- **Service Coordination**: Direct communication with Validation and Automation
- **Event-Driven Feedback**: Real-time task completion reporting

**Task Execution Flow**:
1. **Task Reception**: Receives task from Scheduler service
2. **Step Processing**: Executes each step sequentially based on type:
   - `validation` steps → Call Validation service via RabbitMQ
   - `automation` steps → Call Automation service via RabbitMQ  
   - `robot` steps → Direct robotic arm control
3. **Service Coordination**: Waits for responses from called services
4. **Error Handling**: Aborts task on any step failure
5. **Feedback**: Reports completion/failure to Scheduler

**Sample Task Configuration**:
```json
{
  "brew_espresso": {
    "steps": [
      {
        "type": "validation",
        "function": "check_ingredient_availability",
        "params": {"ingredient": "coffee_beans", "amount": 1}
      },
      {
        "type": "robot",
        "function": "grind_beans",
        "params": {"grind_size": "fine"}
      },
      {
        "type": "automation",
        "function": "heat_water",
        "params": {"target_temp_c": 93, "volume_ml": 250}
      },
      {
        "type": "validation",
        "function": "update_inventory",
        "params": {"ingredient": "coffee_beans", "amount_used": 1}
      }
    ]
  }
}
```

### Validation Service
**Port**: Internal (RabbitMQ only)  
**Role**: Inventory management & quality validation

```python
# Key Responsibilities
- Ingredient availability checking
- Inventory level tracking and updates  
- Quality control validation functions
- Threshold monitoring and alerts
- Test validation for system verification
```

**Message Handlers**:
- `validate` → Execute specific validation function
- `inventory_status` → Get current inventory levels
- `inventory_refill` → Handle inventory refill operations
- `inventory_category_summary` → Get categorized inventory data

**Validation Functions**:
- `check_ingredient_availability` → Verify sufficient ingredients for recipe
- `update_inventory` → Deduct used ingredients from inventory
- `validate_test1/test2` → System integration test validations
- `check_temperature` → Temperature sensor validation
- `check_cup_present` → Cup detection validation

**Communication with Routine**:
```python
# Routine calls Validation for ingredient checks
validation_request = {
  "function": "check_ingredient_availability",
  "params": {"ingredient": "whole_milk", "amount": 2}
}

validation_response = {
  "passed": True,
  "details": "Sufficient whole_milk available: 80 >= 2",
  "data": {"ingredient": "whole_milk", "available": 80, "needed": 2}
}
```

**Events Published**:
- `validation.threshold_warning` → Low inventory alerts
- `validation.failed` → Validation failure notifications
- `validation.inventory_updated` → Inventory change events

### Automation Service
**Port**: Internal (RabbitMQ only)  
**Role**: Equipment control & automation functions

```python
# Key Responsibilities
- Coffee brewing equipment control
- Water heating and temperature management
- Milk dispensing and preparation
- Automated testing functions
- Equipment status monitoring
```

**Message Handlers**:
- `automate` → Execute specific automation function
- `list_functions` → Get available automation functions
- `stop_automation` → Emergency stop automation processes
- `health` → Service health and function availability

**Automation Functions**:
- `heat_water` → Heat water to specified temperature
- `dispense_milk` → Automated milk dispensing system
- `automation_test1/test2` → Equipment testing functions

**Communication with Routine**:
```python
# Routine calls Automation for equipment operations
automation_request = {
  "function": "heat_water",
  "params": {"target_temp_c": 93, "volume_ml": 250}
}

automation_response = {
  "success": True,
  "message": "Heated 250ml water to 93°C",
  "details": {
    "target_temperature": 93,
    "volume": 250,
    "actual_temperature": 93,
    "duration_sec": 3
  }
}
```

**Events Published**:
- `automation.started` → Function execution started
- `automation.completed` → Function execution completed
- `automation.error` → Function execution failed
- `automation.emergency_stopped` → Emergency stop triggered

### Robot Arm Service
**Port**: Internal (RabbitMQ only)  
**Role**: Physical robot control & coordinated movement operations with robot container bridge

```python
# Key Responsibilities
- Robotic arm movement control (cartesian & joint-space)
- Gripper and end-effector operations
- Pick and place task coordination
- Safety monitoring and emergency stops
- Robot calibration and homing
- Hardware/simulation mode switching
```

**Message Handlers**:
- `robot_action` → Execute specific robot function
- `emergency_stop` → Immediate safety stop for specified arm
- `calibrate` → Perform robot calibration routine
- `get_status` → Get current robot arm status
- `list_actions` → Get available robot actions

**Robot Actions**:
- `move_to_position` → Move to cartesian coordinates (x, y, z)
- `move_to_joint_position` → Move to joint angles
- `open_gripper` / `close_gripper` → Gripper control
- `pick_and_place` → Automated pick and place operation
- `home_robot` → Return to home position
- `grind_beans` / `pour_liquid` → Coffee-specific operations

**Communication with Routine**:
```python
# Routine calls Robot Arm for physical operations
robot_request = {
  "function": "move_to_position",
  "params": {"x": 300, "y": 200, "z": 150, "speed": 50},
  "arm_id": 1
}

robot_response = {
  "success": True,
  "message": "Moved to position (300, 200, 150) at 50mm/s",
  "details": {
    "arm_id": 1,
    "final_position": {"x": 300, "y": 200, "z": 150},
    "execution_time": 2.0,
    "simulation_mode": False
  }
}
```

**Deployment Modes**:
- **Simulation Mode** (`ROBOT_SIMULATION=true`): For development and testing
- **Hardware Mode** (`ROBOT_SIMULATION=false`): For physical robot integration

**Robot Function Routing**:
The Robot Arm Service intelligently routes function calls:
- **Test Functions** (`robot_test1`, `robot_test2`): Executed locally for testing
- **Real Robot Functions** (all others): Routed to robot containers via RabbitMQ
- **Robot Container Communication**: `robot_container_1` and `robot_container_2` services
- **ACTION_MAP Integration**: Direct access to robot container's ACTION_MAP functions

**Events Published**:
- `robot.action_started` → Robot action execution started
- `robot.action_completed` → Robot action execution completed
- `robot.action_error` → Robot action execution failed
- `robot.emergency_stopped` → Emergency stop activated
- `robot.calibration_completed` → Calibration procedure finished

### API Bridge Service
**Port**: 8000 (HTTP + WebSocket)  
**Role**: Protocol translator and real-time gateway

```python
# Key Responsibilities
- HTTP → RabbitMQ message translation
- WebSocket connection management
- Real-time event broadcasting
- Dashboard API endpoints
- CORS handling for web clients
```

**WebSocket Event Types**:
- `order_update` → Order status changes
- `inventory_update` → Ingredient level changes  
- `system_alert` → System warnings and errors
- `connection` → Connection status messages

### Dashboard (Frontend)
**Port**: 3000 (HTTP + WebSocket)  
**Technology**: React + Nginx  
**Role**: Real-time monitoring interface

```javascript
// Key Features
- Live order queue with drag-and-drop reordering
- Real-time progress tracking with WebSocket updates
- System health monitoring and alerts
- Camera feed integration
- Responsive design for mobile/desktop
```

**Real-time Integration**:
```javascript
// WebSocket connection management
const wsManager = new WebSocketManager();
wsManager.connect('main', '/ws', {
  onMessage: (data) => {
    if (data.type === 'order_update') {
      // Update order status in real-time
      updateOrderDisplay(data.data);
    }
  }
});
```

---

## ⚡ Scalability & Deployment

### Horizontal Scaling Capabilities

**Individual Service Scaling**:
```bash
# Scale validation service for high-volume inventory checks
docker-compose up -d --scale validation-service=3

# Scale automation service for multiple equipment operations
docker-compose up -d --scale automation-service=2

# Scale routine service for increased robotic arm coordination
docker-compose up -d --scale routine-service=2

# Scale scheduler for complex order processing
docker-compose up -d --scale scheduler-service=2
```

**Service Scaling Recommendations**:
- **Validation Service**: Scale when high inventory validation load
- **Automation Service**: Scale when multiple equipment operations run concurrently  
- **Routine Service**: Scale for increased robotic arm coordination
- **Scheduler Service**: Scale for complex recipe processing loads

**Load Distribution**:
- **RabbitMQ**: Handles message routing and load balancing
- **Redis**: Supports clustering for queue management
- **PostgreSQL**: Read replicas for query scaling
- **Nginx**: Load balancing for dashboard traffic

### Container Architecture

#### Container-Based Deployment
```yaml
# Each service runs in isolated containers
services:
  oms-service:          # Order management
  scheduler-service:    # Task orchestration  
  routine-service:      # Task execution
  validation-service:   # Inventory monitoring
  automation-service:   # Equipment control
  robot-arm-service:    # Robot control & coordination
  api-bridge:          # HTTP interface
  dashboard:           # Web interface
  
  # Infrastructure
  rabbitmq:           # Message broker
  postgres:           # Data persistence
  redis:              # Queue management
  
  # Robot Hardware (docker-compose.arms.yml only)
  robot1:             # Dobot Nova5 Robot Arm 1
  robot2:             # Dobot Nova5 Robot Arm 2
```

#### External Robot Deployment
```yaml
# Core services in containers, robots as external processes
services:
  oms-service:          # Order management
  scheduler-service:    # Task orchestration  
  routine-service:      # Task execution
  validation-service:   # Inventory monitoring
  automation-service:   # Equipment control
  robot-arm-service:    # Robot control & coordination (bridges to external)
  api-bridge:          # HTTP interface
  dashboard:           # Web interface
  
  # Infrastructure only (no robot containers)
  rabbitmq:           # Message broker
  postgres:           # Data persistence
  redis:              # Queue management

# External processes (not containerized)
external_processes:
  robot1_process:     # Native ROS 2 robot process (PC 1)
  robot2_process:     # Native ROS 2 robot process (PC 2)
```

**Deployment Options**:
- **Software-Only**: `docker-compose.yml` - For development and testing without physical robots
- **Container Hardware**: `docker-compose.arms.yml` - Complete system with robot containers
- **External Robots**: Core services in Docker + external robot processes (Production recommended)

### Production Deployment

**Environment Configuration**:
```bash
# Production environment variables
RABBITMQ_URL=amqp://admin:admin123@rabbitmq:5672/
POSTGRES_HOST=postgres
REDIS_HOST=redis
API_BRIDGE_URL=http://api-bridge:8000
VIDEO_STREAM_URL=http://video-stream:8000
```

**Health Monitoring**:
```bash
# Service health checks
docker-compose ps                    # Container status
curl http://localhost:8000/health    # API Bridge health
curl http://localhost:15672          # RabbitMQ management
```

**Resource Requirements**:
- **Development**: 4GB RAM, 2 CPU cores
- **Production**: 8GB+ RAM, 4+ CPU cores
- **High-Load**: 16GB+ RAM, 8+ CPU cores with horizontal scaling

---

## 💻 Development

### Service Documentation
Each service maintains its own detailed documentation:

- **OMS**: [`services/oms/README.md`](services/oms/README.md) - Order management, database schema, API endpoints
- **Scheduler**: [`services/scheduler/README.md`](services/scheduler/README.md) - Recipe management, task orchestration
- **Routine**: [`services/routine/README.md`](services/routine/README.md) - Task execution, robotic control
- **Validation**: [`services/validation/README.md`](services/validation/README.md) - Inventory monitoring, alerts
- **API Bridge**: [`services/api-bridge/README.md`](services/api-bridge/README.md) - HTTP API, WebSocket handling
- **Dashboard**: [`services/barns-dashboard/README.md`](services/barns-dashboard/README.md) - Frontend architecture

### Development Workflow

**Local Development**:
```bash
# Start infrastructure services only
docker-compose up rabbitmq postgres redis -d

# Run individual service in development mode
cd services/oms
python -m uvicorn app:app --reload --host 0.0.0.0 --port 8002

# Or run specific services
docker-compose up oms-service scheduler-service -d
```

**Message Testing**:
```bash
# Test RabbitMQ communication
docker exec -it barns-rabbitmq rabbitmq-diagnostics ping
docker exec -it barns-rabbitmq rabbitmqctl list_exchanges
docker exec -it barns-rabbitmq rabbitmqctl list_queues
```

**Adding New Services**:
1. Create service directory under `services/`
2. Implement RabbitMQ client using `shared/rabbitmq_client.py`
3. Register message handlers and event listeners
4. Add service to `docker-compose.yml`
5. Update API Bridge for HTTP endpoints (if needed)

### Testing Framework
```bash
# Unit tests per service
cd services/oms && python -m pytest tests/
cd services/validation && python -m pytest tests/
cd services/automation && python -m pytest tests/

# Integration testing
docker-compose -f docker-compose.test.yml up --build

# Service coordination testing
# Test validation service integration
curl -X POST http://localhost:8000/api/validation/validate \
  -H "Content-Type: application/json" \
  -d '{"function":"validate_test1","params":{}}'

# Test automation service integration  
curl -X POST http://localhost:8000/api/automation/automate \
  -H "Content-Type: application/json" \
  -d '{"function":"automation_test1","params":{}}'

# Test robot arm service integration (test functions)
curl -X POST http://localhost:8000/api/robot_arm/robot_action \
  -H "Content-Type: application/json" \
  -d '{"function":"robot_test1","params":{},"arm_id":1}'

# Test robot arm service integration (real robot functions via container)
curl -X POST http://localhost:8000/api/robot_arm/robot_action \
  -H "Content-Type: application/json" \
  -d '{"function":"home","params":{},"arm_id":1}'

# Test robot movement
curl -X POST http://localhost:8000/api/robot_arm/robot_action \
  -H "Content-Type: application/json" \
  -d '{"function":"move_to_position","params":{"x":300,"y":200,"z":150},"arm_id":1}'



# End-to-end order processing test
curl -X POST http://localhost:8000/api/orders/create \
  -H "Content-Type: application/json" \
  -d '{"cups":[{"type":"pipelinetest","size":"regular","addons":[]}]}' && \
curl -X POST http://localhost:8000/api/orders/1/start
```

---

## 🔧 Troubleshooting

### Common Issues & Solutions

#### 1. Service Communication Failures
```bash
# Check RabbitMQ connectivity
docker logs barns-rabbitmq
curl http://localhost:15672

# Verify service registration
docker exec -it barns-rabbitmq rabbitmqctl list_connections
docker exec -it barns-rabbitmq rabbitmqctl list_consumers
```

#### 2. Dashboard Not Updating
```bash
# Check WebSocket connection
curl -I http://localhost:8000/ws

# Verify API Bridge event handling
docker logs barns-api-bridge -f

# Test event flow
curl -X POST http://localhost:8000/api/orders/create
```

#### 3. Order Processing Stuck
```bash
# Check service logs in sequence
docker logs barns-oms -f
docker logs barns-scheduler -f  
docker logs barns-routine -f
docker logs barns-validation -f
docker logs barns-automation -f
docker logs barns-robot-arm -f

# Verify recipe configuration
curl http://localhost:8000/api/recipes

# Check validation service health
curl -X POST http://localhost:8000/api/validation/health

# Check automation service health  
curl -X POST http://localhost:8000/api/automation/health

# Check robot arm service health
curl -X POST http://localhost:8000/api/robot_arm/health
```

#### 4. Database Connection Issues
```bash
# PostgreSQL health
docker logs barns-postgres
docker exec -it barns-postgres psql -U barns_user -d barns_oms -c "\dt"

# Redis connectivity  
docker logs barns-redis
docker exec -it barns-redis redis-cli ping
```

#### 5. Robot Arm Connection Issues

**Container-Based Robots (docker-compose.arms.yml only)**
```bash
# Check robot container status
docker logs oms_robot_1 -f
docker logs oms_robot_2 -f

# Verify robot hardware connections
lsusb | grep -i dobot  # Should show USB connections
ping 192.168.100.249   # Robot 1 IP
ping 192.168.100.248   # Robot 2 IP

# Check camera device availability
ls -la /dev/video*     # Should show camera devices

# Access robot VNC interfaces
# Robot 1: localhost:5901
# Robot 2: localhost:5902

# Check ROS 2 domain isolation
docker exec -it oms_robot_1 bash -c "echo \$ROS_DOMAIN_ID"  # Should be 0
docker exec -it oms_robot_2 bash -c "echo \$ROS_DOMAIN_ID"  # Should be 1

# Check robot container RabbitMQ service status
docker logs oms_robot_1 | grep "Robot Container.*service started"
docker logs oms_robot_2 | grep "Robot Container.*service started"

# Test robot container communication
curl -X POST http://localhost:8000/api/robot_arm/list_actions  # Should show robot container actions
```

**External Robot Processes**
```bash
# Check if RabbitMQ is accessible from Robot 2 PC
nc -z <ROBOT1_PC_IP> 5672

# Check robot process status
./robot1-startup.sh status
./robot2-startup.sh check

# Check camera detection
lsusb | grep -i orbbec
ros2 run orbbec_camera list_devices_node

# Check USB permissions
ls -la /dev/bus/usb/

# Robot service startup issues
source $HOME/barns_robot_ws/setup_robot_env.sh
env | grep ROS

# Check workspace build
cd $HOME/barns_robot_ws
source /opt/ros/humble/setup.bash
colcon build --continue-on-error

# USB device conflicts
rm -f /dev/shm/orbbec_device_lock
ROBOT_STARTUP_DELAY=30 ./robot2-startup.sh

# View external robot logs
tail -f robot1.log  # Robot 1 PC
# Robot 2 uses terminal output

# Network connectivity test
ping <ROBOT1_PC_IP>  # From Robot 2 PC
curl http://<ROBOT1_PC_IP>:15672  # RabbitMQ Management UI
```

### Performance Monitoring
```bash
# Resource usage
docker stats

# Message queue metrics
curl http://localhost:15672/api/overview

# Database performance
docker exec -it barns-postgres psql -U barns_user -d barns_oms -c "
  SELECT query, calls, total_time, mean_time 
  FROM pg_stat_statements 
  ORDER BY total_time DESC LIMIT 10;"
```

### Network Debugging
```bash
# Container networking
docker network ls
docker network inspect barns-barns-network

# Port accessibility
netstat -tlnp | grep -E ':(3000|5672|8000|8001)'

# Inter-service communication
docker exec -it barns-api-bridge ping barns-oms
docker exec -it barns-scheduler ping rabbitmq
```

---

## 🤝 Contributing

### Architecture Guidelines
- **Microservice Boundaries**: Each service should encapsulate a single business capability
- **Event-Driven Design**: Use RabbitMQ events for loose coupling between services
- **Database per Service**: Each service manages its own data store
- **API Versioning**: Maintain backward compatibility for service interfaces
- **Error Handling**: Implement comprehensive error handling and circuit breakers

### Adding Features
1. **Design Phase**: Document service interactions and message flows
2. **Implementation**: Follow existing patterns for RabbitMQ integration
3. **Testing**: Add unit, integration, and end-to-end tests
4. **Documentation**: Update service-specific and system-level documentation
5. **Monitoring**: Add health checks and logging for new functionality

### Code Standards
- **Python Services**: Follow PEP 8, use type hints, comprehensive logging
- **Frontend**: Modern React patterns, responsive design, accessibility
- **Docker**: Multi-stage builds, minimal base images, health checks
- **Documentation**: Keep service READMEs updated with API changes

### File Structure Summary

| File | Purpose | Usage |
|------|---------|-------|
| `barns.sh` | Docker services control | `./barns.sh start/stop` |
| `install-robot-dependencies.sh` | Install robot dependencies | Run once on each robot PC |
| `robot1-startup.sh` | Robot 1 startup (Docker + Robot) | `./robot1-startup.sh [start\|docker-only\|robot-only\|stop]` |
| `robot2-startup.sh` | Robot 2 startup (Docker + Robot) | `./robot2-startup.sh [start\|docker-only\|robot-only\|stop]` |
| `docker-compose.yml` | Software-only deployment | Development/testing |
| `docker-compose.arms.yml` | Container-based robots | Full container deployment |

**Common Usage Patterns:**

1. **Single PC Setup (Robot 1 or Robot 2)**:
   ```bash
   ./robot1-startup.sh  # Everything on Robot 1 PC
   # OR
   ./robot2-startup.sh  # Everything on Robot 2 PC
   ```

2. **Distributed Setup (Robot 1 PC + Robot 2 PC)**:
   ```bash
   # Robot 1 PC:
   ./robot1-startup.sh docker-only
   ./robot1-startup.sh robot-only
   
   # Robot 2 PC:
   export DOCKER_HOST_IP="<ROBOT1_PC_IP>"
   ./robot2-startup.sh robot-only
   ```

3. **Services-Only + Robots Separately**:
   ```bash
   ./barns.sh start  # Docker services only
   ./robot1-startup.sh robot-only
   ./robot2-startup.sh robot-only
   ```

---

*This documentation reflects the event-driven, scalable architecture of BARNS with support for both containerized and external robot deployments. Each service is designed for independent scaling and reliable message-based communication through RabbitMQ.*

