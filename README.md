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

---

## 📋 Table of Contents

- [Architecture Overview](#architecture-overview)
- [Communication Patterns](#communication-patterns)
- [Service Data Flows](#service-data-flows)
- [Quick Start](#quick-start)
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
    participant ARM as Robot Arm
    participant S as Scheduler

    Note over RT: Process task steps sequentially
    
    RT->>V: validate (check_ingredient_availability)
    V->>RT: {passed: true, details: "sufficient ingredients"}
    
    RT->>ARM: robot (grind_beans)
    ARM->>RT: {success: true, message: "beans ground"}
    
    RT->>A: automate (heat_water)
    A->>RT: {success: true, details: "water heated to 93°C"}
    
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
                                        ┌─────────────┐   ┌─────────────┐
                                        │ Validation  │   │ Automation  │
                                        └─────────────┘   └─────────────┘
                                        │             │   │             │
                                   Inventory       Test   Equipment    Test
                                  Management    Functions   Control  Functions
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
```bash
# Clone repository
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS

# Start all services (RabbitMQ-based architecture)
docker-compose up --build -d

# Verify services are healthy
docker-compose ps
```

### 2. Access Interfaces
- **Dashboard**: http://localhost:3000 (Main interface)
- **RabbitMQ Management**: http://localhost:15672 (admin/admin123)
- **API Bridge**: http://localhost:8000 (REST API)
- **Video Streams**: http://localhost:8001 (Camera feeds)

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

## 🔧 Services Detailed

### Core Service Communication Matrix

| Service | Listens To | Sends To | Purpose |
|---------|-----------|----------|---------|
| **OMS** | `api-bridge`, `scheduler.*`, `validation.*`, `automation.*`, `routine.*` | `scheduler`, `dashboard.*` | Order lifecycle management |
| **Scheduler** | `oms`, `routine.*` | `routine`, `oms.*` | Task orchestration & dependency resolution |
| **Routine** | `scheduler` | `validation`, `automation`, `scheduler.*` | Task execution & service coordination |
| **Validation** | `routine`, `inventory.*` | `routine.*`, `oms.*`, `alerts.*` | Ingredient validation & inventory monitoring |
| **Automation** | `routine`, `system.*` | `routine.*`, `system.*` | Equipment control & automation functions |
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
```yaml
# Each service runs in isolated containers
services:
  oms-service:          # Order management
  scheduler-service:    # Task orchestration  
  routine-service:      # Task execution
  validation-service:   # Inventory monitoring
  api-bridge:          # HTTP interface
  dashboard:           # Web interface
  
  # Infrastructure
  rabbitmq:           # Message broker
  postgres:           # Data persistence
  redis:              # Queue management
```

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

# Verify recipe configuration
curl http://localhost:8000/api/recipes

# Check validation service health
curl -X POST http://localhost:8000/api/validation/health

# Check automation service health  
curl -X POST http://localhost:8000/api/automation/health
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

---

*This documentation reflects the event-driven, scalable architecture of BARNS. Each service is designed for independent scaling and reliable message-based communication through RabbitMQ.*

