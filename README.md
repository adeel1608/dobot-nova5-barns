# BARNS - Business Automation & Robotics System

## Overview

BARNS (Business Automation & Robotics) is a comprehensive coffee automation system that orchestrates the entire coffee-making process from order placement to completion. The system uses a microservices architecture to manage orders, coordinate robotic arms, perform quality validation, and provide real-time monitoring capabilities.

## System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Dashboard     │    │  Video Stream   │    │   Validation    │
│  (Frontend UI)  │    │   (Cameras)     │    │   (Quality)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
         ┌─────────────────────────────────────────────────┐
         │              Order Management Service (OMS)     │
         │                 (Central Orchestrator)          │
         └─────────────────────┬───────────────────────────┘
                               │
         ┌─────────────────────┼───────────────────────────┐
         │                     │                           │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Scheduler     │____│    Routine      │____│   Robot Arm     │
│ (Task Manager)  │    │  (Executor)     │    │  (Hardware)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Microservices

### 🎯 [Order Management Service (OMS)](./services/oms/README.md)
**Port: 8001** | **Purpose: Central Orchestrator**
- Manages complete order lifecycle from creation to completion
- Provides REST API for order operations and real-time WebSocket updates
- Coordinates with Scheduler service and maintains PostgreSQL database
- Handles system alerts and emergency controls

### 📋 [Scheduler Service](./services/scheduler/README.md)
**Port: 8000** | **Purpose: Task Coordination**
- Breaks down orders into individual tasks based on drink recipes
- Coordinates parallel execution across multiple robotic arms
- Manages task dependencies and resource allocation
- Reports completion/failure status back to OMS

### 🔧 [Routine Service](./services/routine/README.md)
**Port: 8002** | **Purpose: Task Execution**
- Executes individual robotic tasks by coordinating validation and robot operations
- Manages multi-step task configurations from `config/tasks.json`
- Integrates with Validation service for quality checks
- Publishes step-by-step execution events

### ✅ [Validation Service](./services/validation/README.md)
**Port: 8003** | **Purpose: Quality Control**
- Performs quality control and verification checks
- Interfaces with sensors and AI systems for ingredient/equipment validation
- Ensures safety conditions and quality standards are met
- Provides pass/fail results with detailed measurements

### 📹 [Video Stream Service](./services/video-stream/README.md)
**Port: 8004** | **Purpose: Visual Monitoring**
- Manages multiple camera feeds for real-time monitoring
- Provides MJPEG video streams and still image capture
- Supports computer vision integration for advanced monitoring
- Handles graceful fallback when cameras are unavailable

### 🖥️ [Dashboard Service](./services/barns-dashboard/README.md)
**Port: 3000** | **Purpose: User Interface**
- React-based web application for system monitoring and control
- Real-time order management with drag-and-drop queue reordering
- Live video feeds and system status monitoring
- Alert management and analytics dashboard

## Quick Start

### Prerequisites
- **Docker & Docker Compose**: For containerized deployment
- **Node.js 18+**: For dashboard development
- **Python 3.9+**: For backend services development
- **PostgreSQL**: Database for order management
- **Redis**: Queue management and caching

### Development Setup

1. **Clone Repository**:
   ```bash
   git clone <repository-url>
   cd BARNS
   ```

2. **Start All Services**:
   ```bash
   docker-compose up -d
   ```

3. **Verify Services**:
   ```bash
   # Check service status
   docker-compose ps
   
   # View logs
   docker-compose logs -f
   ```

4. **Access Dashboard**:
   Open [http://localhost:3000](http://localhost:3000) in your browser

### Service URLs
- **Dashboard**: http://localhost:3000
- **OMS API**: http://localhost:8001
- **Scheduler API**: http://localhost:8000  
- **Routine API**: http://localhost:8002
- **Validation API**: http://localhost:8003
- **Video Stream API**: http://localhost:8004

## Development Workflow

### Working on Individual Services

Each service has its own development environment and documentation:

```bash
# OMS Service
cd services/oms
pip install -r requirements.txt
uvicorn app:app --reload --port 8001

# Scheduler Service  
cd services/scheduler
pip install -r requirements.txt
uvicorn app:app --reload --port 8000

# Dashboard
cd services/barns-dashboard
npm install
npm start
```

### Database Management

```bash
# Access PostgreSQL
docker-compose exec postgres psql -U postgres -d barns_db

# Reset database (clears all data)
docker-compose exec postgres psql -U postgres -d barns_db -c "
TRUNCATE TABLE alerts, events, task_steps, tasks, order_items, orders RESTART IDENTITY CASCADE;"

# Access Redis
docker-compose exec redis redis-cli
```

### Testing

```bash
# Run individual service tests
cd services/oms && python -m pytest
cd services/scheduler && python -m pytest  
cd services/barns-dashboard && npm test

# Integration testing
curl -X POST "http://localhost:8001/orders/" \
  -H "Content-Type: application/json" \
  -d '{"status": "queued", "cups": [{"type": "Latte", "size": "regular"}]}'
```

## System Features

### ✨ Core Capabilities
- **Real-time Order Management**: Create, track, and manage coffee orders
- **Intelligent Task Scheduling**: Parallel execution across multiple robotic arms
- **Quality Control Integration**: AI-powered validation and sensor monitoring
- **Live Video Monitoring**: Multi-camera feeds with computer vision capabilities
- **Event-Driven Architecture**: Real-time updates and notifications
- **Comprehensive Analytics**: Performance metrics and system insights

### 🔄 Workflow Example
1. **Order Creation**: User creates a Latte order via dashboard
2. **Queue Management**: Order appears in queue, can be reordered via drag-and-drop
3. **Processing Start**: User clicks "Start" button to begin processing
4. **Task Breakdown**: Scheduler breaks order into tasks (pick_cup, pull_espresso, steam_milk, etc.)
5. **Parallel Execution**: Tasks execute in parallel across arms based on dependencies
6. **Quality Validation**: Each step validated for safety and quality standards
7. **Real-time Updates**: Dashboard shows live progress and video feeds
8. **Completion**: Order marked complete when all tasks finish successfully

## Configuration

### Environment Variables

```env
# Database Configuration
DB_NAME=barns_db
DB_USER=postgres
DB_PASSWORD=postgres
DB_HOST=postgres
DB_PORT=5432

# Redis Configuration  
REDIS_HOST=redis
REDIS_PORT=6379

# Service URLs
OMS_URL=http://oms:8001
SCHEDULER_URL=http://scheduler:8000
ROUTINE_URL=http://routine:8002
VALIDATION_URL=http://validation:8003
VIDEO_STREAM_URL=http://video-stream:8004
```

### Recipe Configuration

Drink recipes are defined in `services/scheduler/data/recipes.json`:

```json
{
  "Latte": [
    { "action": "pick_cup", "assigned_arm": "Arm1" },
    { "action": "pull_espresso", "assigned_arm": "Arm1", "depends_on": ["pick_cup"] },
    { "action": "steam_milk", "assigned_arm": "Arm2", "depends_on": ["pick_cup"] },
    { "action": "pour_milk", "assigned_arm": "Arm2", "depends_on": ["pull_espresso", "steam_milk"] },
    { "action": "serve", "assigned_arm": "Arm1", "depends_on": ["pour_milk"] }
  ]
}
```

## Troubleshooting

### Common Issues

**🔴 Orders not starting when clicking Start button**
- Check if backend services are running: `docker-compose ps`
- Verify scheduler service logs: `docker-compose logs scheduler`
- Ensure OMS can communicate with Scheduler

**🔴 Dashboard not updating in real-time**
- Check WebSocket connections in browser dev tools
- Verify OMS WebSocket endpoint is accessible
- Check for CORS issues in browser console

**🔴 Video streams not loading**
- Verify camera permissions and availability
- Check video-stream service logs: `docker-compose logs video-stream`
- Test camera access: `curl http://localhost:8004/cameras`

**🔴 Tasks failing with "Unknown function" errors**
- Check routine service task configurations in `config/tasks.json`
- Ensure task names in recipes match available routine functions
- Verify validation service is accessible

### Debugging Commands

```bash
# View all service logs
docker-compose logs -f

# Check service health
curl http://localhost:8001/system/status  # OMS
curl http://localhost:8000/status         # Scheduler  
curl http://localhost:8004/status         # Video Stream

# Database inspection
docker-compose exec postgres psql -U postgres -d barns_db -c "SELECT * FROM orders;"

# Redis queue inspection  
docker-compose exec redis redis-cli LLEN order_queue
```

## Contributing

### Adding New Services

1. Create service directory under `services/`
2. Add service to `docker-compose.yml`
3. Create comprehensive `README.md` following existing patterns
4. Add integration tests and documentation
5. Update main architecture documentation

### Code Standards

- **Python**: Follow PEP 8, use type hints, comprehensive docstrings
- **TypeScript/React**: Use functional components, proper type definitions
- **API Design**: RESTful APIs with clear error handling and status codes
- **Documentation**: Comprehensive README files with examples and troubleshooting

 