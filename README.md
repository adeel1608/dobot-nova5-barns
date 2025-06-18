# BARNS - Business Automation & Robotics Network System

![BARNS Logo](BARNS%20Logo.png)

A comprehensive microservices-based automation platform for business operations, robotics control, and real-time monitoring with AI integration.

## 🚀 Overview

BARNS is a modular, scalable platform that combines:
- **Order Management System (OMS)** - Complete order lifecycle management
- **Video Streaming** - Real-time camera feeds with lightweight test patterns
- **Automation Services** - Workflow automation and task scheduling
- **Real-time Dashboard** - Live monitoring and control interface
- **Message Queue Architecture** - RabbitMQ-based inter-service communication

## 📋 Table of Contents

- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Services Overview](#services-overview)
- [API Documentation](#api-documentation)
- [Development](#development)
- [Deployment](#deployment)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## 🏗️ Architecture

### Microservices Architecture
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Dashboard     │    │   API Bridge    │    │  Video Stream   │
│   (Port 3000)   │    │   (Port 8000)   │    │   (Port 8001)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │    RabbitMQ     │
                    │   (Port 5672)   │
                    │  Management UI  │
                    │  (Port 15672)   │
                    └─────────────────┘
                                 │
    ┌───────────┬────────────────┼────────────────┬────────────┐
    │           │                │                │            │
┌───▼───┐   ┌───▼──────┐     ┌───▼──────┐     ┌───▼─────┐ ┌───▼───┐
│  OMS  │   │Validation│     │Automation│     │Scheduler│ │Routine│
│Service│   │ Service  │     │ Service  │     │ Service │ │Service│
└───────┘   └──────────┘     └──────────┘     └─────────┘ └───────┘
     │
┌────▼──────┐                              ┌────────────┐
│PostgreSQL │                              │    Redis   │
│(Port 5432)│                              │(Port 6379) │
└───────────┘                              └────────────┘
```

### Message Flow
- **HTTP APIs** → API Bridge → RabbitMQ → Services
- **Real-time Events** → WebSocket → Dashboard
- **Video Streams** → Direct HTTP → Dashboard
- **Service Communication** → RabbitMQ Exchanges

## 🚀 Quick Start

### Prerequisites
- Docker Desktop
- Docker Compose
- 8GB+ RAM recommended

### 1. Clone & Start
```bash
git clone <repository-url>
cd BARNS
docker-compose -f docker-compose.rabbitmq.yml up --build -d
```

### 2. Access Services
- **Dashboard**: http://localhost:3000
- **RabbitMQ Management**: http://localhost:15672 (admin/admin123)
- **Video Stream**: http://localhost:8001
- **API**: http://localhost:8000

### 3. Verify Status
```bash
# Check all services
docker-compose -f docker-compose.rabbitmq.yml ps

# Check video stream
curl http://localhost:8001/status

# Check cameras
curl http://localhost:8001/cameras
```

## 🔧 Services Overview

| Service | Port | Description | Technology |
|---------|------|-------------|------------|
| **Dashboard** | 3000 | Web UI for monitoring and control | React + Nginx |
| **API Bridge** | 8000 | HTTP to RabbitMQ translator | FastAPI + Python |
| **Video Stream** | 8001 | Camera feeds with test patterns | OpenCV + FastAPI |
| **OMS** | - | Order management system | Python + PostgreSQL |
| **Validation** | - | Data validation service | Python |
| **Automation** | - | Workflow automation | Python |
| **Scheduler** | - | Task scheduling | Python |
| **Routine** | - | Routine task management | Python |

### External Services
- **RabbitMQ**: Message broker (5672, 15672)
- **PostgreSQL**: OMS database (5432)
- **Redis**: Queue management (6379)

## 📡 API Documentation

### Video Stream API
```bash
# Get camera status
GET /status

# List all cameras
GET /cameras

# Live video stream
GET /stream/{camera_id}

# Still image capture
GET /still/{camera_id}

# Debug information
GET /debug
```

### API Bridge Endpoints
```bash
# Order management
POST /api/orders/create
GET /api/orders/{order_id}
PUT /api/orders/{order_id}

# System status
GET /api/status

# Service health
GET /api/health
```

## 💻 Development

### Environment Setup
```bash
# Install dependencies
pip install -r requirements.txt

# Development mode (single service)
cd services/video-stream
python -m uvicorn app:app --reload --host 0.0.0.0 --port 8001
```

### Service Development
Each service has its own README.md with specific development instructions:
- [`services/video-stream/README.md`](services/video-stream/README.md)
- [`services/oms/README.md`](services/oms/README.md)
- [`services/automation/README.md`](services/automation/README.md)
- [See each service directory for details]

### Testing
```bash
# Test video stream
curl http://localhost:8001/debug

# Test specific camera
curl http://localhost:8001/stream/test_pattern

# Test API bridge
curl http://localhost:8000/api/health
```

## 🚀 Deployment

### Production Docker
```bash
# Production build
docker-compose -f docker-compose.rabbitmq.yml build --no-cache

# Start all services
docker-compose -f docker-compose.rabbitmq.yml up -d

# Scale specific services
docker-compose -f docker-compose.rabbitmq.yml up -d --scale validation-service=3
```

### Configuration
- **Environment Variables**: Set in docker-compose.rabbitmq.yml
- **RabbitMQ**: Default credentials admin/admin123
- **Database**: PostgreSQL with persistent volumes
- **Video**: Supports real webcams and test patterns

### Monitoring
```bash
# View all logs
docker-compose -f docker-compose.rabbitmq.yml logs -f

# View specific service
docker logs barns-video-stream -f

# Service status
docker-compose -f docker-compose.rabbitmq.yml ps
```

## 🔧 Troubleshooting

### Common Issues

#### Dashboard Connection Errors
```bash
# Clear browser cache completely
# Check port 8001 is accessible
curl http://localhost:8001/cameras
```

#### Video Stream Issues
```bash
# Check video service logs
docker logs barns-video-stream

# Verify camera endpoints
curl http://localhost:8001/debug
```

#### RabbitMQ Connection Issues
```bash
# Check RabbitMQ status
curl http://localhost:15672

# Restart message services
docker-compose -f docker-compose.rabbitmq.yml restart validation-service
```

#### Performance Issues
- Video streams use lightweight test patterns (10 FPS)
- For better performance, reduce video quality in configuration
- Scale services horizontally as needed

### Debug Commands
```bash
# Full system status
docker-compose -f docker-compose.rabbitmq.yml ps
docker system df

# Network debugging
docker network ls
docker network inspect barns-barns-network

# Volume debugging
docker volume ls
docker volume inspect barns_postgres_data
```

