# BARNS Inventory Management System

## Overview

The BARNS Inventory Management System provides complete tracking and management of ingredients (milk, beans, cups, syrup) across the entire coffee automation workflow. The system automatically monitors ingredient levels, sends threshold warnings, and handles refill operations.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Validation     │    │      OMS       │    │   Dashboard     │
│   Service       │    │   Service      │    │    (React)      │
│  (Port 8003)    │    │  (Port 8000)   │    │  (Port 3000)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │ 1. Threshold Warning  │                       │
         ├──────────────────────►│                       │
         │                       │ 2. Alert Broadcast   │
         │                       ├──────────────────────►│
         │                       │                       │
         │                       │ 3. Refill Request    │
         │ 4. Refill Ack         │◄──────────────────────┤
         │◄──────────────────────┤                       │
         │                       │ 5. Refill Confirm    │
         │                       ├──────────────────────►│
```

## Components

### 1. Validation Service (`services/validation/app.py`)

**Responsibilities:**
- Track ingredient inventory levels
- Monitor consumption during validation checks
- Send threshold warnings to OMS when levels are low
- Handle refill acknowledgments from Dashboard
- Provide inventory status

**Key Features:**
- In-memory inventory tracking (easily replaceable with database)
- Configurable thresholds (low, medium, high)
- Automatic threshold warning generation
- RESTful API for inventory operations

**API Endpoints:**
```
POST /validate                    # Run validation functions
POST /inventory/refill           # Handle refill acknowledgment
GET  /inventory/status           # Get current inventory levels
POST /inventory/check            # Check ingredient availability
GET  /health                     # Health check
```

### 2. OMS Service (`services/oms/app.py`)

**Responsibilities:**
- Receive threshold warnings from Validation Service
- Create alerts and broadcast to Dashboard
- Handle refill requests from Dashboard
- Forward refill requests to Validation Service
- Manage WebSocket connections for real-time updates

**Key Features:**
- WebSocket broadcasting for real-time updates
- Alert management and logging
- Inventory refill coordination
- Database event logging

**API Endpoints:**
```
POST /inventory/threshold-warning # Receive threshold warnings
POST /inventory/refill           # Handle refill requests
GET  /inventory/status           # Get inventory status
```

### 3. Dashboard (`services/barns-dashboard/`)

**Responsibilities:**
- Display real-time inventory status
- Show threshold warning alerts
- Provide refill interface for operators
- Real-time updates via WebSocket

**Key Features:**
- **InventoryPanel**: Visual inventory status with refill buttons
- **AlertPanel**: Threshold warning alerts with quick refill actions
- **Real-time Updates**: WebSocket integration for live data
- **Visual Indicators**: Color-coded status (🟢 High, 🟡 Medium, 🔴 Low)

## Workflow

### 1. Normal Operation
1. Validation Service tracks ingredient levels
2. During order processing, ingredients are consumed
3. Levels are automatically updated

### 2. Threshold Warning Flow
1. **Validation Service** detects low ingredient level
2. **Validation Service** → **OMS**: Sends threshold warning
3. **OMS** creates alert and logs event
4. **OMS** → **Dashboard**: Broadcasts alert via WebSocket
5. **Dashboard** shows alert in AlertPanel and updates InventoryPanel

### 3. Refill Flow
1. **Operator** clicks refill button in Dashboard
2. **Dashboard** → **OMS**: Sends refill request
3. **OMS** logs refill event
4. **OMS** → **Validation Service**: Forwards refill request
5. **Validation Service** updates inventory levels
6. **OMS** → **Dashboard**: Broadcasts refill confirmation
7. **Dashboard** updates inventory status and resolves alerts

## Installation & Setup

### Prerequisites
- Python 3.8+
- Node.js 16+
- npm

### Quick Start
```bash
# Clone the repository
git clone <repository-url>
cd BARNS

# Start all services
python start_services.py
```

### Manual Setup
```bash
# 1. Install Python dependencies
pip install fastapi uvicorn requests httpx psycopg2-binary redis

# 2. Install Dashboard dependencies
cd services/barns-dashboard
npm install
cd ../..

# 3. Start Validation Service
cd services/validation
python -m uvicorn app:app --port 8003 &

# 4. Start OMS Service
cd ../oms
python -m uvicorn app:app --port 8000 &

# 5. Start Dashboard
cd ../barns-dashboard
npm start &
```

## Testing

### Complete Workflow Test
```bash
python test_complete_inventory_workflow.py
```

This test demonstrates:
- Ingredient consumption simulation
- Threshold warning generation
- Refill workflow (Dashboard → OMS → Validation)
- Real-time status updates

### Individual API Tests
```bash
# Test threshold warnings
python test_inventory_api.py

# Test validation service directly
curl -X POST http://localhost:8003/inventory/check \
  -H "Content-Type: application/json" \
  -d '{"ingredient": "milk", "amount_needed": 10}'

# Test refill via dashboard
curl -X POST http://localhost:8000/inventory/refill \
  -H "Content-Type: application/json" \
  -d '{"ingredient": "milk"}'
```

## Configuration

### Inventory Thresholds (Validation Service)
```python
INVENTORY_LEVELS = {
    "milk": {"level": 100, "threshold_low": 20, "threshold_medium": 50},
    "beans": {"level": 80, "threshold_low": 15, "threshold_medium": 40},
    "cup": {"level": 150, "threshold_low": 30, "threshold_medium": 75},
    "syrup": {"level": 60, "threshold_low": 10, "threshold_medium": 30}
}
```

### Service URLs
- **Validation Service**: `http://localhost:8003`
- **OMS Service**: `http://localhost:8000`
- **Dashboard**: `http://localhost:3000`

## API Documentation

### Validation Service API

#### POST `/inventory/refill`
Handle refill acknowledgment from Dashboard.

**Request:**
```json
{
  "ingredient": "milk"
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Ingredient milk refilled successfully",
  "ingredient": "milk",
  "old_level": 15,
  "new_level": 100,
  "refilled_at": "2024-01-15T10:30:00Z"
}
```

#### GET `/inventory/status`
Get current inventory status.

**Response:**
```json
{
  "status": "success",
  "inventory": {
    "milk": {
      "level": "low",
      "actual_amount": 15,
      "last_refilled": "2024-01-15T10:30:00Z"
    }
  }
}
```

### OMS Service API

#### POST `/inventory/threshold-warning`
Receive threshold warning from Validation Service.

**Request:**
```json
{
  "ingredient": "milk",
  "severity": "low"
}
```

#### POST `/inventory/refill`
Handle refill request from Dashboard.

**Request:**
```json
{
  "ingredient": "milk"
}
```

## Dashboard Components

### InventoryPanel
- **Location**: Sidebar of main dashboard
- **Features**:
  - Real-time inventory status display
  - Individual refill buttons per ingredient
  - Quick actions (Refill All Low, Refill All)
  - Visual status indicators with emojis
  - Last refilled timestamps

### AlertPanel
- **Location**: Sidebar of main dashboard
- **Features**:
  - Real-time threshold warning alerts
  - Quick refill buttons directly from alerts
  - Alert acknowledgment
  - Visual alert categorization

## Monitoring & Logging

### System Logs
All inventory operations are logged with:
- Timestamp
- Service name
- Operation type
- Ingredient details
- Success/failure status

### WebSocket Events
Real-time events broadcasted:
- `threshold_warning`: Ingredient level warnings
- `inventory_refilled`: Refill confirmations
- `order_started`, `order_completed`: Order status updates

## Troubleshooting

### Common Issues

1. **Services not starting**
   ```bash
   # Check if ports are available
   netstat -an | grep :8000
   netstat -an | grep :8003
   netstat -an | grep :3000
   ```

2. **WebSocket connection issues**
   - Ensure OMS service is running
   - Check browser console for connection errors
   - Verify CORS settings

3. **Threshold warnings not appearing**
   - Check Validation Service logs
   - Verify OMS service is receiving warnings
   - Check WebSocket connection in Dashboard

### Debug Mode
Start services with debug logging:
```bash
# Validation Service
python -m uvicorn app:app --port 8003 --log-level debug

# OMS Service
python -m uvicorn app:app --port 8000 --log-level debug
```

## Future Enhancements

1. **Database Integration**: Replace in-memory storage with persistent database
2. **Authentication**: Add user authentication and role-based access
3. **Historical Analytics**: Track inventory usage patterns
4. **Predictive Alerts**: ML-based consumption prediction
5. **Mobile App**: Mobile interface for inventory management
6. **Integration**: Connect with actual hardware sensors

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## License

[Your License Here] 