# Validation Service

## Brief Overview

The Validation Service is the inventory management and computer vision backbone of BARNS. It tracks ingredient levels, validates order feasibility, performs automated coffee beans detection using CV, detects cup presence, manages stock thresholds, and broadcasts real-time inventory updates to the dashboard.

## Key Features

- **Real-Time Inventory Tracking**: PostgreSQL-backed ingredient management with in-memory caching
- **Automated Coffee Detection**: Periodic CV-based coffee beans level detection (every 10 minutes)
- **Cup Detection**: RF-DETR-based cup presence validation (RTSP + `rfdetr`/`supervision`)
- **Pre-Order Validation**: Check ingredient availability before order processing
- **Threshold Alerts**: Automatic low-stock warnings (low/empty levels)
- **Inventory Refill**: Manual and CV-assisted refill operations
- **Stock Analytics**: Category summaries, stock level statistics, filtering by level
- **Event Broadcasting**: Real-time updates pushed to API Bridge/Dashboard
- **Database Persistence**: Historical tracking and audit trail

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│          Validation Service (AsyncIO + RabbitMQ)            │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │          Request Handlers (app.py)                 │    │
│  │  - Inventory CRUD                                  │    │
│  │  - Pre-check validation                            │    │
│  │  - Stock analytics                                 │    │
│  │  - Cup/Coffee detection                            │    │
│  └────────────┬────────────────────────┬──────────────┘    │
│               │                        │                    │
│               ↓                        ↓                    │
│  ┌────────────────────┐    ┌──────────────────────────┐   │
│  │  MainValidation    │    │   InventoryManager       │   │
│  │  - Orchestration   │───→│   - Business Logic       │   │
│  │  - CV Coordination │    │   - Cache Management     │   │
│  └────────────┬───────┘    └──────────┬───────────────┘   │
│               │                       │                    │
│               ↓                       ↓                    │
│  ┌────────────────────┐    ┌──────────────────────────┐   │
│  │  CV Detection      │    │   DatabaseClient         │   │
│  │  - Coffee (Camera) │    │   (PostgreSQL)           │   │
│  │  - Cup (RF-DETR)   │    │   - CRUD Operations      │   │
│  └────────────────────┘    └──────────────────────────┘   │
│               │                       │                    │
└───────────────┼───────────────────────┼────────────────────┘
                │                       │
                ↓                       ↓
       ┌────────────────┐      ┌────────────────┐
       │   IP Cameras   │      │  PostgreSQL    │
       │   (RTSP/HTTP)  │      │   Database     │
       └────────────────┘      └────────────────┘
                                        
       RabbitMQ Communication:
       ← Receives: Requests from Scheduler, Routine, API Bridge
       → Sends: Inventory updates, alerts, responses
```

### Component Breakdown

1. **ValidationServiceApp** (`app.py`): RabbitMQ handler registration and message routing
2. **MainValidation** (`main_validation.py`): Business logic orchestration
3. **InventoryManager** (`inventory_manager.py`): Inventory CRUD and calculations
4. **DatabaseClient** (`db_client.py`): PostgreSQL operations
5. **ProductionCoffeeDetector** (`coffee_detection/`): CV-based coffee level detection
6. **CupDetector** (`cup_detection/`): RF-DETR-based cup presence detection

## Setup & Installation

### Prerequisites

- Python 3.8+
- PostgreSQL database
- RabbitMQ server
- IP cameras (RTSP/HTTP) for coffee and cup detection
- Docker (for containerized deployment)

### Local Development

```bash
# Navigate to service directory
cd services/validation

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export RABBITMQ_URL="amqp://admin:admin123@localhost:5672/"
export POSTGRES_HOST="localhost"
export POSTGRES_PORT="5432"
export POSTGRES_DB="barns_validation"
export POSTGRES_USER="validation_user"
export POSTGRES_PASSWORD="validation_pass"
export PYTHONPATH="/path/to/barns"

# Initialize database schema
psql -h localhost -U validation_user -d barns_validation -f validation_schema.sql

# Run service
python app.py
```

### Docker Deployment

Automatically deployed via `docker-compose.yml`:

```bash
# Start validation service and dependencies
docker-compose up -d postgres rabbitmq validation-service

# View logs
docker-compose logs -f validation-service

# Check periodic detection
docker-compose logs -f validation-service | grep "detection"
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection string |
| `POSTGRES_HOST` | `postgres` | PostgreSQL host |
| `POSTGRES_PORT` | `5432` | PostgreSQL port |
| `POSTGRES_DB` | `barns_validation` | Database name |
| `POSTGRES_USER` | `validation_user` | Database user |
| `POSTGRES_PASSWORD` | `validation_pass` | Database password |
| `DETECTION_INTERVAL_SECONDS` | `600` | Coffee detection interval (10 min) |
| `ENABLE_PERIODIC_DETECTION` | `true` | Enable/disable periodic CV detection |
| `DETECTION_TIMEOUT_SECONDS` | `30` | Detection operation timeout |
| `MAX_DETECTION_WORKERS` | `3` | Thread pool size for CV operations |
| `PYTHONPATH` | `/app` | Python module search path |

### Inventory Rules (`inventory_rules.json`)

Defines thresholds and capacities:

```json
{
  "coffee_beans": {
    "unit": "grams",
    "subtypes": {
      "regular": {
        "max_capacity": 1000,
        "warning_threshold": 300,
        "critical_threshold": 100,
        "low_threshold": 200
      }
    }
  },
  "cups": {
    "unit": "count",
    "subtypes": {
      "small": {"max_capacity": 50, "warning_threshold": 15, "critical_threshold": 5},
      "medium": {"max_capacity": 50, "warning_threshold": 15, "critical_threshold": 5},
      "large": {"max_capacity": 50, "warning_threshold": 15, "critical_threshold": 5}
    }
  }
}
```

### Coffee Detection Config (`coffee_detection/detection_config.json`)

```json
{
  "snapshot_url": "http://user:pass@192.168.200.88/cgi-bin/snapshot.cgi",
  "frames_to_sample": 5,
  "coffee_threshold": 5.0,
  "enable_debug": true,
  "max_debug_frames": 10,
  "cleanup_after_detection": true,
  "roi_points": [[126, 380], [928, 273], ...]
}
```

### Cup Detection Config (`cup_detection/config.py`)

```python
# Store secrets outside git; prefer injecting RTSP URL via env or secret manager.
RTSP_URL = "rtsp://<user>:<pass>@<camera-ip>:554/stream1"

# RF-DETR local model settings
RFDETR_VARIANT = "large"          # "base", "medium", "large"
RFDETR_CONFIDENCE = 0.10          # 0..1
RFDETR_MODEL_PATHS = {
  "large": "models/rf-detr-large.pth",
  "base": "models/rf-detr-base.pth",
  "medium": "models/rf-detr-medium.pth"
}

# Runtime behavior
FRAMES = 1
DEBUG_MODE = True
SAVE_FRAMES = True
```

## API/Endpoints

### Transport & Message Envelope

This service **does not expose an HTTP API**. All request/response actions below are **RabbitMQ RPC** actions handled by `ValidationServiceApp` (`services/validation/app.py`).

The handler methods accept both of these shapes (the service normalizes them internally):

```json
{
  "request_id": "req-123",
  "client_type": "scheduler",
  "payload": { "..." : "..." }
}
```

```json
{
  "request_id": "req-123",
  "client_type": "scheduler",
  "...": "payload fields at top-level"
}
```

### Inventory Management

#### Action: `inventory_status`
Get inventory levels (all, by type, or specific).

**Request:**
```json
{
  "request_id": "req-001",
  "payload": {
    "ingredient_type": "coffee_beans",
    "subtype": "regular"
  }
}
```

**Response:**
```json
{
  "passed": true,
  "details": {
    "coffee_beans": {
      "regular": {
        "amount": 750,
        "percentage": 75,
        "status": "high",
        "warning_threshold": 300,
        "critical_threshold": 100,
        "max_capacity": 1000
      }
    }
  }
}
```

#### Action: `inventory_refill`
Refill inventory (with CV detection for coffee beans).

**Request:**
```json
{
  "request_id": "req-002",
  "payload": {
    "ingredient_type": "coffee_beans",
    "subtype": "regular"
  }
}
```

**Response:**
```json
{
  "passed": true,
  "details": {
    "coffee_beans_message": "Refilled with 85% detected",
    "coffee_beans_percentage": 85
  }
}
```

#### Action: `pre_check`
Validate ingredients before order processing.

**Request:**
```json
{
  "request_id": "req-003",
  "client_type": "scheduler",
  "payload": {
    "items": [
      {
        "drink_name": "latte",
        "cup_id": "medium",
        "ingredients": {
          "coffee_beans": {"type": "regular", "amount": 2},
          "milk": {"type": "whole", "amount": 200}
        }
      }
    ]
  }
}
```

**Response:**
```json
{
  "passed": true,
  "details": {
    "latte": {
      "status": true,
      "cup": {"type": "medium", "status": true},
      "coffee_beans": {"type": "regular", "status": true},
      "milk": {"type": "whole", "status": true}
    }
  }
}
```

#### Action: `update_inventory`
Update inventory after consumption.

**Request:**
```json
{
  "request_id": "req-004",
  "client_type": "scheduler",
  "payload": {
    "ingredients": [
      {
        "coffee_beans": {"type": "regular", "amount": 18}
      }
    ]
  }
}
```

**Response:**
```json
{
  "passed": true,
  "details": {
    "coffee_beans": {
      "type": "regular",
      "updated_amount": -18,
      "status": "no_warning"
    }
  }
}
```

#### Action: `update_limits`
Update inventory rule limits (writes to `inventory_rules.json` and updates in-memory cache; may adjust DB `current_amount` if lowering `max_capacity`).

**Request:**

```json
{
  "request_id": "req-005",
  "payload": {
    "updates": [
      { "category": "milk", "subtype": "whole_fat_milk", "field": "max_capacity", "value": 20000 },
      { "category": "cups", "subtype": "cup_H7", "field": "warning_threshold", "value": 60 }
    ]
  }
}
```

**Notes:**
- Allowed `field` values: `max_capacity`, `warning_threshold`, `critical_threshold`, `low_threshold`
- This is an **admin-style operation** and should only be invoked from trusted components (e.g. API Bridge with proper access controls).

### Analytics

#### Action: `category_summary`
Get lowest stock level per category.

**Response:**
```json
{
  "passed": true,
  "details": {
    "coffee_beans": "high",
    "cups": "medium",
    "milk": "low",
    "syrups": "high"
  }
}
```

#### Action: `stock_level`
Get count of items per stock level.

**Response:**
```json
{
  "passed": true,
  "details": {
    "high": 5,
    "medium": 3,
    "low": 2,
    "empty": 0
  }
}
```

#### Action: `inventory_by_stock_level`
Get ingredients filtered by level.

**Request:**
```json
{
  "payload": {
    "stock_level": "low"
  }
}
```

**Response:**
```json
{
  "passed": true,
  "details": {
    "milk": {
      "whole": {"current_amount": 150, "status": "low"}
    }
  }
}
```

#### Action: `category_info`
Get inventory category metadata (categories, subtypes, capacities/thresholds).

#### Action: `category_count`
Get count of items per category (for dashboard summary cards).

### Computer Vision

#### Action: `cup_detection`
Detect cup presence at positions.

**Response:**
```json
{
  "passed": true,
  "details": {
    "cups_detected": {
      "0": true,
      "1": false,
      "2": true,
      "3": false
    },
    "detected_count": 2,
    "total_positions": 4
  }
}
```

#### Action: `milk_cup_detection_present`
Validate the milk station: **passes only if** a cup is detected (expected present).

#### Action: `milk_cup_detection_absent`
Validate the milk station: **passes only if** a cup is not detected (expected absent).

#### Action: `sauce_cup_detection_present`
Validate the sauce station: **passes only if** a cup is detected (expected present).

#### Action: `sauce_cup_detection_absent`
Validate the sauce station: **passes only if** a cup is not detected (expected absent).

### Routine Service Integration

These actions accept the routine-style payload where ingredient groups are sent by key, for example:

```json
{
  "request_id": "routine-cup123-...",
  "client_type": "routine",
  "cup_id": "cup123",
  "milk": { "20": 110.0 },
  "syrups": { "14": 5.0, "7": 8.0 },
  "sauce": { "12": 6.0 },
  "cups": { "cup_H9": 1.0 },
  "espresso": { "espresso_shot_single": 1.0 }
}
```

Supported routine actions:
- `validate_ingredients`: validate all provided ingredients (no inventory mutation)
- `update_ingredients`: deduct all provided ingredients
- `update_milk`: deduct milk only
- `update_water`: deduct water only (if tracked)
- `update_syrup`: deduct syrups only
- `update_sauce`: deduct sauce only
- `check_coffee_beans`: coffee-beans focused validation using the same routine format

### System

#### Action: `health`
Health check.

**Response:**
```json
{
  "status": "healthy",
  "service": "validation",
  "capabilities": ["pre_check", "update_inventory", "cup_detection", ...]
}
```

#### Action: `validation_test` / `validation_test2`
Simple diagnostic actions used to validate RPC connectivity and latency in non-production flows.

## Usage Examples

### Python Client

```python
from shared.rabbitmq_client import RabbitMQClient
import asyncio

async def check_inventory():
    client = RabbitMQClient("test_client")
    await client.connect()
    
    # Get coffee beans status
    response = await client.send_request(
        target_service="validation",
        action="inventory_status",
        data={
            "ingredient_type": "coffee_beans",
            "subtype": "regular"
        },
        timeout=10
    )
    
    print(f"Coffee beans: {response['details']}")
    await client.disconnect()

asyncio.run(check_inventory())
```

### Pre-Check Before Order

```python
async def validate_order(order_items):
    client = RabbitMQClient("scheduler")
    await client.connect()
    
    response = await client.send_request(
        target_service="validation",
        action="pre_check",
        data={
            "client_type": "scheduler",
            "payload": {"items": order_items}
        },
        timeout=10
    )
    
    if response["passed"]:
        print("Order can proceed")
    else:
        print("Insufficient ingredients")
    
    await client.disconnect()
```

### Refill with Detection

```python
async def refill_coffee():
    client = RabbitMQClient("api_bridge")
    await client.connect()
    
    # Triggers CV detection
    response = await client.send_request(
        target_service="validation",
        action="inventory_refill",
        data={
            "ingredient_type": "coffee_beans",
            "subtype": "regular"
        },
        timeout=30
    )
    
    if response["passed"]:
        percentage = response["details"]["coffee_beans_percentage"]
        print(f"Refilled to {percentage}%")
    
    await client.disconnect()
```

## Dependencies

### Core Dependencies

- **aio-pika** (9.5.5): Async RabbitMQ client
- **psycopg2-binary** (2.9.10): PostgreSQL driver
- **pydantic** (2.11.4): Data validation
- **FastAPI** (0.115.12): Used for shared schemas/exceptions; this service does not run an HTTP server

### Computer Vision

- **opencv-python-headless** (4.8.1.78): Image processing (no GUI)
- **numpy** (2.2.6): Array operations
- **torch** (2.8.0): Deep learning framework
- **torchvision** (0.23.0): Vision models
- **transformers**: Model/runtime utilities
- **rfdetr** (1.3.0): Cup detection model
- **supervision** (0.26.1): Detection post-processing utilities
- **pillow** (11.3.0): Image IO utilities
- **requests** (2.31.0): HTTP client for camera snapshots

### Utilities

- **psutil** (5.9.0): System monitoring

## Integration Points

### Downstream Services (Calls To)

1. **RabbitMQ Broker**
   - RPC request handling and event publication
   - **Port**: 5672 (typical)

2. **PostgreSQL Database**
   - Inventory CRUD operations
   - Historical data persistence
   - **Port**: 5432

3. **IP Cameras**
   - Coffee beans detection (HTTP snapshot)
   - Cup detection (RTSP stream)
   - **Protocols**: HTTP/RTSP

### Upstream Services (Receives From)

1. **Scheduler Service**
   - `pre_check`: Validate orders before processing
   - `update_inventory`: Deduct ingredients after consumption
   - **Protocol**: RabbitMQ RPC

2. **Routine Service**
   - `validate_ingredients`, `update_ingredients`, `update_milk`, `update_water`, `update_syrup`, `update_sauce`, `check_coffee_beans`
   - **Protocol**: RabbitMQ RPC

3. **API Bridge / Dashboard**
   - `inventory_status`: Get current levels
   - `inventory_refill`: Manual refill operations
   - `category_summary`, `stock_level`, `inventory_by_stock_level`, `category_info`, `category_count`
   - `update_limits`: Update capacities/thresholds
   - **Protocol**: RabbitMQ RPC

4. **Automation / Diagnostics**
   - `validation_test`, `validation_test2`
   - **Protocol**: RabbitMQ RPC

### Event Publications

Broadcasts to `barns_events` exchange:
- `validation.inventory_updated`: Category-specific updates
- `validation.all_inventory_updated`: Full inventory status
- `validation.stock_level_updated`: Stock statistics
- `validation.category_summary_updated`: Category summaries
- `validation.threshold_warning`: Low/empty alerts
- `validation.threshold_resolved`: Stock restored alerts

## Troubleshooting

### Periodic Detection Not Running

**Issue**: Coffee beans detection not executing every 10 minutes

**Solutions:**
1. Check configuration:
   ```bash
   docker-compose logs validation-service | grep "periodic detection"
   ```

2. Verify environment variable:
   ```yaml
   ENABLE_PERIODIC_DETECTION: "true"
   DETECTION_INTERVAL_SECONDS: 600
   ```

3. Check camera connectivity:
   ```bash
   curl "http://qltyss:QSS2030QSS@192.168.200.88/cgi-bin/snapshot.cgi" -o test.jpg
   ```

### Database Connection Failed

**Issue**: Service fails to connect to PostgreSQL

**Solutions:**
1. Verify database is running:
   ```bash
   docker-compose ps postgres
   ```

2. Check database initialization:
   ```bash
   docker-compose logs postgres | grep "database system is ready"
   ```

3. Test connection manually:
   ```bash
   docker exec -it barns-postgres psql -U validation_user -d barns_validation -c "SELECT * FROM inventory_items LIMIT 1;"
   ```

### CV Detection Errors

**Issue**: Coffee or cup detection failing

**Debugging:**
1. Enable debug mode and check saved frames:
   ```bash
   ls -lh services/validation/coffee_detection/debug_frames_coffee/
   ls -lh services/validation/cup_detection/debug_frames_cup/
   ```

2. Test camera access:
   ```python
   import cv2
   cap = cv2.VideoCapture(0)  # or RTSP URL
   ret, frame = cap.read()
   print(f"Camera accessible: {ret}")
   ```

3. Check RF-DETR model files:
   ```bash
   ls -lh services/validation/cup_detection/models/
   ```

### Inventory Out of Sync

**Issue**: Database doesn't match cache

**Solution**: Restart service to reload from database:
```bash
docker-compose restart validation-service
```

### Memory Leak from Debug Frames

**Issue**: Disk space growing from debug images

**Solution**: Debug frame cleanup is now automatic (max 10 frames). Verify:
```bash
# Should show only 10 most recent frame sets
ls -lh services/validation/coffee_detection/debug_frames_coffee/ | wc -l
```

## Performance Considerations

- **Detection Latency**: Coffee detection ~3-5s, Cup detection ~0.5-1s
- **Database Queries**: Cached in memory, ~1ms access time
- **RabbitMQ Throughput**: Handles 100+ req/sec
- **Memory Usage**: ~500MB base + ~100MB during CV operations
- **Thread Pool**: 3 workers for concurrent CV operations
- **PostgreSQL Connections**: Connection pooling recommended

## Security Notes

- Database credentials in environment variables
- Camera URLs contain credentials (consider IP whitelisting)
- No API authentication (internal network only)
- RabbitMQ credentials in connection string
- Debug frames may contain operational data

## Future Enhancements

- Multi-camera support for redundancy
- ML-based consumption prediction
- Automated reordering system
- Mobile app for inventory management
- Advanced analytics and reporting
- Integration with supplier APIs
- QR code-based manual count verification
