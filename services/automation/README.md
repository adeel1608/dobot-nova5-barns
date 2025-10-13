# Automation Service

## Brief Overview

The Automation Service controls physical brewing equipment including grinders, espresso machines, milk frothers, syrup dispensers, and other hardware components via MQTT and direct interfaces, executing automated workflows for drink preparation.

## Key Features

- **Hardware Control**: Interface with grinders, espresso machines, dispensers
- **MQTT Integration**: RabbitMQ MQTT plugin for device communication
- **Function Library**: 20+ automation functions (grind, brew, dispense, steam)
- **Async Execution**: Non-blocking hardware operations
- **Event Broadcasting**: Real-time status updates
- **Error Handling**: Hardware failure detection and reporting
- **Emergency Stop**: Immediate halt capability

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│              Automation Service                          │
│                                                          │
│  ┌──────────────────────────────────────────────┐      │
│  │  AutomationService (app.py)                  │      │
│  │  - RabbitMQ Handler                          │      │
│  └───────────┬──────────────────────────────────┘      │
│              │                                          │
│              ↓                                          │
│  ┌──────────────────────────────────────────────┐      │
│  │  AUTOMATION_FUNCTIONS                        │      │
│  │  (automation_functions.py)                   │      │
│  │  - heat_water()                              │      │
│  │  - dispense_syrup()                          │      │
│  │  - activate_grinder()                        │      │
│  │  - pull_espresso_shot()                      │      │
│  │  - steam_milk()                              │      │
│  │  - dispense_milk()                           │      │
│  │  + 15 more...                                │      │
│  └───────────┬──────────────────────────────────┘      │
│              │                                          │
└──────────────┼──────────────────────────────────────────┘
               │
               ↓
    ┌──────────────────────────┐
    │   MQTT Broker (RabbitMQ) │
    │   Port 1883              │
    └──────────┬───────────────┘
               │
      ┌────────┼────────┐
      ↓        ↓        ↓
┌──────────┐ ┌───────┐ ┌──────────┐
│ Grinder  │ │ESP Machine│ │Dispensers│
│ (Arduino)│ │ (Arduino) │ │ (Arduino)│
└──────────┘ └───────┘ └──────────┘
```

## Setup & Installation

### Local Development

```bash
cd services/automation
pip install -r requirements.txt

export RABBITMQ_URL="amqp://admin:admin123@localhost:5672/"
export MQTT_HOST="localhost"
export PYTHONPATH="/path/to/barns"

python app.py
```

### Docker Deployment

```bash
docker-compose up -d rabbitmq automation-service
docker-compose logs -f automation-service
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ AMQP connection |
| `MQTT_HOST` | `rabbitmq` | MQTT broker host (uses RabbitMQ MQTT plugin) |
| `PYTHONPATH` | `/app` | Python module path |

### MQTT Topics

- **Request**: `automation/request` - Send automation commands
- **Response**: `automation/response` - Receive device responses

## API/Endpoints

### Action: `automate`
Execute an automation function.

**Request:**
```json
{
  "function": "activate_grinder",
  "params": {
    "duration_sec": 15,
    "grind_size": "fine"
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Ground coffee for 15 seconds at fine setting",
  "details": {
    "duration": 15,
    "grind_size": "fine",
    "amount_g": 18
  }
}
```

### Action: `list_functions`
Get available automation functions.

**Response:**
```json
{
  "success": true,
  "functions": [
    "heat_water",
    "dispense_syrup",
    "activate_grinder",
    "pull_espresso_shot",
    "steam_milk",
    "dispense_milk",
    "...20 more"
  ]
}
```

### Action: `stop_automation`
Emergency stop all operations.

### Action: `health`
Health check.

## Available Functions

### Coffee Preparation
- `activate_grinder(duration_sec, grind_size)`: Grind coffee beans
- `pull_espresso_shot(shots, temperature)`: Extract espresso
- `tamp_coffee(pressure)`: Tamp ground coffee

### Milk Operations
- `steam_milk(temperature, texture)`: Steam milk to temp
- `dispense_milk(pump_number, amount)`: Dispense milk type
- `froth_milk(duration, intensity)`: Create microfoam

### Dispensing
- `dispense_syrup(pump_number, amount)`: Dispense syrups
- `dispense_water(amount_ml, temperature)`: Hot water
- `dispense_ice(amount)`: Ice dispenser

### Utilities
- `heat_water(target_temp, volume_ml)`: Heat water
- `clean_group_head()`: Cleaning cycle
- `purge_steam_wand()`: Steam wand flush

## Usage Examples

### From Routine Service

```python
response = await rabbitmq_client.send_request(
    target_service="automation",
    action="automate",
    data={
        "function": "activate_grinder",
        "params": {"duration_sec": 15}
    },
    timeout=80
)
```

### MQTT Direct Control

```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client()
client.username_pw_set("admin", "admin123")
client.connect("rabbitmq", 1883)

payload = json.dumps({
    "pump_number": 9,
    "amount": 20
})

client.publish("automation/request", payload)
```

## Dependencies

- **paho-mqtt** (1.6.1): MQTT client
- **aio-pika** (9.3.1): RabbitMQ async client
- **asyncio-mqtt** (0.13.0): Async MQTT support

## Integration Points

### Upstream Services
- **Routine Service**: Task execution requests

### Downstream Services
- **Arduino/ESP32 Devices**: Via MQTT
- **Direct Hardware**: Via serial/GPIO (if configured)

### Event Publications
- `automation.started`: Function execution began
- `automation.completed`: Function completed successfully
- `automation.failed`: Function execution failed

## Troubleshooting

### MQTT Connection Failed
```bash
# Check RabbitMQ MQTT plugin
docker exec -it barns-rabbitmq rabbitmq-plugins list | grep mqtt

# Enable if needed
docker exec -it barns-rabbitmq rabbitmq-plugins enable rabbitmq_mqtt
```

### Hardware Not Responding
1. Check device power and connectivity
2. Verify MQTT topics: `docker-compose logs | grep "mqtt"`
3. Test MQTT manually:
   ```bash
   mosquitto_sub -h localhost -p 1883 -t "automation/#" -u admin -P admin123
   ```

## Security Notes

- MQTT credentials required for device access
- No encryption on MQTT (local network only)
- Hardware commands not authenticated beyond MQTT

## Future Enhancements

- CAN bus integration for industrial devices
- Predictive maintenance alerts
- Hardware telemetry and monitoring
- Recipe optimization based on equipment state
