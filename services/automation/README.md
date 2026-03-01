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

### MQTT Device Topics (via RabbitMQ MQTT plugin)

The service publishes hardware commands to these topics and waits for responses on the paired `/response` topic:

- `automation_coffee_machine_hot_water` / `automation_coffee_machine_hot_water/response`
- `automation_syrup` / `automation_syrup/response`
- `automation_milk` / `automation_milk/response`
- `automation_trigger` / `automation_trigger/response`
- `automation_slush` / `automation_slush/response`
- `automation_coffee_machine` / `automation_coffee_machine/response`
- `automation_grinding` / `automation_grinding/response`
- `automation_tampering` / `automation_tampering/response`
- `automation_ice` / `automation_ice/response`
- `automation_frother` / `automation_frother/response`
- `automation_frother_init` / `automation_frother_init/response`
- `automation_clean_frother` / `automation_clean_frother/response`
- `automation_rinser` / `automation_rinser/response`

## APIs In (Consumed by Automation Service)

This service does not expose HTTP endpoints. It consumes RabbitMQ actions/events.

### RabbitMQ Actions

| Action | Purpose | Request Shape |
|--------|---------|---------------|
| `automate` | Execute one automation function | `{ "function": "<name>", "params": { ... } }` |
| `health` | Service health check | `{}` |
| `list_functions` | List supported function names | `{}` |
| `stop_automation` | Stop active automation flow | `{}` |

### Event Subscriptions

- `system.shutdown`
- `automation.emergency_stop`
- Subscription patterns: `system.*`, `automation.*`

### `automate` Request Example

```json
{
  "function": "grinding_machine",
  "params": {
    "grind_size": "fine",
    "timeout": 75
  }
}
```

## APIs Out (Produced by Automation Service)

### RabbitMQ Events Published

- `automation.started`
- `automation.completed`
- `automation.error`
- `automation.stopped`
- `automation.emergency_stopped`

### Outbound Device Command APIs (MQTT)

| Function (via `automate`) | Command Topic | Response Topic |
|---------------------------|---------------|----------------|
| `dispense_hot_water` | `automation_coffee_machine_hot_water` | `automation_coffee_machine_hot_water/response` |
| `dispense_syrup` | `automation_syrup` | `automation_syrup/response` |
| `dispense_sauce` | `automation_milk` | `automation_milk/response` |
| `dispense_milk` | `automation_milk` | `automation_milk/response` |
| `purge_milks_syrups` | `automation_trigger` | `automation_trigger/response` |
| `slush_machine` | `automation_slush` | `automation_slush/response` |
| `coffee_machine` | `automation_coffee_machine` | `automation_coffee_machine/response` |
| `coffee_machine_wait` | `automation_coffee_machine` | `automation_coffee_machine/response` |
| `coffee_machine_purge` | `automation_coffee_machine` | `automation_coffee_machine/response` |
| `grinding_machine` | `automation_grinding` | `automation_grinding/response` |
| `tampering_machine` | `automation_tampering` | `automation_tampering/response` |
| `dispense_ice` | `automation_ice` | `automation_ice/response` |
| `froth_milk` | `automation_frother` | `automation_frother/response` |
| `initialize_frother` | `automation_frother_init` | `automation_frother_init/response` |
| `clean_frother` | `automation_clean_frother` | `automation_clean_frother/response` |
| `rinser_machine` | `automation_rinser` | `automation_rinser/response` |
| `automation_test` | No external call (returns local success payload) | N/A |

## Available Functions

- `dispense_syrup`
- `dispense_sauce`
- `dispense_milk`
- `purge_milks_syrups`
- `slush_machine`
- `coffee_machine`
- `coffee_machine_purge`
- `coffee_machine_wait`
- `grinding_machine`
- `tampering_machine`
- `dispense_ice`
- `froth_milk`
- `initialize_frother`
- `clean_frother`
- `rinser_machine`
- `automation_test`
- `dispense_hot_water`

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
- **Routine Service**: Sends `automate` requests and queries (`health`, `list_functions`)

### Downstream Services
- **Arduino/ESP32 Devices**: Controlled via MQTT topics listed above

### Event Publications
- `automation.started`: Function execution began
- `automation.completed`: Function execution completed
- `automation.error`: Function execution failed
- `automation.stopped`: Stop request handled
- `automation.emergency_stopped`: Emergency stop processed

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
