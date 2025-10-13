# Robot Arm Service

## Brief Overview

The Robot Arm Service provides high-level control interface for robotic arm operations including pick-and-place, movement coordination, gripper control, and integration with ROS/MoveIt for motion planning in both simulation and hardware modes.

## Key Features

- **Simulation Mode**: Test without physical hardware
- **Hardware Mode**: Control actual robotic arms
- **Action Library**: pick_cup, move_to_position, place_object, open/close_gripper
- **ROS Integration**: Communicates with robot_container (ROS/MoveIt)
- **Dual Arm Support**: Independent control of Arm 1 and Arm 2
- **Emergency Stop**: Immediate halt capability
- **Status Monitoring**: Real-time position and state tracking

## Architecture

```
┌──────────────────────────────────────────────────┐
│          Robot Arm Service                       │
│                                                  │
│  ┌────────────────────────────────────┐         │
│  │  RobotArmService (app.py)          │         │
│  │  - Message Handlers                │         │
│  └───────────┬────────────────────────┘         │
│              │                                   │
│              ↓                                   │
│  ┌────────────────────────────────────┐         │
│  │  ROBOT_ACTIONS                     │         │
│  │  (robot_actions.py)                │         │
│  │  - pick_cup()                      │         │
│  │  - move_to_position()              │         │
│  │  - place_object()                  │         │
│  │  - open_gripper()                  │         │
│  │  - close_gripper()                 │         │
│  └───────────┬────────────────────────┘         │
│              │                                   │
└──────────────┼───────────────────────────────────┘
               │
               ↓
    ┌──────────────────────┐
    │  Robot Container     │
    │  (ROS/MoveIt)        │
    │  - Motion Planning   │
    │  - Inverse Kinematics│
    │  - Collision Avoid   │
    └──────────────────────┘
               │
               ↓
    ┌──────────────────────┐
    │  Physical/Simulated  │
    │  Robotic Arms        │
    │  - Arm 1             │
    │  - Arm 2             │
    └──────────────────────┘
```

## Setup & Installation

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RABBITMQ_URL` | `amqp://admin:admin123@rabbitmq:5672/` | RabbitMQ connection |
| `ROBOT_SIMULATION` | `true` | `true` for simulation, `false` for hardware |
| `PYTHONPATH` | `/app` | Python module path |

### Docker Deployment

```bash
# Simulation mode (default)
docker-compose up -d robot-arm-service

# Hardware mode
ROBOT_SIMULATION=false docker-compose up -d robot-arm-service
```

## API/Endpoints

### Action: `robot_action`
Execute a robot action.

**Request:**
```json
{
  "function": "pick_cup",
  "params": {
    "arm_id": 1,
    "position": "medium_cup_stack",
    "grip_force": 30
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Cup picked successfully",
  "details": {
    "arm_id": 1,
    "position": [0.3, 0.2, 0.15],
    "duration_sec": 3.2
  }
}
```

### Action: `list_actions`
Get available robot actions.

**Response:**
```json
{
  "success": true,
  "actions": [
    "pick_cup",
    "place_cup",
    "move_to_position",
    "open_gripper",
    "close_gripper",
    "home_position",
    "calibrate"
  ]
}
```

### Action: `emergency_stop`
Immediately halt all robot movement.

### Action: `calibrate`
Calibrate robot home position.

### Action: `get_status`
Get current robot status.

**Response:**
```json
{
  "success": true,
  "status": {
    "arm_1": {"position": [0,0,0], "state": "idle"},
    "arm_2": {"position": [0,0,0], "state": "idle"}
  },
  "simulation_mode": true
}
```

## Available Actions

### Basic Movements
- `move_to_position(x, y, z, arm_id)`: Move to coordinates
- `home_position(arm_id)`: Return to home
- `calibrate(arm_id)`: Calibrate arm

### Cup Operations
- `pick_cup(position, arm_id, grip_force)`: Pick cup from stack
- `place_cup(position, arm_id)`: Place cup at location
- `hold_cup(arm_id)`: Maintain grip on cup

### Gripper Control
- `open_gripper(arm_id, width)`: Open gripper
- `close_gripper(arm_id, force)`: Close gripper

## Usage Examples

### From Routine Service

```python
response = await rabbitmq_client.send_request(
    target_service="robot_arm",
    action="robot_action",
    data={
        "function": "pick_cup",
        "params": {
            "arm_id": 1,
            "position": "medium_cup_stack"
        }
    },
    timeout=300
)
```

## Dependencies

- **grpcio** (1.60.0): gRPC for ROS communication
- **protobuf** (6.32.1): Protocol buffers
- **aio-pika** (9.3.1): RabbitMQ async client

## Integration Points

### Upstream Services
- **Routine Service**: Task execution requests

### Downstream Services
- **robot_container**: ROS/MoveIt motion planning

### Event Publications
- `robot.action_started`: Action began
- `robot.action_completed`: Action completed
- `robot.action_failed`: Action failed

## Troubleshooting

### Simulation vs Hardware Mode
Check current mode:
```bash
docker-compose logs robot-arm-service | grep "mode"
```

### Robot Not Responding
1. Check robot_container:
   ```bash
   docker-compose ps robot_container_1 robot_container_2
   ```

2. Verify ROS nodes:
   ```bash
   docker exec -it robot_container_1 ros2 node list
   ```

### Motion Planning Failed
- Increase timeout (motion planning can take 30s+)
- Check for collisions in workspace
- Verify target position is reachable

## Security Notes

- No authentication (internal network only)
- Emergency stop accessible without credentials
- Simulation mode recommended for testing

## Future Enhancements

- Computer vision integration for object detection
- Force feedback for precise control
- Multi-arm coordinated actions
- Obstacle avoidance improvements
