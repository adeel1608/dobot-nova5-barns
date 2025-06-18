# Robot Arm Service

Physical robotic arm control service that provides hardware interfaces, motion planning, and safety protocols for automated operations within the BARNS system.

## Overview

The Robot Arm Service manages physical robotic hardware, executing precise movements and operations as coordinated by the task scheduling system. It includes safety monitoring, calibration routines, and real-time position feedback.

## Features

- **Hardware Control**: Direct interface with robotic arm controllers
- **Motion Planning**: Path planning and trajectory optimization
- **Safety Monitoring**: Emergency stops and collision detection
- **Calibration System**: Automated calibration and homing routines
- **Position Feedback**: Real-time joint and end-effector positioning
- **Tool Management**: Support for various end-effector tools
- **Error Recovery**: Automatic recovery from minor faults

## Supported Operations

### Basic Movements

```python
# Joint control
move_to_joint_position([90, 45, 30, 0, 90, 0])  # degrees

# Cartesian control
move_to_position(x=300, y=200, z=150)  # mm

# Linear movements
linear_move(start_pos, end_pos, speed=50)  # mm/s

# Circular movements
circular_move(center, start_angle, end_angle, radius)
```

### Tool Operations

```python
# Gripper control
open_gripper()
close_gripper()
set_gripper_force(50)  # percentage

# Specialized tools
activate_suction_cup()
dispense_liquid(volume_ml=50)
pick_and_place(pick_pos, place_pos)
```

### Safety Functions

```python
# Emergency controls
emergency_stop()
soft_stop()
resume_operation()

# Safety monitoring
check_workspace_collision()
verify_tool_attachment()
monitor_joint_limits()
```

## Configuration

### Hardware Settings

```yaml
# robot_config.yaml
robot:
  model: "UR5e"
  ip_address: "192.168.1.100"
  port: 30001
  
joints:
  count: 6
  limits:
    - [-360, 360]  # Joint 1 (degrees)
    - [-360, 360]  # Joint 2
    - [-360, 360]  # Joint 3
    - [-360, 360]  # Joint 4
    - [-360, 360]  # Joint 5
    - [-360, 360]  # Joint 6

workspace:
  x_limits: [-500, 500]  # mm
  y_limits: [-500, 500]  # mm
  z_limits: [0, 600]     # mm

safety:
  max_speed: 100         # mm/s
  max_acceleration: 500  # mm/s²
  emergency_stop_time: 0.1  # seconds
```

### Environment Variables

```env
# Robot connection
ROBOT_IP=192.168.1.100
ROBOT_PORT=30001
ROBOT_MODEL=UR5e

# Safety settings
ENABLE_COLLISION_DETECTION=true
EMERGENCY_STOP_PIN=18
SAFETY_ZONE_ENABLED=true

# RabbitMQ connection
RABBITMQ_URL=amqp://admin:admin123@rabbitmq:5672/
ROBOT_QUEUE=robot_commands
```

## Message Interface

### Command Messages

```json
{
  "command_id": "cmd_12345",
  "robot_id": "arm_1", 
  "operation": "move_to_position",
  "parameters": {
    "x": 300,
    "y": 200,
    "z": 150,
    "speed": 50,
    "wait_for_completion": true
  },
  "priority": "normal",
  "timeout": 30
}
```

### Response Messages

```json
{
  "command_id": "cmd_12345",
  "robot_id": "arm_1",
  "status": "completed",
  "result": {
    "final_position": {"x": 300, "y": 200, "z": 150},
    "execution_time": 2.5,
    "path_deviation": 0.1
  },
  "timestamp": "2025-12-12T10:30:00Z"
}
```

### Status Updates

```json
{
  "robot_id": "arm_1",
  "type": "status_update",
  "data": {
    "joint_positions": [90, 45, 30, 0, 90, 0],
    "end_effector_position": {"x": 300, "y": 200, "z": 150},
    "tool_status": "gripper_closed",
    "is_moving": false,
    "safety_status": "ok"
  }
}
```

## Architecture

### Service Components

```
Robot Arm Service
├── Hardware Interface
│   ├── Robot Controller Driver
│   ├── Sensor Interface
│   └── Emergency Stop Monitor
├── Motion Planning
│   ├── Path Planner
│   ├── Collision Detector
│   └── Trajectory Optimizer
├── Safety System
│   ├── Workspace Monitor
│   ├── Joint Limit Checker
│   └── Emergency Handler
└── Message Handler
    ├── Command Processor
    ├── Status Publisher
    └── Error Reporter
```

### Control Flow

```
RabbitMQ Command
    ↓
Command Validation
    ↓
Safety Check
    ↓
Motion Planning
    ↓
Hardware Execution
    ↓
Position Feedback
    ↓
Status Update (RabbitMQ)
```

## Development

### Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Install robot driver (example for Universal Robots)
pip install urx

# Start service with simulation mode
python app.py --simulation

# Test with mock hardware
python -m robot_arm.simulation
```

### Hardware Testing

```bash
# Connect to real robot
python app.py --robot-ip 192.168.1.100

# Run calibration
python -m robot_arm.calibrate

# Test basic movements
python -m robot_arm.test_movements
```

### Simulation Mode

```python
# Enable simulation for development
SIMULATION_MODE = True

class SimulatedRobot:
    def __init__(self):
        self.position = [0, 0, 0, 0, 0, 0]
        self.is_connected = True
    
    def move_to_joint_position(self, positions):
        # Simulate movement with delay
        time.sleep(2)
        self.position = positions
        return True
```

## Safety Protocols

### Emergency Procedures

```python
# Emergency stop activation
def emergency_stop():
    # Hardware emergency stop
    robot.emergency_stop()
    
    # Stop all queued commands
    command_queue.clear()
    
    # Notify all services
    publish_emergency_alert("ROBOT_EMERGENCY_STOP")
    
    # Log incident
    log_safety_incident("emergency_stop", get_robot_status())
```

### Collision Detection

```python
# Real-time collision monitoring
def monitor_collisions():
    while robot.is_active():
        current_pos = robot.get_position()
        
        if check_workspace_collision(current_pos):
            trigger_safety_stop("collision_detected")
            
        if check_joint_limits(robot.get_joint_positions()):
            trigger_safety_stop("joint_limit_exceeded")
            
        time.sleep(0.01)  # 100Hz monitoring
```

### Safety Zones

```python
# Define safety zones
SAFETY_ZONES = {
    "human_workspace": {
        "type": "exclusion",
        "bounds": {"x": [-200, 200], "y": [300, 500], "z": [0, 400]}
    },
    "equipment_area": {
        "type": "speed_limit",
        "bounds": {"x": [400, 600], "y": [-100, 100], "z": [0, 200]},
        "max_speed": 25
    }
}
```

## Calibration

### Auto-Calibration Routine

```python
def perform_calibration():
    # Home all joints
    robot.move_to_home_position()
    
    # Touch reference points
    reference_points = load_reference_points()
    measured_points = []
    
    for point in reference_points:
        robot.move_to_position(point)
        actual_pos = robot.get_precise_position()
        measured_points.append(actual_pos)
    
    # Calculate calibration matrix
    calibration_matrix = calculate_calibration(
        reference_points, measured_points
    )
    
    # Save calibration
    save_calibration_data(calibration_matrix)
    
    return calibration_matrix
```

### Manual Calibration

```bash
# Interactive calibration mode
python -m robot_arm.calibrate --interactive

# Verify calibration accuracy
python -m robot_arm.verify_calibration

# Reset to factory calibration
python -m robot_arm.reset_calibration
```

## Monitoring

### Performance Metrics

```python
# Real-time metrics
METRICS = {
    "position_accuracy": 0.1,      # mm
    "repeat_accuracy": 0.05,       # mm
    "max_speed_achieved": 95,      # mm/s
    "cycle_time": 2.3,             # seconds
    "uptime": 99.8,                # percentage
    "error_rate": 0.02             # percentage
}
```

### Health Monitoring

```python
def check_robot_health():
    health_status = {
        "joint_temperatures": robot.get_joint_temperatures(),
        "motor_currents": robot.get_motor_currents(),
        "position_errors": robot.get_position_errors(),
        "communication_latency": measure_communication_delay(),
        "last_maintenance": get_last_maintenance_date()
    }
    
    # Check for issues
    issues = []
    if max(health_status["joint_temperatures"]) > 70:
        issues.append("high_temperature")
    
    if health_status["communication_latency"] > 10:
        issues.append("communication_delay")
    
    return {"status": "healthy" if not issues else "warning", "issues": issues}
```

## Troubleshooting

### Common Issues

#### Robot Not Responding
```bash
# Check network connection
ping 192.168.1.100

# Verify robot controller status
curl http://192.168.1.100/status

# Check service logs
docker logs barns-robot-arm -f
```

#### Position Accuracy Issues
```bash
# Run calibration check
python -m robot_arm.verify_calibration

# Check for mechanical wear
python -m robot_arm.diagnostic_test

# Recalibrate if needed
python -m robot_arm.calibrate
```

#### Safety System Triggered
```bash
# Check safety logs
grep "safety" /var/log/robot_arm.log

# Reset safety system
python -m robot_arm.reset_safety

# Verify workspace clear
python -m robot_arm.check_workspace
```

### Diagnostic Tools

```bash
# Complete system diagnostic
python -m robot_arm.diagnostic --full

# Test specific joints
python -m robot_arm.test_joint --joint 1

# Communication test
python -m robot_arm.comm_test --duration 60
```

## Integration

### Task Execution Integration

```python
# Receive task from routine service
@app.message_handler('task.execute.robot')
async def handle_robot_task(message):
    task_data = json.loads(message.body)
    
    try:
        # Execute robot operation
        result = await execute_robot_operation(task_data)
        
        # Send completion message
        await publish_task_result(task_data['task_id'], result)
        
    except RobotError as e:
        await publish_task_error(task_data['task_id'], str(e))
```

### Real-time Position Streaming

```python
# Stream position updates
async def stream_position_updates():
    while robot.is_connected():
        position = robot.get_current_position()
        
        await publish_status_update({
            'robot_id': ROBOT_ID,
            'position': position,
            'timestamp': time.time()
        })
        
        await asyncio.sleep(0.1)  # 10Hz updates
```

## Maintenance

### Scheduled Maintenance

```python
# Maintenance checklist
MAINTENANCE_TASKS = [
    "lubricate_joints",
    "check_belt_tension", 
    "calibrate_sensors",
    "verify_safety_systems",
    "update_firmware",
    "backup_configuration"
]

# Maintenance scheduler
def schedule_maintenance():
    next_maintenance = calculate_next_maintenance()
    
    if next_maintenance <= datetime.now():
        trigger_maintenance_mode()
        execute_maintenance_routine()
```

### Performance Optimization

```python
# Motion optimization
def optimize_path(waypoints):
    # Minimize jerk and acceleration
    optimized_path = trajectory_optimizer.optimize(
        waypoints=waypoints,
        max_speed=robot_config.max_speed,
        max_acceleration=robot_config.max_acceleration
    )
    
    return optimized_path
```

## Dependencies

### Hardware Dependencies
- **Robot Controller**: Universal Robots, ABB, KUKA, etc.
- **Safety System**: Emergency stop buttons, light curtains
- **Sensors**: Position encoders, force sensors
- **Network**: Ethernet connection to robot controller

### Software Libraries
- **urx**: Universal Robots communication library
- **robotics-toolbox**: Motion planning and kinematics
- **numpy**: Numerical computations
- **scipy**: Scientific computing for optimization

---

**Hardware**: 6-DOF Industrial Robot Arm  
**Technology**: Python + RobbitMQ + Robot Controllers  
**Safety**: ISO 10218 compliant  
**Precision**: ±0.1mm repeatability 