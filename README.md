# Dobot Nova 5 - BARNS Coffee Automation System

> **Advanced robotic coffee preparation system with integrated computer vision, motion planning, and multi-station orchestration**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 🌟 Overview

This workspace contains the complete software stack for the BARNS (Barista Autonomous Robotic Navigation System) coffee automation platform, featuring the Dobot Nova 5 robotic arm. The system autonomously handles the entire coffee preparation workflow including espresso extraction, milk frothing, cup dispensing, and beverage assembly across multiple stations.

## 📁 Repository Structure

```
barns_ws/
├── src/
│   ├── pickn_place/          # 🎯 MAIN PACKAGE - Core automation logic
│   ├── dobot_bringup_v3/     # Robot hardware interface
│   ├── dobot_msgs_v3/        # Custom message definitions
│   ├── dobot_moveit/         # Motion planning configuration
│   ├── dobot_rviz/           # Visualization and URDF models
│   ├── nova5_moveit/         # MoveIt2 configuration
│   ├── pymoveit2/            # Python MoveIt2 interface
│   ├── servo_action/         # Real-time servo control
│   ├── oms_v1/               # Order management system
│   ├── moveit2/              # MoveIt2 framework
│   ├── moveit_msgs/          # MoveIt2 messages
│   ├── moveit_resources/     # Robot models and configs
│   └── ros2_control/         # Hardware control interface
├── build/                    # Build artifacts (gitignored)
├── install/                  # Installation files (gitignored)
└── log/                      # Logs (gitignored)
```

## 🎯 pickn_place Package - The Heart of the System

The `pickn_place` package is the **core automation brain** containing all high-level behaviors, computer vision, and motion primitives.

### Key Features

#### 🤖 Motion Control
- **4 Motion Node Versions** (`manipulate_node_v1.py` through `v4.py`)
  - v1: Original DirectTfMotionNode implementation
  - v2: Enhanced robot_motion class with fixes
  - v3: Hybrid - best methods from v1 + v2
  - v4: **Complete** - All functions including new experimental features

#### 🎬 Sequences & Skills (`testing_v1.py`)
Complete beverage preparation workflows:
- **Espresso Operations**: Unmount → Grind → Tamp → Mount → Pour
- **Milk Frothing**: Pick frother → Mount → Froth → Swirl → Pour
- **Cup Management**: Paper cups (7oz, 9oz, 12oz) and Plastic cups (7oz-16oz)
- **Ice & Slush**: Cold beverage preparation with ice dispensing
- **Multi-Station Orchestration**: 4 serving stations with precise placement

#### 👁️ Computer Vision (`computer_vision.py`)
- ArUco marker detection for machine localization
- Cup detection in gripper using depth segmentation
- Real-time pose estimation and calibration

#### 📊 Configuration (`params.py`)
Centralized parameter management:
- Machine positions and offsets
- Tool calibration data
- Speed profiles and timing constants
- Gripper positions for different cup sizes

### Quick Start - Testing the System

```bash
# Source the workspace
cd /home/adeel/barns_ws
source install/setup.bash

# Run the interactive testing interface
ros2 run pickn_place testing_v1

# Or run a specific sequence
python3 -c "from pickn_place.testing_v1 import espresso; espresso()"
```

### Available Sequences

#### Complete Drinks
```python
espresso()         # Full espresso workflow
americano()        # Espresso + hot water
milk()            # Milk frothing and pouring
slushie()         # Frozen beverage preparation
multi_espresso()  # Batch espresso (4 cups)
```

#### Station Operations
```python
# Cup dispensing
grab_paper_cup(size="7oz")
place_paper_cup(position={'cup_position': 1})
dispense_plastic_cup(cups={'cup_C16': 1.0})

# Ice operations
go_to_ice(cups={'cup_C16': 1.0})
place_plastic_cup_station(position={'cup_position': 1})

# Milk frothing
pick_frother()
mount_frother(milk={'milk_whole': 200.0})
unmount_and_swirl_milk()
pour_milk_cup_station(position={'cup_position': 1})
```

## 🛠️ Installation

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- MoveIt2
- Intel RealSense SDK (for depth camera)

### Build Instructions

```bash
# Clone the repository
cd ~
git clone https://github.com/adeel1608/dobot-nova5-barns.git barns_ws
cd barns_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 🚀 Running the System

### 1. Launch Robot Hardware
```bash
ros2 launch dobot_bringup_v3 bringup.launch.py
```

### 2. Launch MoveIt2 (optional, for planning)
```bash
ros2 launch nova5_moveit demo.launch.py
```

### 3. Run Automation Sequences
```bash
# Interactive menu
ros2 run pickn_place testing_v1

# Or direct sequence execution
python3 src/pickn_place/pickn_place/testing_v1.py
```

## 📋 System Capabilities

### Beverage Types
- ☕ **Espresso** (Single/Double shot)
- ☕ **Americano** (Espresso + Hot water)
- 🥛 **Latte/Cappuccino** (Espresso + Frothed milk)
- 🧊 **Iced Drinks** (Cold brew with ice)
- 🍧 **Slushies** (Frozen beverages)

### Hardware Integration
- **Dobot Nova 5**: 6-DOF collaborative arm
- **Intel RealSense**: D435/D405 depth cameras
- **Gripper**: Adaptive parallel jaw with force control
- **Espresso Machine**: 3-group commercial machine with pitcher system
- **Grinder/Tamper**: Automated grinding and tamping station
- **Steam Wand**: Automated milk frothing with temperature control
- **Cup Dispensers**: Paper (7oz-12oz) and Plastic (7oz-16oz)
- **Ice Dispenser**: Automated ice portioning
- **Slush Machines**: Dual-dispenser frozen beverage system

### Station Layout
```
        [Espresso Machine]
              [Grinder]
        [Portafilter Cleaner]
              
[Paper Cups]    [Plastic Cups]
   [Ice]           [Slush 1/2]
   
[Serving Stations 1-4]
   [Milk Frother Area]
```

## 🧪 Calibration

### Machine Position Calibration
```bash
# Run calibration sequence
python3 -c "from pickn_place.testing_v1 import get_machine_position; get_machine_position()"
```

This calibrates:
- ✅ Three-group espresso machine (markers 41-43)
- ✅ Espresso grinder (marker 31)
- ✅ Portafilter cleaner (marker 23)
- ✅ Steam wand (marker 51)

### Tool Calibration
Use the teaching interfaces:
```bash
# Machine mounting points
python3 src/pickn_place/pickn_place/machine_mount_teach.py

# Tool mounting points
python3 src/pickn_place/pickn_place/tool_mount_teach.py
```

## 📊 Configuration Files

Located in `src/pickn_place/share/`:

- `arucoID_name_config.yaml`: ArUco marker ID to machine name mapping
- `aruco_size_config.yaml`: Physical marker sizes for pose estimation
- `machine_pose_data_memory.yaml`: Calibrated machine positions
- `tool_offset_points.yaml`: Tool mounting offsets
- `machine_offset_points.yaml`: Machine mounting offsets

## 🔧 Development

### Version Control
The system uses a versioned approach for motion control:
```python
# In testing_v1.py, line 25
USE_VERSION = "v4"  # Change to "v1", "v2", "v3", or "v4"
```

### Adding New Sequences
1. Define function in appropriate section of `testing_v1.py`
2. Add to `SEQUENCES` dictionary with human-friendly key
3. Test using interactive menu

### Parameters
All robot-specific parameters are in `params.py`:
- **Positions**: `HOME_ANGLES`, `ESPRESSO_HOME`, etc.
- **Offsets**: Movement deltas for safety margins
- **Timings**: Delays for settling, pouring, frothing
- **Gripper**: Force/position constants per cup size

## 📝 Key Files Explained

| File | Purpose |
|------|---------|
| `testing_v1.py` | Main automation sequences and interactive interface |
| `manipulate_node_v4.py` | Latest motion control implementation |
| `params.py` | Centralized configuration constants |
| `computer_vision.py` | ArUco detection and cup sensing |
| `servo_controller.py` | Low-level servo communication |
| `calibration_node.py` | Hand-eye calibration routines |

## 🐛 Troubleshooting

### Robot Not Responding
```bash
# Check servo connection
ros2 topic echo /dobot_status

# Verify servo service
ros2 service call /servo_j dobot_msgs_v3/srv/ServoJ "..."
```

### Cup Detection Issues
- Ensure proper lighting conditions
- Verify depth camera is publishing: `ros2 topic hz /camera/depth/image_rect_raw`
- Check ArUco markers are visible and undamaged

### Motion Planning Failures
- Verify MoveIt2 is running: `ros2 node list | grep move_group`
- Check joint limits in `nova5_moveit` package
- Review collision geometries in URDF

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test thoroughly with actual hardware
4. Submit a pull request with detailed description

## 📧 Contact

For questions or collaboration:
- GitHub: [@adeel1608](https://github.com/adeel1608)

---

**Built with ❤️ and ☕ for the future of automated coffee**

