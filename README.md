# Dobot Nova 5 BARNS Handover README

## 1. Purpose

This workspace runs the Dobot Nova 5 BARNS coffee automation system.

The full runtime stack is:

1. Dobot robot bringup
2. MoveIt planning
3. Servo action bridge for real robot execution
4. Orbbec Gemini camera
5. ArUco perception
6. Obstacle generator
7. Motion debug GUI
8. Pick-and-place automation scripts

The latest project repository is:

```bash
https://github.com/adeel1608/dobot-nova5-barns
```

---

## 2. Machine Prerequisites

Use:

```text
Ubuntu 22.04
ROS 2 Humble
Python 3.10+
Dobot Nova 5 reachable on network
Orbbec Gemini 330/335 camera connected over USB
```

Install base tools:

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake gnome-terminal sshpass \
  python3-pip python3-rosdep python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-yaml python3-opencv \
  ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-tf-transformations
```

Initialize rosdep if it has not already been initialized:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

---

## 3. Clone the BARNS Workspace Properly

Clone the full repository as the ROS workspace root.

Do **not** clone it inside another `src/` folder. The repository already contains its own `src/` directory.

```bash
cd ~
git clone https://github.com/adeel1608/dobot-nova5-barns.git barns_ws
cd ~/barns_ws
```

Install ROS dependencies:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
cd ~/barns_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source after building:

```bash
source ~/barns_ws/install/setup.bash
```

To update later:

```bash
cd ~/barns_ws
git pull
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Orbbec Camera Full Setup

The Orbbec camera is built in a separate workspace:

```bash
~/orbbec_ws
```

Create and run the setup script:

```bash
cat > ~/setup_orbbec.sh <<'EOF'
#!/bin/bash
set -e

sudo apt update && sudo apt install -y \
  git build-essential cmake python3-colcon-common-extensions \
  libgflags-dev nlohmann-json3-dev libdw-dev libomp-dev freeglut3-dev \
  libgoogle-glog-dev libusb-1.0-0-dev libudev-dev libeigen3-dev \
  libopencv-dev libgtk-3-0 libgl1-mesa-glx libglib2.0-0 libsm6 libxrender1 libxext6

mkdir -p ~/orbbec_ws/src
cd ~/orbbec_ws/src

if [ ! -d OrbbecSDK_ROS2 ]; then
  git clone --branch v1.5.14 --depth 1 https://github.com/orbbec/OrbbecSDK_ROS2.git
else
  cd OrbbecSDK_ROS2
  git fetch --depth 1 origin v1.5.14
  git checkout v1.5.14
  cd ..
fi

cd ~/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
  SDK_LIB_DIR=x64
elif [ "$ARCH" = "aarch64" ]; then
  SDK_LIB_DIR=arm64
else
  SDK_LIB_DIR=arm32
fi

sudo cp -v "$SDK_LIB_DIR"/*.so* /usr/local/lib/
cd /usr/local/lib

ORBBEC_LIB=$(ls libOrbbecSDK.so.*.* 2>/dev/null | head -1)
sudo ln -sf "$ORBBEC_LIB" libOrbbecSDK.so
sudo ln -sf "$ORBBEC_LIB" libOrbbecSDK.so.1

DEPTH_LIB=$(ls libdepthengine.so.*.* 2>/dev/null | head -1)
sudo ln -sf "$DEPTH_LIB" libdepthengine.so

echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/orbbec.conf
sudo ldconfig

cd ~/orbbec_ws
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export LIBRARY_PATH=/usr/local/lib:${LIBRARY_PATH:-}

colcon build --symlink-install \
  --packages-select orbbec_camera \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_LIBRARY_PATH=/usr/local/lib \
    -DCMAKE_PREFIX_PATH=/usr/local \
  --event-handlers console_direct+

cd ~/orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

cd ~/orbbec_ws
source install/setup.bash

echo "=== DONE ==="
echo "Test camera with:"
echo "ros2 launch orbbec_camera gemini_330_series.launch.py"
EOF

chmod +x ~/setup_orbbec.sh
~/setup_orbbec.sh
```

Test after installation:

```bash
source /opt/ros/humble/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

---

## 5. Environment Setup and Sourcing

Create one environment file for the system:

```bash
cat > ~/barns_ws/barns_env.sh <<'EOF'
#!/bin/bash

export BARNS_WS="$HOME/barns_ws"
export ORBBEC_WS="$HOME/orbbec_ws"

# Change this to the actual Dobot controller IP.
export IP_address="<ROBOT_IP_ADDRESS>"

# Robot type used by MoveIt and servo_action.
export DOBOT_TYPE="nova5"

source /opt/ros/humble/setup.bash

if [ -f "$ORBBEC_WS/install/setup.bash" ]; then
  source "$ORBBEC_WS/install/setup.bash"
fi

if [ -f "$BARNS_WS/install/setup.bash" ]; then
  source "$BARNS_WS/install/setup.bash"
fi

export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export LIBRARY_PATH=/usr/local/lib:${LIBRARY_PATH:-}
EOF

chmod +x ~/barns_ws/barns_env.sh
```

Edit the robot IP:

```bash
nano ~/barns_ws/barns_env.sh
```

Replace:

```bash
export IP_address="<ROBOT_IP_ADDRESS>"
```

with the real Dobot IP.

Add the environment file to `.bashrc`:

```bash
grep -qxF 'source ~/barns_ws/barns_env.sh' ~/.bashrc || echo 'source ~/barns_ws/barns_env.sh' >> ~/.bashrc
source ~/.bashrc
```

Check the environment:

```bash
echo $IP_address
echo $DOBOT_TYPE
ros2 pkg list | grep dobot_bringup_v3
ros2 pkg list | grep orbbec_camera
```

---

## 6. Full System Launch Script

Create the startup script:

```bash
cat > ~/barns_ws/start_barns_system.sh <<'EOF'
#!/bin/bash

set -e

BARNS_WS="${BARNS_WS:-$HOME/barns_ws}"
ENV_FILE="$BARNS_WS/barns_env.sh"

if [ ! -f "$ENV_FILE" ]; then
  echo "Missing environment file: $ENV_FILE"
  exit 1
fi

source "$ENV_FILE"
cd "$BARNS_WS"

TERM_PREFIX="source '$ENV_FILE'; cd '$BARNS_WS';"

open_terminal() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="$title" -- bash -lc "$TERM_PREFIX echo '$title'; $command; exec bash"
}

echo "Killing any existing nodes..."
pkill -f "ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py" || true
pkill -f "ros2 run servo_action action_move_client_reality" || true
pkill -f "ros2 launch dobot_moveit dobot_moveit.launch.py" || true
pkill -f "ros2 run servo_action action_move_server_reality" || true
pkill -f "ros2 launch orbbec_camera gemini_330_series.launch.py" || true
pkill -f "ros2 run pickn_place aruco_perception" || true
pkill -f "ros2 run pickn_place obstacle_generator" || true
pkill -f "ros2 launch motion_debug motion_debug.launch.py" || true

sleep 1.0

# Step 1: Dobot bringup
open_terminal "Dobot Bringup" "ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py"
sleep 1.0

# Step 2: Temporary action client to sync current joint state
open_terminal "Servo Action Client Sync" "ros2 run servo_action action_move_client_reality"
sleep 1.0

# Step 3: MoveIt
open_terminal "Dobot MoveIt" "ros2 launch dobot_moveit dobot_moveit.launch.py"
sleep 1.0

# Step 4: Kill temporary client and start real action server
pkill -f "ros2 run servo_action action_move_client_reality" || true
sleep 1.0
open_terminal "Servo Action Server" "ros2 run servo_action action_move_server_reality"
sleep 1.0

# Step 5: Camera
open_terminal "Orbbec Gemini" "ros2 launch orbbec_camera gemini_330_series.launch.py"
sleep 10.0

# Step 6: ArUco perception
open_terminal "ArUco Perception" "ros2 run pickn_place aruco_perception"
sleep 1.0

# Step 7: Base obstacle
open_terminal "Obstacle Generator" "ros2 run pickn_place obstacle_generator"
sleep 1.0

# Step 8: Motion debug GUI
open_terminal "Motion Debug" "ros2 launch motion_debug motion_debug.launch.py"

echo "System launch complete."
EOF

chmod +x ~/barns_ws/start_barns_system.sh
```

Run the system:

```bash
cd ~/barns_ws
./start_barns_system.sh
```

---

## 7. Manual Launch Order

Use this only if the startup script is not being used.

```bash
source ~/barns_ws/barns_env.sh
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

```bash
source ~/barns_ws/barns_env.sh
ros2 run servo_action action_move_client_reality
```

```bash
source ~/barns_ws/barns_env.sh
ros2 launch dobot_moveit dobot_moveit.launch.py
```

After MoveIt starts, stop the temporary action client:

```bash
pkill -f "ros2 run servo_action action_move_client_reality"
```

Start the execution server:

```bash
source ~/barns_ws/barns_env.sh
ros2 run servo_action action_move_server_reality
```

Start the camera:

```bash
source ~/barns_ws/barns_env.sh
ros2 launch orbbec_camera gemini_330_series.launch.py
```

Start ArUco perception:

```bash
source ~/barns_ws/barns_env.sh
ros2 run pickn_place aruco_perception
```

Start obstacles:

```bash
source ~/barns_ws/barns_env.sh
ros2 run pickn_place obstacle_generator
```

Start motion debug:

```bash
source ~/barns_ws/barns_env.sh
ros2 launch motion_debug motion_debug.launch.py
```

---

## 8. Teaching Commands

Tool teaching:

```bash
ros2 run pickn_place tool_mount_teach
```

Machine teaching:

```bash
ros2 run pickn_place machine_mount_teach
```

Tool teaching saves tool offsets to:

```text
tool_offset_points.yaml
```

Machine teaching saves machine offsets to:

```text
machine_offset_points.yaml
```

Use the ArUco marker name from:

```text
arucoID_name_config.yaml
```

Do not enter the numeric marker ID. Enter the configured marker name.

---

## 9. Running Pick-and-Place Automation

Source the workspace:

```bash
source ~/barns_ws/barns_env.sh
```

Run the interactive automation menu:

```bash
ros2 run pickn_place testing_v1
```

Inside the menu:

```text
list        show all available sequences
q           quit
kin         open Dobot kinematics tools menu
```

If `ros2 run pickn_place testing_v1` is not registered as a console script in the current build, run it directly:

```bash
cd ~/barns_ws
python3 src/pickn_place/pickn_place/testing_v1.py
```

---

## 10. Important Runtime Topics and Services

Dobot feedback:

```text
/joint_states_robot
```

MoveIt planned path:

```text
/display_planned_path
```

Servo action status:

```text
/servo_controller_status
/max_points
/sleep_timing
```

Dobot services used by motion:

```text
/dobot_bringup_v3/srv/ServoJ
/dobot_bringup_v3/srv/EnableRobot
/dobot_bringup_v3/srv/GetPose
/dobot_bringup_v3/srv/GetAngle
/dobot_bringup_v3/srv/JointMovJ
/dobot_bringup_v3/srv/MovJ
/dobot_bringup_v3/srv/MovL
/dobot_bringup_v3/srv/RelMovJ
/dobot_bringup_v3/srv/RelMovL
/dobot_bringup_v3/srv/Sync
/dobot_bringup_v3/srv/SetGripperPosition
/dobot_bringup_v3/srv/GetGripperPosition
```

Orbbec topics expected by perception:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth_to_color
```

ArUco TF chain:

```text
Link6 -> calibrated_camera_link
calibrated_camera_link -> <aruco_marker_name>
```

---

## 11. Key Files

```text
src/pickn_place/pickn_place/testing_v1.py
```

Main sequence file. It provides the interactive menu, drink workflows, cup handling, espresso handling, milk handling, calibration helpers, and robot test utilities.

```text
src/pickn_place/pickn_place/manipulate_node_v4.py
```

Main robot motion layer. It wraps Dobot services, MoveIt moves, TF lookups, tool approach/grab, machine approach/mount, gripper commands, and motion helpers.

```text
src/pickn_place/pickn_place/params.py
```

Central parameter file. It stores speed constants, gripper positions, home positions, cup parameters, machine positions, espresso parameters, milk parameters, slush parameters, and helper parsers.

---

## 12. `testing_v1.py` Function Guide

### Motion node and version handling

| Function | Purpose |
|---|---|
| `get_motion_node()` | Creates or returns the persistent robot motion node. |
| `run_skill()` | Calls a method from the selected motion node version. Main wrapper used by sequences. |
| `cleanup_motion_node()` | Safely destroys the persistent motion node and shuts down `rclpy`. |
| `show_version_info()` | Prints which manipulate node version is active. |
| `switch_version()` | Prints instructions for switching between motion node versions. |

### Home, calibration, and diagnostics

| Function | Purpose |
|---|---|
| `home()` | Moves the robot to a named compass home pose. |
| `return_back_to_home()` | Returns to a safe home pose based on current J1 angle. |
| `get_machine_position()` | Calibrates and saves machine marker positions. |
| `check_saved_data()` | Reads saved machine position data from YAML. |
| `check_aruco_status()` | Checks ArUco status and saved machine data. |
| `solution()` | Converts joints to Cartesian pose, applies offsets, and solves back to joints. |
| `solution_interactive()` | Prompts the user for joint/offset values and calls `solution()`. |
| `open_gripper()` | Opens the gripper. |
| `close_gripper()` | Closes the gripper. |
| `toggle_drag_mode()` | Toggles Dobot drag mode. |
| `reset_robot1()` | Restarts the `robot1` deployment on the NUC. |
| `reset_robot2()` | Restarts the `robot2` deployment on the NUC. |

### Paper cup functions

| Function | Purpose |
|---|---|
| `grab_paper_cup()` | Grabs a paper cup from the dispenser. |
| `place_paper_cup()` | Places a paper cup at a staging position. |
| `grab_paper_cup_arm1()` | Arm-1 paper cup grab routine with retry/detection. |
| `place_paper_cup_arm1()` | Arm-1 paper cup placement routine. |
| `dispense_paper_arm1_cup_station()` | Arm-1 grab-and-place paper cup routine. |
| `grab_paper_arm2_cup_station()` | Arm-2 paper cup grab routine. |
| `place_paper_arm2_cup_station()` | Arm-2 paper cup placement routine. |
| `dispense_paper_arm2_cup_station()` | Arm-2 grab-and-place paper cup routine. |
| `dispense_paper_cup_station()` | General paper cup dispense routine. |
| `pick_paper_cup_station()` | Picks a paper cup from a staging position. |
| `place_paper_cup_station()` | Places a paper cup to a staging position. |
| `place_paper_cup_sauces()` | Places paper cup at sauces station. |
| `pick_paper_cup_sauces()` | Picks paper cup from sauces station. |
| `place_paper_cup_milk()` | Places paper cup at milk station. |
| `pick_paper_cup_milk()` | Picks paper cup from milk station. |
| `pick_cup_for_hot_water()` | Picks cup and moves it to hot water position. |
| `return_cup_with_hot_water()` | Returns cup after hot water operation. |

### Espresso and portafilter functions

| Function | Purpose |
|---|---|
| `unmount()` | Removes portafilter from espresso machine. |
| `grinder()` | Moves portafilter to grinder and tamper positions. |
| `single_grinder()` | Grinder routine using single portafilter. |
| `double_grinder()` | Grinder routine using double portafilter. |
| `tamper()` | Picks portafilter/tool and performs tamper sequence. |
| `single_tamper()` | Tamper routine using single portafilter. |
| `double_tamper()` | Tamper routine using double portafilter. |
| `mount()` | Mounts portafilter back into espresso machine. |
| `grab_espresso_pitcher()` | Grabs espresso pitcher and closes gripper. |
| `pick_espresso_pitcher()` | Completes pitcher pickup after gripper close. |
| `pour_espresso_pitcher_cup_station()` | Pours espresso pitcher into selected cup station. |
| `get_hot_water()` | Moves to hot water target on espresso machine. |
| `with_hot_water()` | Performs hot water retreat/pour handling. |
| `return_espresso_pitcher()` | Returns espresso pitcher to machine. |
| `return_cleaned_espresso_pitcher()` | Returns pitcher after cleaning routine. |
| `unmount_single()` / `unmount_double()` | Convenience wrappers for single/double unmount. |
| `mount_single()` / `mount_double()` | Convenience wrappers for single/double mount. |
| `single_*` / `double_*` pitcher wrappers | Convenience wrappers for pitcher pickup, pour, return, and cleaning. |

### Angled espresso functions

| Function | Purpose |
|---|---|
| `angled_unmount()` | Angled-toolhead version of portafilter unmount. |
| `angled_grinder()` | Angled-toolhead grinder routine. |
| `angled_tamper()` | Angled-toolhead tamper routine. |
| `angled_mount()` | Angled-toolhead mount routine. |
| `angled_grab_espresso_pitcher()` | Angled-toolhead pitcher grab. |
| `angled_pick_espresso_pitcher()` | Angled-toolhead pitcher pickup. |
| `angled_pour_espresso_pitcher_cup_station()` | Angled-toolhead pitcher pour. |
| `angled_get_hot_water()` | Angled-toolhead hot water approach. |
| `angled_with_hot_water()` | Angled-toolhead hot water retreat. |
| `angled_return_espresso_pitcher()` | Angled-toolhead pitcher return. |
| `angled_return_cleaned_espresso_pitcher()` | Angled-toolhead cleaned pitcher return. |
| `angled_single_*` / `angled_double_*` wrappers | Convenience wrappers for angled single/double workflows. |

### Portafilter cleaning

| Function | Purpose |
|---|---|
| `clean_portafilter()` | Cleans the portafilter using trained cleaner poses. |
| `clean_portafilter_single()` | Single portafilter cleaning wrapper. |
| `clean_portafilter_double()` | Double portafilter cleaning wrapper. |
| `angled_clean_portafilter()` | Angled-toolhead cleaner routine. |
| `angled_clean_portafilter_single()` | Angled single cleaning wrapper. |
| `angled_clean_portafilter_double()` | Angled double cleaning wrapper. |

### Milk frothing

| Function | Purpose |
|---|---|
| `get_frother_position()` | Calibrates and records milk frother position. |
| `pick_frother()` | Picks the frother tool. |
| `place_frother_milk_station()` | Places frother at milk station. |
| `pick_frother_milk_station()` | Picks frother from milk station. |
| `mount_frother()` | Mounts frother to steam wand. |
| `unmount_and_swirl_milk()` | Unmounts frother and performs swirl motion. |
| `pour_milk_cup_station()` | Pours milk into selected cup station. |
| `clean_milk_pitcher()` | Runs milk pitcher cleaning routine. |
| `return_frother()` | Returns frother to original location. |

### Plastic cup, ice, and slush functions

| Function | Purpose |
|---|---|
| `dispense_plastic_cup()` | Dispenses plastic cup. |
| `go_to_ice()` | Moves cup to ice dispenser. |
| `go_home_with_ice()` | Returns home after ice operation. |
| `place_plastic_cup_station()` | Places plastic cup at station. |
| `pick_plastic_cup_station()` | Picks plastic cup from station. |
| `place_plastic_cup_sauces()` | Places plastic cup at sauces station. |
| `pick_plastic_cup_sauces()` | Picks plastic cup from sauces station. |
| `place_plastic_cup_milk()` | Places plastic cup at milk station. |
| `pick_plastic_cup_milk()` | Picks plastic cup from milk station. |
| `get_slush()` | Moves cup to slush dispenser. |
| `place_slush()` | Places cup after slush operation. |

### External station call functions

| Function | Purpose |
|---|---|
| `call_tamper()` | Triggers tamper station command. |
| `call_coffee_machine()` | Triggers coffee machine command. |
| `call_hot_water()` | Triggers hot water command. |
| `call_coffee_purge()` | Triggers coffee purge command. |
| `call_frother()` | Triggers frother command. |
| `call_grinder()` | Triggers grinder command. |
| `call_ice()` | Triggers ice command. |
| `call_milk_syrup()` | Triggers milk/syrup command. |
| `call_slush()` | Triggers slush command. |

### Complete recipes, training, and tests

| Function | Purpose |
|---|---|
| `espresso()` | Full espresso workflow. |
| `espresso_angled()` | Full espresso workflow using angled toolhead. |
| `americano()` | Espresso plus hot water workflow. |
| `multi_espresso()` | Multi-cup espresso workflow. |
| `milk_frothing()` | Milk frothing workflow. |
| `milk_1()` / `milk_2()` / `milk_3()` / `milk_4()` | Direct milk pour routines for station 1 to 4. |
| `slushie()` | Full slushie workflow. |
| `espresso_training()` | Step-by-step espresso training routine. |
| `milk_training()` | Step-by-step milk training routine. |
| `angled_espresso_training()` | Step-by-step angled espresso training routine. |
| `espresso_port_1_training()` | Training for espresso port 1. |
| `espresso_port_2_training()` | Training for espresso port 2. |
| `angled_espresso_port_1_training()` | Training for angled port 1. |
| `angled_espresso_port_2_training()` | Training for angled port 2. |
| `angled_grinder_training()` | Training for angled grinder routine. |
| `angled_cleaner_training()` | Training for angled cleaner routine. |
| `test()` / `test_arm1()` / `test_arm2()` | Robot test routines. |
| `test_both_port()` | Test routine for both port setups. |
| `test_plastic_cup()` | Plastic cup test routine. |
| `test_paper_cup()` | Paper cup test routine. |
| `robot_arm_test()` | Deterministic timing test for robot motion. |
| `hello()` | Basic sanity command. |
| `run_kinematics_tools_menu()` | Interactive Dobot kinematics service menu. |
| `_main()` | Main CLI loop for the interactive menu. |

---

## 13. `manipulate_node_v4.py` Function Guide

### Classes

| Class | Purpose |
|---|---|
| `RobotMotionError` | Base exception for robot motion failures. |
| `SyncFailureError` | Raised when sync fails after retries. |
| `MovementFailureError` | Raised when movement fails after retries. |
| `robot_perception` | Temporary TF/perception helper node for stable transform acquisition. |
| `robot_motion` | Main motion node used by automation sequences. |

### `robot_perception` methods

| Method | Purpose |
|---|---|
| `get_tf()` | Looks up a TF frame with retries. |
| `acquire_target_transform()` | Collects stable TF samples and returns an averaged pose. |
| `destroy_node()` | Cleans up TF listener and executor thread. |

### `robot_motion` core methods

| Method | Purpose |
|---|---|
| `health_check()` | Checks critical service availability. |
| `is_robot_likely_responsive()` | Uses recent success/failure state to estimate robot responsiveness. |
| `log_robot_status()` | Logs current service and failure status. |
| `reset_robot_connection()` | Recreates critical service clients after connection issues. |
| `safe_log()` | Logs safely even during shutdown or ROS context issues. |
| `verify_joint_positions()` | Compares current joints against expected joints. |
| `get_machine_position()` | Captures stable machine marker pose and writes it to YAML. |
| `release_tension()` | Uses drag mode to release mechanical tension. |
| `inverse_solution()` | Calls Dobot inverse kinematics service. |
| `positive_solution()` | Calls Dobot forward kinematics service. |
| `sync()` | Waits until Dobot motion is complete. |
| `set_gripper_position()` | Commands gripper and verifies final gripper position. |
| `move_to()` | Moves toward a TF target while stopping at a set distance. |
| `enforce_rxry()` | Forces Link6 orientation while keeping portafilter position stable. |
| `enforce_rxry_angled()` | Angled-toolhead version of orientation enforcement. |
| `move_portafilter_arc()` | Rotates portafilter through an arc using MoveIt path logic. |
| `move_portafilter_arc_movL()` | Portafilter arc motion using linear Cartesian command. |
| `move_portafilter_arc_movJ()` | Portafilter arc motion using `MovJ`. |
| `move_portafilter_arc_movJ_angled()` | Angled-toolhead portafilter arc using `MovJ`. |
| `move_portafilter_arc_tool()` | Configures TCP and performs portafilter arc using tool frame. |
| `move_portafilter_arc_tool_angled()` | Angled version of TCP-based portafilter arc. |
| `enforce_rxry_moveit()` | MoveIt-based version of orientation enforcement. |
| `moveEE()` | Relative Cartesian move using `RelMovL`. |
| `moveEE_movJ()` | Relative Cartesian move by reading current pose and sending `MovJ`. |
| `moveJ_deg()` | Relative joint move using `RelMovJ`. |
| `gotoEE()` | Absolute Cartesian move using `MovL`. |
| `gotoEE_movJ()` | Absolute Cartesian move using `MovJ`. |
| `gotoJ_deg()` | Absolute joint move using `JointMovJ`. |
| `approach_tool()` | Moves to a tool's saved approach pose. |
| `grab_tool()` | Moves to a tool's saved grab pose. |
| `approach_machine()` | Moves to a machine point's saved approach pose. |
| `mount_machine()` | Moves to a machine point's saved mount pose. |
| `set_DO()` | Sets Dobot digital output. |
| `toggle_drag_mode()` | Alternates StartDrag/StopDrag. |
| `move_arc()` | Executes Dobot Arc command from offset poses. |
| `move_circle()` | Executes Dobot Circle3 command from offset poses. |
| `set_speed_factor()` | Sets Dobot speed factor. |
| `use_tool()` | Selects Dobot tool index. |
| `set_tool()` | Configures Dobot TCP/tool data. |
| `wait_for_joint_state()` | Waits for a JointState message. |
| `wait_for_servo_ready()` | Waits for servo action status to become READY. |
| `current_angles()` | Returns current joint angles. |
| `current_pose()` | Returns current Cartesian pose. |

### Module-level helpers

| Function | Purpose |
|---|---|
| `get_transform_list()` | Converts `TransformStamped` to list format. |
| `init_motion_node()` | Initializes global motion node. |
| `cleanup_motion_node()` | Cleans up global motion node. |
| `run_skill_with_node()` | Executes a skill using a provided node. |
| `run_skill()` | Executes a skill using the global motion node. |
| `execute_sequence()` | Runs a sequence with managed motion node setup/cleanup. |

---

## 14. `params.py` Function and Config Guide

### Helper functions

| Function | Purpose |
|---|---|
| `_set_cup_dispensed()` | Sets internal flag after plastic cup/ice dispense. |
| `_check_and_clear_cup_dispensed()` | Reads and clears cup-dispensed flag. |
| `_extract_cup_position()` | Extracts `cup_position` from new or legacy params. |
| `_extract_cups_dict()` | Extracts cup dictionary from order params. |
| `_normalize_cup_size()` | Converts cup codes like `cup_H12` or `cup_C16` to `12oz` / `16oz`. |
| `validate_port()` | Validates espresso port name. |
| `validate_stage()` | Validates stage number. |
| `validate_cup_size()` | Validates cup size. |
| `get_param_with_default()` | Reads parameter with fallback value. |
| `log_step()` | Prints formatted step log. |
| `log_success()` | Prints success log. |
| `log_error()` | Prints error log. |
| `log_warning()` | Prints warning log. |
| `log_info()` | Prints info log. |

### Main config groups

| Config Group | Purpose |
|---|---|
| `SPEED_*` | Robot speed presets. |
| `GRIPPER_*` | Standard gripper positions. |
| `DELAY_*` | Common timing delays. |
| `HOME_ANGLES` | Named compass home poses. |
| `HOME_CALIBRATION_PARAMS` | Joint poses used during machine calibration. |
| `ESPRESSO_*` | Espresso, grinder, pitcher, hot water, and portafilter parameters. |
| `PAPER_*` | Paper cup navigation, pickup, placement, and station parameters. |
| `PLASTIC_*` | Plastic cup dispense, pickup, placement, and station parameters. |
| `MILK_*` | Milk frother, swirling, pouring, and cleaning parameters. |
| `SLUSH_*` | Slush machine motion and dispenser parameters. |
| `VALID_*` | Allowed values for ports, cups, stages, and home positions. |
| `DEFAULT_*` | Default values used when order params are missing. |

---

## 15. Debug Commands

Check robot joint feedback:

```bash
ros2 topic echo /joint_states_robot
```

Check Dobot services:

```bash
ros2 service list | grep dobot_bringup_v3
```

Check servo status:

```bash
ros2 service call /get_servo_status std_srvs/srv/Trigger
```

Reset servo action if stuck:

```bash
ros2 service call /emergency_reset std_srvs/srv/Trigger
```

Check camera topics:

```bash
ros2 topic list | grep camera
```

Check TF tree:

```bash
ros2 run tf2_tools view_frames
```

Run ArUco perception with GUI:

```bash
ros2 run pickn_place aruco_perception --ros-args -p visualize:=true
```

---

## 16. Notes for Handover

- Source `~/barns_ws/barns_env.sh` in every terminal.
- Keep `DOBOT_TYPE=nova5` unless the MoveIt package/controller name changes.
- `IP_address` must match the real Dobot controller IP.
- Start Dobot bringup before MoveIt and automation scripts.
- Start Orbbec camera before ArUco perception.
- `action_move_client_reality` is temporary during startup and is killed after MoveIt gets the current joint state.
- `action_move_server_reality` must stay running during real robot execution.
- If MoveIt starts with the wrong robot pose, rerun `action_move_client_reality` briefly.
- If servo action is stuck, call `/emergency_reset`.
- Teaching files write offsets to both source and install locations. Rebuild/source if a new terminal does not see updated YAML data.
