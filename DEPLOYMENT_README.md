# BARNS Deployment README

## Purpose

This README explains how to deploy the BARNS Dobot Nova 5 robot stack using Docker and Docker Compose.

It covers:

- Cloning the project
- Host prerequisites
- Robot 1 Docker image build flow
- Robot 1 and Robot 2 Docker Compose services
- Robot 1 production startup script
- Runtime environment variables
- Device mappings
- Build and run commands
- Startup sequence
- Script function reference
- Troubleshooting
- TLDR

Use this README for deployment and operations. Use the normal project README for package-level ROS usage and development notes.

---

## 1. Repository Clone

The canonical repository for deployment is:

```text
https://github.com/QSS-AI-Robotics/BARNS
```

This repository may be private. Make sure the deployment machine is logged in with a GitHub account that has access to the `QSS-AI-Robotics` organization and the `BARNS` repository.

### Option A: Clone with HTTPS

```bash
cd ~
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS
```

### Option B: Clone with SSH

Use this when SSH keys are already configured for the GitHub account.

```bash
cd ~
git clone git@github.com:QSS-AI-Robotics/BARNS.git
cd BARNS
```

### If clone returns 404 or permission denied

A GitHub `404` on a private repo usually means the current account does not have access, or Git is not authenticated. Check access before debugging the deployment scripts.

For HTTPS authentication:

```bash
gh auth login
gh auth status
```

For SSH authentication:

```bash
ssh -T git@github.com
```

Update an existing clone:

```bash
cd ~/BARNS
git pull origin main
```

Check the current branch and last commit:

```bash
git branch --show-current
git log -1 --oneline
```

The expected project layout for Docker deployment includes:

```text
BARNS/
├── services/
│   └── robot_container/
│       └── ros_ws/
│           └── src/
├── shared/
├── docker-compose.arms.yml
├── robot1-startup.sh
└── docker/
```

Some local/dev deployments may use a different compose filename. The Robot 1 startup script specifically expects:

```text
docker-compose.arms.yml
```

in the project root.

---

## 2. Host Prerequisites

The deployment host should have:

```text
Ubuntu 22.04
Docker Engine
Docker Compose plugin
Git
USB access to Dobot / Orbbec devices
Network access to the robot controller
```

Install basic host tools:

```bash
sudo apt update
sudo apt install -y git curl ca-certificates docker.io docker-compose-plugin usbutils v4l-utils
```

Add your user to the Docker group if needed:

```bash
sudo usermod -aG docker $USER
```

Log out and log back in after changing Docker group membership.

Check Docker:

```bash
docker info
docker compose version
```

For GUI/X11 tools from containers, allow local root connections:

```bash
xhost +local:root
```

---

## 3. Deployment Modes

There are three deployment modes in this project.

### Mode A: Local Docker Image Build

Use this when you only want to build the Robot 1 image locally.

This is handled by the Robot 1 build script.

### Mode B: Docker Compose Robot Services

Use this when you want Robot 1 and Robot 2 services started through Docker Compose.

The provided compose configuration defines:

```text
robot1 -> oms_robot_1
robot2 -> oms_robot_2
```

### Mode C: Robot 1 Production Startup Script

Use this when Robot 1 runs on a dedicated PC that also starts Docker services, RabbitMQ, the robot stack, camera, perception, MoveIt, servo action, obstacle generation, and OMS service.

This is handled by:

```bash
./robot1-startup.sh
```

---

# Part A - Robot 1 Docker Image Build

## 4. Robot 1 Build Script

The Robot 1 build script builds a local Docker image without pushing it to a registry.

Default image name:

```text
barns-robot1:latest
```

The image name and tag can be overridden using environment variables:

```bash
IMAGE_NAME=<name> IMAGE_TAG=<tag> ./build-robot1.sh
```

Example for matching a Compose-style image name:

```bash
IMAGE_NAME=oms_robot_1 IMAGE_TAG=dev ./build-robot1.sh
```

## 5. Build Script Requirements

The build script expects:

```text
Dockerfile.robot1
```

to exist in the same directory as the build script.

It also checks for these project directories:

```text
services/robot_container/ros_ws/src
shared
```

The script calculates:

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
```

This means the script can be called from another directory, but it expects the repository structure to be correct relative to the script location.

## 6. Build Commands

Normal cached build:

```bash
./build-robot1.sh
```

Full rebuild with no Docker layer cache:

```bash
./build-robot1.sh --no-cache
```

Show help:

```bash
./build-robot1.sh --help
```

Optional faster build on a machine with enough RAM:

```bash
ROBOT_COLCON_PARALLEL=4 ROBOT_MAKE_JOBS=4 ./build-robot1.sh
```

The build script passes these into Docker as build args:

```text
COLCON_PARALLEL_WORKERS
MAKE_JOBS
```

Default values are lower to avoid high RAM usage.

## 7. Build Cache Behavior

The build script is designed to benefit from Docker layer caching.

Expected behavior:

```text
Stable ROS workspace unchanged -> full colcon build is skipped by Docker cache
pickn_place changes          -> only pickn_place rebuilds
oms_v1 changes               -> Python files are re-copied late, usually very fast
--no-cache                   -> everything rebuilds from scratch
```

Use `--no-cache` only when the image is broken, dependencies changed, or Docker cache is suspected to be stale.

---

# Part B - Dockerfile.robot1 Image Details

## 8. Base Image

The Robot 1 Dockerfile is based on:

```text
ubuntu:22.04
```

It installs:

```text
ROS 2 Humble
ros-base
MoveIt
ros2_control
ros2_controllers
Orbbec camera dependencies
Python perception dependencies
Dobot ROS workspace packages
OMS Python service files
```

## 9. Workspace Location Inside Image

The Dockerfile uses:

```text
/opt/barns-robot/ros_ws
```

as the main ROS workspace inside the image.

The environment variable is:

```bash
WORKSPACE_DIR=/opt/barns-robot/ros_ws
```

## 10. Packages Built Into the Image

The Dockerfile first copies and builds stable ROS packages:

```text
dobot_bringup_v3
dobot_moveit
dobot_msgs_v3
dobot_rviz
moveit2
moveit_msgs
moveit_resources
nova5_moveit
pymoveit2
ros2_control
ros2_controllers
servo_action
shared
```

Then it separately copies and builds:

```text
pickn_place
```

Finally, it copies:

```text
oms_v1
```

late in the Docker build because it is pure Python and does not need a full colcon rebuild every time.

## 11. Orbbec SDK Setup in Dockerfile

The Dockerfile clones Orbbec SDK ROS 2 release:

```bash
git clone --branch v1.5.14 --depth 1 https://github.com/orbbec/OrbbecSDK_ROS2.git
```

The SDK libraries are copied to:

```text
/usr/local/lib
```

The Dockerfile creates symlinks for:

```text
libOrbbecSDK.so
libOrbbecSDK.so.1
libdepthengine.so
```

and registers `/usr/local/lib` using:

```text
/etc/ld.so.conf.d/orbbec.conf
ldconfig
```

This means the Orbbec driver should work inside the image without separately installing the SDK on the host, as long as USB devices are passed through correctly.

## 12. Python Dependencies in Image

The Dockerfile installs key Python packages:

```text
opencv-contrib-python==4.10.0.84
numpy==1.23.5
scipy==1.11.4
transformations==2025.1.1
aio-pika==9.4.3
pika==1.3.2
pyorbbecsdk==1.3.2
```

## 13. Image Environment Defaults

Robot 1 Dockerfile defaults:

```bash
ROBOT_ID=1
ROS_DOMAIN_ID=0
DOBOT_TYPE=nova5
IP_ADDRESS=192.168.200.249
CAMERA_SERIAL_NUMBER=CP1Z842000YW
CAM_NAME=cam0
DEVICE_ID=1
DEVICE_NUM=1
ORBBEC_CONNECTION_DELAY=3000
USBFS_MEMORY_MB=128
RABBITMQ_URL=amqp://admin:admin123@rabbitmq:5672/
```

These can be overridden by Docker Compose, Kubernetes, or command-line environment variables.

## 14. Image Default Command

The Dockerfile default command is:

```bash
source /opt/barns-robot/ros_ws/setup_robot_env.sh && /opt/barns-robot/robot1-startup.sh robot-only
```

This means the baked image starts only the robot stack by default, assuming external Docker services are already running.

---

# Part C - Docker Compose Deployment

## 15. Docker Compose Runtime Settings

Both Robot 1 and Robot 2 containers use:

```yaml
network_mode: host
ipc: host
privileged: true
shm_size: "1g"
```

These settings are important.

### `network_mode: host`

Required for ROS 2 / Fast-DDS discovery and communication.

### `ipc: host`

Helps ROS 2 and shared memory transport behavior.

### `privileged: true`

Required for USB camera, serial, and hardware access.

### `shm_size: "1g"`

Provides more shared memory for camera streams and ROS 2 processes.

## 16. Robot 1 Compose Service

Robot 1 service:

```yaml
robot1:
  image: oms_robot_1:dev
  container_name: oms_robot_1
```

Robot 1 important environment variables:

```bash
DISPLAY=${DISPLAY}
QT_X11_NO_MITSHM=1
DOBOT_TYPE=nova5
IP_address=192.168.100.249
ROS_DOMAIN_ID=0
ROBOT_ID=1
RABBITMQ_URL=amqp://admin:admin123@localhost:5672/
```

Robot 1 devices:

```text
/dev/video0  -> /dev/video0
/dev/video1  -> /dev/video1
/dev/video2  -> /dev/video2
/dev/video3  -> /dev/video3
/dev/video4  -> /dev/video4
/dev/video5  -> /dev/video5
/dev/video6  -> /dev/video6
/dev/video7  -> /dev/video7
/dev/media0  -> /dev/media0
/dev/media1  -> /dev/media1
/dev/ttyUSB0 -> /dev/ttyUSB0
```

Robot 1 debug/VNC port:

```text
5901:5901
```

## 17. Robot 2 Compose Service

Robot 2 service:

```yaml
robot2:
  image: oms_robot_2:dev
  container_name: oms_robot_2
```

Robot 2 important environment variables:

```bash
DISPLAY=${DISPLAY}
QT_X11_NO_MITSHM=1
DOBOT_TYPE=nova5
IP_address=192.168.100.248
ROS_DOMAIN_ID=1
ROBOT_ID=2
RABBITMQ_URL=amqp://admin:admin123@localhost:5672/
```

Robot 2 devices:

```text
/dev/video10 -> /dev/video10
/dev/video11 -> /dev/video11
/dev/video12 -> /dev/video12
/dev/video13 -> /dev/video13
/dev/video14 -> /dev/video14
/dev/video15 -> /dev/video15
/dev/video16 -> /dev/video16
/dev/video17 -> /dev/video17
/dev/media3  -> /dev/media3
/dev/media4  -> /dev/media4
/dev/ttyUSB1 -> /dev/ttyUSB1
```

Robot 2 debug/VNC port:

```text
5902:5901
```

## 18. Compose Volumes

Both robot containers mount:

```yaml
./ros_ws:/root/ros_ws
../../shared:/app/shared:ro
/tmp/.X11-unix:/tmp/.X11-unix:rw
```

Meaning:

```text
./ros_ws                  -> ROS workspace inside container at /root/ros_ws
../../shared              -> shared RabbitMQ/client code at /app/shared, read-only
/tmp/.X11-unix            -> X11 socket for GUI tools
```

## 19. Compose Startup Command

Both robot containers run:

```bash
/root/ros_ws/run_full_stack.sh &
cd /root/ros_ws/src/oms_v1
python3 -m oms_v1.app --service
```

This starts the ROS robot stack in the background, then starts the OMS service in the same container.

## 20. Running Docker Compose

Start all services:

```bash
docker compose -f docker-compose.arms.yml up -d --build
```

Start only Robot 1:

```bash
docker compose -f docker-compose.arms.yml up -d --build robot1
```

Start only Robot 2:

```bash
docker compose -f docker-compose.arms.yml up -d --build robot2
```

View logs:

```bash
docker compose -f docker-compose.arms.yml logs -f robot1
```

```bash
docker compose -f docker-compose.arms.yml logs -f robot2
```

Open shell into Robot 1:

```bash
docker exec -it oms_robot_1 bash
```

Open shell into Robot 2:

```bash
docker exec -it oms_robot_2 bash
```

Stop services:

```bash
docker compose -f docker-compose.arms.yml down
```

Stop services and remove volumes:

```bash
docker compose -f docker-compose.arms.yml down --volumes
```

---

# Part D - Robot 1 Production Startup Script

## 21. Robot 1 Startup Script Purpose

The Robot 1 startup script is for a separate PC running Docker services and Robot 1.

It handles:

```text
Docker services startup
RabbitMQ readiness
RabbitMQ queue cleanup
ROS 2 daemon cleanup
Dobot robot initialization
Orbbec camera launch
MoveIt headless launch
Servo action server launch
ArUco perception launch
Obstacle generator launch
OMS service launch
Camera/perception monitoring
Camera recovery and USB reset
Full robot stack relaunch when Dobot Sync fails
```

## 22. Startup Script Commands

Default full startup:

```bash
./robot1-startup.sh start
```

Start only Docker services:

```bash
./robot1-startup.sh docker-only
```

Start only Robot 1, assuming Docker services are already running:

```bash
./robot1-startup.sh robot-only
```

Stop Robot 1 and Docker services:

```bash
./robot1-startup.sh stop
```

Show help:

```bash
./robot1-startup.sh --help
```

## 23. Startup Script Environment Variables

The Robot 1 script supports these environment variables:

```bash
WORKSPACE_DIR
DOCKER_HOST_IP
DOBOT_TYPE
IP_ADDRESS
CAMERA_SERIAL_NUMBER
```

Defaults:

```bash
WORKSPACE_DIR=./services/robot_container/ros_ws
DOCKER_HOST_IP=192.168.200.129
DOBOT_TYPE=nova5
IP_ADDRESS=192.168.200.249
ROS_DOMAIN_ID=0
ROBOT_ID=1
CAMERA_SERIAL_NUMBER=CP1Z842000YW
CAM_NAME=cam0
DEVICE_ID=1
DEVICE_NUM=1
ORBBEC_CONNECTION_DELAY=3000
USBFS_MEMORY_MB=128
```

Example override:

```bash
IP_ADDRESS=192.168.200.249 DOCKER_HOST_IP=192.168.200.129 ./robot1-startup.sh start
```

## 24. Important IP Variable Naming

There are two similar Dobot IP variable names:

```bash
IP_ADDRESS
IP_address
```

The Robot 1 startup script uses:

```bash
IP_ADDRESS
```

then exports it for the ROS Dobot bringup as:

```bash
IP_address="$IP_ADDRESS"
```

The Dobot bringup Python code expects:

```bash
IP_address
```

Do not rename this without checking the Dobot bringup package.

## 25. Robot Network Notes

There are two IP ranges shown in the deployment files:

```text
192.168.100.x
192.168.200.x
```

Use the IP range that matches the actual robot network for the deployment machine.

The Docker Compose example uses:

```text
Robot 1: 192.168.100.249
Robot 2: 192.168.100.248
```

The Robot 1 startup script and Robot 1 Dockerfile default to:

```text
Robot 1: 192.168.200.249
```

Before deployment, confirm robot reachability:

```bash
ping <robot-ip>
```

## 26. Robot 1 Startup Order

The Robot 1 startup script runs this order:

```text
1. Check workspace exists
2. Start Docker services
3. Wait for RabbitMQ and core services
4. Clean stale RabbitMQ queues
5. Clean ROS 2 daemon/runtime state
6. Source robot environment
7. Export robot variables
8. Launch Dobot bringup
9. Launch Orbbec camera
10. Check ROS 2 connectivity
11. Initialize Dobot
12. Move robot to a safe compass home based on J1 angle
13. Set up Modbus / gripper registers
14. Launch MoveIt headless
15. Launch servo_action server
16. Launch ArUco perception
17. Launch obstacle generator
18. Probe Dobot Sync service
19. Launch OMS service
20. Keep monitoring camera/perception health
```

## 27. Dobot Initialization Sequence

The script initializes the robot using these Dobot services:

```bash
/dobot_bringup_v3/srv/ClearError
/dobot_bringup_v3/srv/DisableRobot
/dobot_bringup_v3/srv/EnableRobot
/dobot_bringup_v3/srv/CP
/dobot_bringup_v3/srv/SetGripperPosition
/dobot_bringup_v3/srv/StartDrag
/dobot_bringup_v3/srv/StopDrag
```

The sequence:

```text
ClearError
DisableRobot
EnableRobot with load 2.0
CP r=100
Open gripper
StartDrag
Wait
StopDrag
```

The wrapper retries the whole initialization up to 3 times.

## 28. Robot Home Selection

After initialization, the script calls:

```bash
/dobot_bringup_v3/srv/GetAngle
```

It reads J1 and selects the nearest compass-style safe home angle:

```text
0
45
90
135
180
-135
-90
-45
```

Then it sends a `JointMovJ` command with:

```text
j2=30.0
j3=-130.0
j4=-100.0
j5=-90.0
j6=0.0
```

## 29. Modbus / Gripper Register Setup

The script runs:

```bash
ModbusClose index 0
ModbusCreate ip=127.0.0.1 port=60000 slave_id=9 is_rtu=1
SetHoldRegs addr=1000 count=3 val_tab="0,0,0"
SetHoldRegs addr=1000 count=3 val_tab="256,0,0"
```

This prepares the gripper/Modbus path used by the Dobot bringup service layer.

## 30. Headless MoveIt Launch

The script launches MoveIt without RViz:

```bash
ros2 launch dobot_moveit dobot_moveit.launch.py use_rviz:=false debug:=false __log_level:=fatal
```

This is the production/headless launch path.

## 31. Servo Action Launch

The script launches:

```bash
ros2 run servo_action action_move_server_reality __log_level:=fatal
```

This server bridges MoveIt planned trajectories into real Dobot `ServoJ` commands.

## 32. Perception Launch

The script launches:

```bash
ros2 run pickn_place aruco_perception __log_level:=info
```

and:

```bash
ros2 run pickn_place obstacle_generator __log_level:=fatal
```

## 33. Dobot Sync Probe

Before starting OMS, the script checks if the Dobot service layer is responsive using:

```bash
ros2 service call /dobot_bringup_v3/srv/Sync dobot_msgs_v3/srv/Sync "{}"
```

It tries 3 times with a timeout.

If all attempts fail, the inner stack exits with code:

```text
42
```

The outer startup loop treats this as a request to relaunch the full robot stack.

## 34. OMS Service Launch

After the Dobot Sync probe passes, the script runs:

```bash
cd src/oms_v1
python -m oms_v1.app --service
```

This starts the OMS robot service and connects to RabbitMQ.

---

# Part E - Startup Script Function Reference

## 35. Logging Helpers

### `log()`

Prints normal Robot 1 status messages with the `[ROBOT1]` prefix.

### `warn()`

Prints warning messages with the `[WARNING]` prefix.

### `error()`

Prints an error message and exits the script with failure.

### `info()`

Prints informational messages with the `[INFO]` prefix.

---

## 36. `reset_usb_device()`

Purpose:

Recover the Orbbec camera when it stops publishing, fails to initialize, or gets stuck at the USB level.

What it does:

1. Looks for an Orbbec USB device using vendor ID pattern `2bc5` from `lsusb`.
2. Extracts USB bus and device number.
3. Attempts a soft reset by toggling the USB device `authorized` flag.
4. Attempts a hard reset using `usbreset` or a Python `USBDEVFS_RESET` ioctl fallback.
5. Waits for the device to reappear.

Used by:

```text
restart_camera_and_perception()
```

Failure behavior:

If no Orbbec USB device is found, it logs a warning and returns failure, but the parent recovery function may continue.

---

## 37. `_dobot_init_once()`

Purpose:

Run one attempt of the Dobot initialization sequence.

It calls:

```text
ClearError
DisableRobot
EnableRobot
CP
SetGripperPosition
StartDrag
StopDrag
```

Failure behavior:

Each service call has a timeout. If any required service fails or times out, the function returns failure.

Used by:

```text
initialize_dobot()
```

---

## 38. `initialize_dobot()`

Purpose:

Retry Dobot initialization up to 3 times.

What it does:

1. Calls `_dobot_init_once()`.
2. If it fails, clears errors and disables the robot before retrying.
3. Returns success on the first good initialization.
4. Returns failure after all retries fail.

---

## 39. `restart_camera_and_perception()`

Purpose:

Restart Orbbec camera and ArUco perception if camera data stops flowing or the perception node dies.

What it does:

1. Prevents duplicate restarts using `RESTART_IN_PROGRESS`.
2. Kills existing `orbbec_camera` and `aruco_perception` processes.
3. Runs staged USB recovery depending on restart count.
4. Clears Orbbec and DDS shared memory locks.
5. Relaunches Orbbec camera.
6. Waits for camera initialization.
7. Checks for color/depth camera info topics.
8. Relaunches ArUco perception.
9. Enforces a cooldown before another restart is allowed.

Recovery levels:

```text
Attempts 1-2 -> soft USB reset
Attempt 3    -> reload uvcvideo kernel module
Attempt 4    -> rebind USB controller
Attempt 5    -> reboot system or force pod/container restart
```

---

## 40. `wait_for_robot_stack_ready()`

Purpose:

Wait until camera and ArUco perception are actually publishing data, not just advertising topics.

It checks:

```text
/camera/color/camera_info
/camera/depth/camera_info
/camera/color/image_raw
/camera/depth/image_raw
```

It uses `ros2 topic echo --once` with timeouts to verify messages are flowing.

This prevents false readiness where topics exist but no camera frames or intrinsics are being published.

---

## 41. `monitor_perception_errors()`

Purpose:

Continuously monitor camera/perception health after startup.

It watches for:

```text
No color camera info received
Waiting for depth camera intrinsics
```

It also checks whether the `orbbec_camera` or `aruco_perception` processes died.

If repeated warnings are detected, it calls:

```text
restart_camera_and_perception()
```

Warning thresholds:

```text
Depth intrinsics warning -> restart after 4 repeated warnings within 2 minutes
Color camera info warning -> restart after 3 repeated warnings within 1 minute
```

---

## 42. `check_workspace()`

Purpose:

Verify the robot workspace exists before startup.

It requires:

```text
$WORKSPACE_DIR
$WORKSPACE_DIR/setup_robot_env.sh
```

If either is missing, the script exits and asks to run the dependency/install process first.

---

## 43. `cleanup_rabbitmq_queues()`

Purpose:

Delete stale RabbitMQ queues from prior robot sessions.

Queues cleaned:

```text
robot_container_1_responses
robot_container_1_requests
robot_container_2_responses
robot_container_2_requests
```

It also kills lingering `oms_v1` Python processes to prevent old clients from reconnecting with stale queue settings.

---

## 44. `cleanup_ros2_environment()`

Purpose:

Reset stale ROS 2 runtime state before starting the robot.

It does:

```text
pkill ros2 processes
pkill _ros2_daemon
ros2 daemon stop
remove /tmp/.ros*
clear ~/.ros/log/*
```

This helps avoid stale DDS/daemon issues during repeated deployments.

---

## 45. `start_docker_services()`

Purpose:

Start the Docker-side services required by the robot.

It checks:

```text
Docker installed
Docker daemon running
docker-compose.arms.yml exists
```

Then it:

1. Enables X11 forwarding with `xhost +local:root`.
2. Stops existing BARNS Docker services.
3. Runs `docker compose -f docker-compose.arms.yml up -d --build`.
4. Waits for RabbitMQ to pass `rabbitmq-diagnostics ping`.
5. Prints dashboard and RabbitMQ management URLs.

Expected RabbitMQ container name:

```text
barns-rabbitmq
```

Expected routine service container name:

```text
barns-routine
```

---

## 46. `wait_for_services()`

Purpose:

Wait for Docker-side services to become ready before starting the robot.

It checks whether the routine service can connect to RabbitMQ on port `5672`.

If it times out, it warns and continues.

---

## 47. `start_robot()`

Purpose:

Start the complete Robot 1 ROS stack and OMS service.

It does:

1. Sources `setup_robot_env.sh`.
2. Exports robot-specific environment variables.
3. Cleans Orbbec device lock.
4. Starts an inner launch shell.
5. Launches Dobot bringup.
6. Launches Orbbec camera.
7. Checks ROS 2 connectivity.
8. Initializes Dobot.
9. Moves robot to safe home based on current J1.
10. Sets up Modbus/gripper registers.
11. Launches MoveIt headless.
12. Launches servo action server.
13. Launches ArUco perception.
14. Launches obstacle generator.
15. Probes Dobot Sync.
16. Launches OMS service.
17. Starts camera/perception monitoring.
18. Relaunches the full stack up to 3 times if Sync probe fails.

Full stack relaunch trigger:

```text
Inner shell exits with code 42
```

Maximum relaunch attempts:

```text
3
```

---

## 48. `cleanup()`

Purpose:

Stop all Robot 1 processes during shutdown.

It kills:

```text
dobot_bringup_v3
orbbec_camera
dobot_moveit
servo_action
pickn_place
oms_v1.app
ros2
```

---

## 49. `main()`

Purpose:

Top-level startup path for:

```bash
./robot1-startup.sh start
```

It runs:

```text
check_workspace
start_docker_services
wait_for_services
cleanup_rabbitmq_queues
cleanup_ros2_environment
start_robot
```

---

# Part F - Environment Setup and Sourcing

## 50. Local Host / Manual ROS Sourcing

For manual ROS debugging outside Docker:

```bash
cd services/robot_container/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DOBOT_TYPE=nova5
export IP_address=<robot-ip>
```

## 51. Docker Image Sourcing

Inside the Robot 1 Docker image:

```bash
source /opt/barns-robot/ros_ws/setup_robot_env.sh
```

This script sources ROS Humble and the built workspace, and exports runtime defaults for headless operation.

## 52. Dev Docker Compose Sourcing

In the dev Docker Compose setup, the workspace is mounted at:

```text
/root/ros_ws
```

If entering the container manually:

```bash
docker exec -it oms_robot_1 bash
cd /root/ros_ws
source install/setup.bash
```

---

# Part G - Device and Hardware Checks

## 53. Check Robot Network

```bash
ping <robot-ip>
```

Robot 1 examples:

```bash
ping 192.168.100.249
ping 192.168.200.249
```

Use whichever network matches the deployment.

## 54. Check Orbbec USB Device

```bash
lsusb | grep -i 2bc5
```

## 55. Check Video Devices

```bash
ls -l /dev/video*
ls -l /dev/media*
```

## 56. Check Serial Device

Robot 1 usually maps:

```text
/dev/ttyUSB0
```

Robot 2 usually maps:

```text
/dev/ttyUSB1
```

Check:

```bash
ls -l /dev/ttyUSB*
```

If device numbers changed after reboot, update the compose device mappings.

---

# Part H - Runtime Verification

## 57. Check Containers

```bash
docker ps
```

## 58. Check Robot 1 Logs

```bash
docker logs -f oms_robot_1
```

or:

```bash
docker compose -f docker-compose.arms.yml logs -f robot1
```

## 59. Check RabbitMQ

```bash
docker exec barns-rabbitmq rabbitmq-diagnostics ping
```

List queues:

```bash
docker exec barns-rabbitmq rabbitmqctl list_queues name messages consumers
```

## 60. Check ROS 2 Nodes

Inside the robot container or robot host environment:

```bash
ros2 node list
```

Expected major nodes include:

```text
dobot_bringup_v3
camera / Orbbec camera node
aruco_perception_node
move_group / MoveIt nodes
action_move_server
obstacle generator
```

Exact node names can vary by launch file and namespace.

## 61. Check ROS 2 Topics

```bash
ros2 topic list
```

Important topics:

```text
/joint_states_robot
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/display_planned_path
/servo_controller_status
```

Verify camera messages are actually flowing:

```bash
timeout 5 ros2 topic echo --once /camera/color/camera_info
timeout 5 ros2 topic echo --once /camera/depth/camera_info
timeout 5 ros2 topic echo --once --no-arr /camera/color/image_raw
timeout 5 ros2 topic echo --once --no-arr /camera/depth/image_raw
```

## 62. Check Dobot Services

```bash
ros2 service list | grep dobot_bringup_v3
```

Probe Dobot Sync:

```bash
timeout 10 ros2 service call /dobot_bringup_v3/srv/Sync dobot_msgs_v3/srv/Sync "{}"
```

Check gripper:

```bash
ros2 service call /dobot_bringup_v3/srv/SetGripperPosition dobot_msgs_v3/srv/SetGripperPosition "{position: 0, speed: 255, force: 255}"
```

---

# Part I - Troubleshooting

## 63. Docker Build Fails Immediately

Check:

```bash
docker info
```

If Docker daemon is not running:

```bash
sudo systemctl start docker
```

If permissions fail, add the user to the Docker group or run with `sudo`.

## 64. `Dockerfile.robot1 not found`

The build script expects `Dockerfile.robot1` in the same directory as the script.

Fix the file location or run the script from the correct deployment layout.

## 65. Build Is Too Slow

Use cached builds whenever possible:

```bash
./build-robot1.sh
```

Avoid `--no-cache` unless necessary.

On high-RAM machines:

```bash
ROBOT_COLCON_PARALLEL=4 ROBOT_MAKE_JOBS=4 ./build-robot1.sh
```

If the machine runs out of memory, reduce both values back to `2`.

## 66. RabbitMQ Fails to Start

Check:

```bash
docker ps
docker logs barns-rabbitmq
```

Probe RabbitMQ:

```bash
docker exec barns-rabbitmq rabbitmq-diagnostics ping
```

If queues are stale, the Robot 1 script deletes known robot request/response queues during startup.

## 67. Robot 1 Cannot Connect to RabbitMQ

Check `DOCKER_HOST_IP` and `RABBITMQ_URL`.

For Robot 1 startup script:

```bash
RABBITMQ_URL=amqp://admin:admin123@${DOCKER_HOST_IP}:5672/
```

Make sure the robot host can reach RabbitMQ:

```bash
nc -vz <docker-host-ip> 5672
```

## 68. ROS 2 Nodes Cannot Discover Each Other

Check:

```bash
ROS_DOMAIN_ID
network_mode: host
```

Robot 1 and Robot 2 should use different domain IDs unless intentionally sharing a ROS graph.

Current defaults:

```text
Robot 1 -> ROS_DOMAIN_ID=0
Robot 2 -> ROS_DOMAIN_ID=1
```

## 69. Camera Topics Exist but No Data Arrives

A topic existing does not mean data is flowing.

Check actual messages:

```bash
timeout 5 ros2 topic echo --once /camera/color/camera_info
timeout 5 ros2 topic echo --once /camera/depth/camera_info
```

If these fail, restart camera/perception or use the Robot 1 startup script recovery.

## 70. ArUco Perception Logs `No color camera info received`

This usually means the Orbbec camera node exists but color camera info is not being published.

The Robot 1 startup monitor watches for repeated warnings and restarts camera/perception automatically.

Manual recovery:

```bash
pkill -f orbbec_camera
pkill -f aruco_perception
rm -f /dev/shm/orbbec_device_lock
rm -f /dev/shm/fastrtps_port*
rm -f /dev/shm/sem.fastrtps_port*
ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
ros2 run pickn_place aruco_perception
```

## 71. Orbbec Camera Is Stuck After Multiple Restarts

Check USB:

```bash
lsusb | grep -i 2bc5
```

Try replugging the camera or rebooting the host.

The Robot 1 startup script escalates recovery through:

```text
USB soft reset
uvcvideo reload
USB controller rebind
system reboot / pod restart
```

## 72. Dobot Sync Probe Fails

The startup script probes:

```bash
/dobot_bringup_v3/srv/Sync
```

If it fails 3 times, the inner stack exits with code `42`, and the outer script relaunches the full stack.

Manual checks:

```bash
ros2 service list | grep Sync
timeout 10 ros2 service call /dobot_bringup_v3/srv/Sync dobot_msgs_v3/srv/Sync "{}"
```

If services exist but calls hang, restart Dobot bringup or the full stack.

## 73. Wrong Robot IP

Check whether the deployment uses the `192.168.100.x` network or the `192.168.200.x` network.

Then set the correct variable:

```bash
IP_ADDRESS=<robot-ip> ./robot1-startup.sh start
```

or in Compose:

```yaml
- IP_address=<robot-ip>
```

## 74. GUI Does Not Open From Container

Check:

```bash
echo $DISPLAY
xhost +local:root
ls /tmp/.X11-unix
```

The Compose file must mount:

```yaml
/tmp/.X11-unix:/tmp/.X11-unix:rw
```

and set:

```yaml
DISPLAY=${DISPLAY}
QT_X11_NO_MITSHM=1
```

## 75. Device Mapping Is Wrong

If the camera or serial device path changed, update the Compose file.

Check host devices:

```bash
ls -l /dev/video*
ls -l /dev/media*
ls -l /dev/ttyUSB*
```

Robot 1 and Robot 2 must not map the same physical camera/serial devices unless that is intentional.

---

# Part J - Operational Notes

## 76. Manual Local Launch vs Docker Deployment

Use manual ROS launch for development and debugging.

Use Docker deployment for OMS/RabbitMQ/service-style robot operation.

Manual launch usually uses separate terminals.

Docker deployment uses one startup command or Compose service and starts everything in sequence.

## 77. Image Name Difference

There are two image naming styles in the deployment files:

```text
barns-robot1:latest
oms_robot_1:dev
oms_robot_2:dev
```

Meaning:

```text
barns-robot1:latest -> default output of Robot 1 build script
oms_robot_1:dev     -> Robot 1 image name used by Compose
oms_robot_2:dev     -> Robot 2 image name used by Compose
```

To align them, either:

```bash
IMAGE_NAME=oms_robot_1 IMAGE_TAG=dev ./build-robot1.sh
```

or update the Compose file to use the image name produced by the build script.

## 78. Security Note

The deployment uses development credentials in the examples:

```text
RabbitMQ: admin / admin123
```

For production outside a controlled network, replace these credentials and update all `RABBITMQ_URL` values.

---

# TLDR

## Clone

```bash
git clone https://github.com/QSS-AI-Robotics/BARNS.git
cd BARNS
```

## Build Robot 1 Image

```bash
./build-robot1.sh
```

Full rebuild:

```bash
./build-robot1.sh --no-cache
```

Build with Compose-style name:

```bash
IMAGE_NAME=oms_robot_1 IMAGE_TAG=dev ./build-robot1.sh
```

## Start With Docker Compose

```bash
docker compose -f docker-compose.arms.yml up -d --build
```

Logs:

```bash
docker compose -f docker-compose.arms.yml logs -f robot1
```

Shell:

```bash
docker exec -it oms_robot_1 bash
```

Stop:

```bash
docker compose -f docker-compose.arms.yml down
```

## Start Robot 1 Production Script

Full startup:

```bash
./robot1-startup.sh start
```

Docker only:

```bash
./robot1-startup.sh docker-only
```

Robot only:

```bash
./robot1-startup.sh robot-only
```

Stop:

```bash
./robot1-startup.sh stop
```

## Key Variables

```bash
DOBOT_TYPE=nova5
IP_ADDRESS=<robot-ip-for-startup-script>
IP_address=<robot-ip-for-dobot-bringup>
ROS_DOMAIN_ID=0
ROBOT_ID=1
RABBITMQ_URL=amqp://admin:admin123@<host>:5672/
CAMERA_SERIAL_NUMBER=<orbbec-serial>
```

## Key Checks

```bash
docker ps
ros2 node list
ros2 topic list
ros2 service list | grep dobot
lsusb | grep -i 2bc5
ping <robot-ip>
```

## Most Important Notes

- Use `network_mode: host` for ROS 2 discovery.
- Use `privileged: true` and correct `/dev/video*`, `/dev/media*`, and `/dev/ttyUSB*` mappings.
- Robot 1 and Robot 2 should use different `ROS_DOMAIN_ID` values.
- Confirm whether the robot is on `192.168.100.x` or `192.168.200.x` before deploying.
- The Robot 1 startup script automatically handles RabbitMQ cleanup, ROS cleanup, Dobot initialization, camera monitoring, camera recovery, and full stack relaunch on Dobot Sync failure.
