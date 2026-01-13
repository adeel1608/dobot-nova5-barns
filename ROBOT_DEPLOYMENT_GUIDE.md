# BARNS Robot Services - Kubernetes Deployment Guide

## Overview

This guide covers the deployment of Robot1 and Robot2 as containerized Kubernetes pods with node pinning and hardware access.

## Architecture

```
Master Node (qss-jetson) - ARM64
â”œâ”€â”€ Robot1 Pod
â”‚   â”œâ”€â”€ Robot Arm: 192.168.200.249
â”‚   â”œâ”€â”€ Camera: CP1Z842000YW  
â”‚   â”œâ”€â”€ ROS_DOMAIN_ID: 0
â”‚   â””â”€â”€ Connects to: rabbitmq.barns.svc.cluster.local

Worker Node (ubuntu) - ARM64
â”œâ”€â”€ Robot2 Pod
â”‚   â”œâ”€â”€ Robot Arm: 192.168.200.248
â”‚   â”œâ”€â”€ Camera: CP1Z842000F6
â”‚   â”œâ”€â”€ ROS_DOMAIN_ID: 1
â”‚   â””â”€â”€ Connects to: rabbitmq.barns.svc.cluster.local
â”œâ”€â”€ All other BARNS services
```

## Prerequisites

### One-Time Host Setup (USB Rules)

Run `install-robot-dependencies.sh` ONLY for USB udev rules:

```bash
# On both nodes
cd ~/Barns/BARNS
chmod +x install-robot-dependencies.sh
./install-robot-dependencies.sh
```

This installs USB rules for Orbbec cameras. The ROS workspace is now built inside the container.

## Deployment Workflow

### Step 1: Sync Code to Nodes

From Windows machine:

```powershell
cd D:\D-Drive\BARNS
git pull

# Sync to master
rsync -avz --exclude node_modules --exclude .git ./ qss@qss-jetson:~/Barns/BARNS/

# Sync to worker
rsync -avz --exclude node_modules --exclude .git ./ qss@ubuntu:~/Barns/BARNS/
```

Or from master node:

```bash
cd ~/Barns/BARNS
git pull
```

### Step 2: Build Robot Images

**On Master Node (qss-jetson) - Build Robot1:**

```bash
cd ~/Barns/BARNS
chmod +x build-robot-images.sh
./build-robot-images.sh robot1
```

Expected build time: 15-20 minutes (first build with ROS workspace compilation)

**On Worker Node (ubuntu) - Build Robot2:**

```bash
cd ~/Barns/BARNS
chmod +x build-robot-images.sh
sudo ./build-robot-images.sh robot2
```

Expected build time: 15-20 minutes

### Step 3: Deploy to Kubernetes

From master node (where kubectl is configured):

```bash
cd ~/Barns/BARNS

# Deploy robot1
kubectl apply -f k8s/services/robot1-deployment.yaml

# Deploy robot2
kubectl apply -f k8s/services/robot2-deployment.yaml
```

### Step 4: Verify Deployment

```bash
# Check pod status and node placement
kubectl get pods -n barns -o wide | grep robot

# Expected output:
# robot1-xxx   1/1   Running   qss-jetson   ...
# robot2-xxx   1/1   Running   ubuntu       ...

# Check logs
kubectl logs -f -n barns -l app=robot1
kubectl logs -f -n barns -l app=robot2

# Verify USB devices
kubectl exec -n barns deployment/robot1 -- lsusb | grep -i orbbec
kubectl exec -n barns deployment/robot2 -- lsusb | grep -i orbbec

# Verify RabbitMQ connection
kubectl exec -n barns deployment/robot1 -- env | grep RABBITMQ
```

## Key Features

### Node Pinning
- **Strict Enforcement**: Pods will ONLY run on their assigned nodes
- Robot1: Always on `qss-jetson` (master)
- Robot2: Always on `ubuntu` (worker)
- Prevents pod migration even during node failures

### Hardware Access
- **hostNetwork: true**: Direct access to host network for:
  - Robot arm IPs (192.168.200.249, 192.168.200.248)
  - ROS2 DDS communication
- **privileged: true**: Direct USB device access for cameras
- **Volume Mounts**: `/dev`, `/sys`, `/run/udev` for hardware control

### Tolerations
- Robot1 has tolerations for master node control-plane taints
- Allows scheduling on master node despite typical restrictions

## Image Details

### What's Inside
- ROS2 Humble fully configured
- Complete ROS workspace built (84 packages, orbbec_camera skipped)
- All Python dependencies (OpenCV, NumPy, SciPy, aio-pika, pika)
- Robot startup scripts integrated
- Self-contained, no host dependencies except USB

### Image Size
- ~3-4GB per robot image
- Build time: 15-20 minutes (ROS workspace compilation)
- Cached layers speed up subsequent builds

## Management Commands

### Restart Robots
```bash
kubectl rollout restart deployment/robot1 -n barns
kubectl rollout restart deployment/robot2 -n barns
```

### Stop Robots
```bash
kubectl scale deployment robot1 robot2 --replicas=0 -n barns
```

### Start Robots
```bash
kubectl scale deployment robot1 robot2 --replicas=1 -n barns
```

### View Logs
```bash
# Follow logs
kubectl logs -f -n barns -l app=robot1
kubectl logs -f -n barns -l app=robot2

# Last 100 lines
kubectl logs --tail=100 -n barns -l app=robot1
```

### Shell Access
```bash
kubectl exec -it -n barns deployment/robot1 -- /bin/bash
kubectl exec -it -n barns deployment/robot2 -- /bin/bash
```

## Troubleshooting

### Pod Not Starting

**Check pod status:**
```bash
kubectl describe pod -n barns <pod-name>
```

**Common issues:**
- **Pending**: Node selector not matching â†’ Check `kubectl get nodes` for actual hostnames
- **ImagePullBackOff**: Image not in containerd â†’ Rebuild and import on correct node
- **CrashLoopBackOff**: Container failing â†’ Check logs with `kubectl logs`

### Wrong Node Placement

If robot appears on wrong node:

1. Check node hostnames match deployment YAML:
```bash
kubectl get nodes -o custom-columns=NAME:.metadata.name
```

2. Update deployment YAML if needed:
```yaml
nodeSelector:
  kubernetes.io/hostname: <actual-hostname>
```

3. Redeploy:
```bash
kubectl delete pod -n barns <pod-name>
```

### USB Device Not Detected

**Check on host first:**
```bash
lsusb | grep -i orbbec
```

**If device present but not in pod:**
1. Verify privileged mode enabled
2. Check volume mounts for `/dev`
3. Restart pod

**If device not on host:**
1. Check USB cable connection
2. Check udev rules: `ls /etc/udev/rules.d/ | grep orbbec`
3. Reload udev: `sudo udevadm control --reload && sudo udevadm trigger`

### Robot Arm Not Responding

**Test connectivity:**
```bash
kubectl exec -n barns deployment/robot1 -- ping 192.168.200.249
kubectl exec -n barns deployment/robot2 -- ping 192.168.200.248
```

**Verify hostNetwork:**
```bash
kubectl get pod <pod-name> -n barns -o yaml | grep hostNetwork
# Should show: hostNetwork: true
```

### ROS2 Communication Issues

**Check ROS_DOMAIN_ID:**
```bash
kubectl exec -n barns deployment/robot1 -- env | grep ROS_DOMAIN_ID  # Should be 0
kubectl exec -n barns deployment/robot2 -- env | grep ROS_DOMAIN_ID  # Should be 1
```

**Test ROS2 topics:**
```bash
kubectl exec -n barns deployment/robot1 -- bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"
```

### RabbitMQ Connection Failure

**Check RabbitMQ service:**
```bash
kubectl get svc -n barns rabbitmq
kubectl get pods -n barns -l app=rabbitmq
```

**Test connection from pod:**
```bash
kubectl exec -n barns deployment/robot1 -- \
  curl -u admin:admin123 http://rabbitmq.barns.svc.cluster.local:15672/api/overview
```

## Rebuild After Code Changes

When you update robot startup scripts or ROS workspace:

```bash
# 1. Sync code
cd ~/Barns/BARNS
git pull

# 2. On respective node, rebuild
./build-robot-images.sh robot1  # or robot2

# 3. Restart deployment
kubectl rollout restart deployment/robot1 -n barns
```

## Performance

- **Startup time**: 60-120 seconds (ROS2 initialization + robot arm setup)
- **Memory usage**: 1-2GB typical, 3-4GB during CV operations
- **CPU usage**: 0.5-1.5 cores per robot
- **No performance degradation** vs bare metal (hostNetwork + privileged mode)

## Important Notes

1. **install-robot-dependencies.sh** is now ONLY for USB udev rules
2. Images are **local-only** (`imagePullPolicy: Never`)
3. No Docker registry needed
4. ROS workspace built inside container for portability
5. Each node builds its own robot image
6. Tolerations allow robot1 to run on master despite control-plane taints

## Files Structure

```
BARNS/
â”œâ”€â”€ services/robot/
â”‚   â”œâ”€â”€ Dockerfile.robot1         # Robot1 container (for master)
â”‚   â””â”€â”€ Dockerfile.robot2         # Robot2 container (for worker)
â”œâ”€â”€ k8s/services/
â”‚   â”œâ”€â”€ robot1-deployment.yaml    # Robot1 K8s manifest
â”‚   â””â”€â”€ robot2-deployment.yaml    # Robot2 K8s manifest
â”œâ”€â”€ build-robot-images.sh         # Build script for individual robots
â”œâ”€â”€ robot1-startup.sh             # Robot1 startup logic (copied into image)
â”œâ”€â”€ robot2-startup.sh             # Robot2 startup logic (copied into image)
â”œâ”€â”€ install-robot-dependencies.sh # One-time USB setup only
â””â”€â”€ ROBOT_DEPLOYMENT_GUIDE.md     # This file
```

## Quick Reference

| Command | Purpose |
|---------|---------|
| `./build-robot-images.sh robot1` | Build robot1 image (on master) |
| `./build-robot-images.sh robot2` | Build robot2 image (on worker) |
| `kubectl apply -f k8s/services/robot1-deployment.yaml` | Deploy robot1 |
| `kubectl apply -f k8s/services/robot2-deployment.yaml` | Deploy robot2 |
| `kubectl get pods -n barns -o wide \| grep robot` | Check robot pod status |
| `kubectl logs -f -n barns -l app=robot1` | View robot1 logs |
| `kubectl rollout restart deployment/robot1 -n barns` | Restart robot1 |
| `kubectl exec -it -n barns deployment/robot1 -- bash` | Shell into robot1 |

## Support

For issues or questions, check:
1. Pod logs: `kubectl logs -f -n barns -l app=robot<N>`
2. Pod events: `kubectl describe pod -n barns <pod-name>`
3. Node status: `kubectl get nodes -o wide`
4. USB devices: `kubectl exec -n barns deployment/robot<N> -- lsusb`
