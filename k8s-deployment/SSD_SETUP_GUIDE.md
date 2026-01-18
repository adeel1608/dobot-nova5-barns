# BARNS Kubernetes Setup with SSD Storage

This guide explains how to set up your 2-node Jetson Kubernetes cluster with all data stored on the 2TB SSD mounted at `/mnt/ssd`.

## Overview

Your setup consists of:
- **1 Master Node (Jetson)**: Runs Kubernetes control plane
- **1 Worker Node (Jetson)**: Runs application workloads
- **Storage**: All Kubernetes and application data stored on `/mnt/ssd`

## Prerequisites

### On Both Nodes

1. **Mount your 2TB SSD at `/mnt/ssd`**:
   ```bash
   # Check if SSD is mounted
   df -h /mnt/ssd
   
   # If not mounted, mount it (adjust device as needed)
   sudo mkdir -p /mnt/ssd
   sudo mount /dev/sda1 /mnt/ssd
   
   # Make it permanent in /etc/fstab
   echo "/dev/sda1  /mnt/ssd  ext4  defaults  0  2" | sudo tee -a /etc/fstab
   ```

2. **Verify network connectivity**:
   ```bash
   ping -c 3 8.8.8.8
   ```

3. **Ensure both nodes can reach each other**:
   ```bash
   # From master
   ping -c 3 <worker-ip>
   
   # From worker
   ping -c 3 <master-ip>
   ```

## Step 1: Clean Previous Setup (If Needed)

If you have a previous K8s installation that crashed, clean it up first:

```bash
cd ~/Barns/k8s-deployment/scripts
sudo ./cleanup.sh
```

This will:
- Reset kubeadm
- Clean Kubernetes configuration
- Optionally clean SSD Kubernetes data
- Preserve your application data

## Step 2: Set Up Master Node

On the Jetson you want as the **master**:

```bash
cd ~/Barns/k8s-deployment/scripts
sudo ./setup-master.sh
```

This script will:
1. Verify SSD is mounted at `/mnt/ssd`
2. Install and configure all prerequisites
3. Configure containerd to use `/mnt/ssd/var/lib/containerd`
4. Configure kubelet to use `/mnt/ssd/var/lib/kubelet`
5. Configure etcd to use `/mnt/ssd/var/lib/etcd`
6. Initialize the Kubernetes cluster
7. Install Flannel CNI
8. Generate worker join command

**Important**: At the end, you'll see a join command. Copy it - you'll need it for the worker node.

Example output:
```
Join Command:
---
sudo kubeadm join 10.0.0.78:6443 --token abc123... --discovery-token-ca-cert-hash sha256:xyz...
---
```

## Step 3: Set Up Worker Node

On the Jetson you want as the **worker**:

```bash
cd ~/Barns/k8s-deployment/scripts
sudo ./setup-worker.sh
```

This script will:
1. Verify SSD is mounted at `/mnt/ssd`
2. Install Docker with storage at `/mnt/ssd/var/lib/docker`
3. Configure containerd to use `/mnt/ssd/var/lib/containerd`
4. Configure kubelet to use `/mnt/ssd/var/lib/kubelet`
5. Create application storage directories on SSD
6. Install Kubernetes components
7. Prompt for cluster join (if join command exists)

When prompted, paste the join command from the master node.

## Step 4: Verify Cluster

On the **master node**, verify the cluster is working:

```bash
# Check nodes
kubectl get nodes

# Expected output:
# NAME            STATUS   ROLES           AGE   VERSION
# master-node     Ready    control-plane   5m    v1.30.x
# worker-node     Ready    <none>          2m    v1.30.x

# Check all system pods
kubectl get pods -A

# All pods should be Running
```

## Step 5: Verify SSD Usage

On both nodes, verify that Kubernetes is using the SSD:

```bash
# Check disk usage on SSD
du -sh /mnt/ssd/var/lib/*

# Example output:
# 2.1G    /mnt/ssd/var/lib/containerd
# 156M    /mnt/ssd/var/lib/docker
# 89M     /mnt/ssd/var/lib/etcd
# 45M     /mnt/ssd/var/lib/kubelet

# Check that internal storage is NOT being used
du -sh /var/lib/kubelet 2>/dev/null || echo "Not using internal storage ✓"
```

## Storage Layout on SSD

Your SSD is organized as follows:

```
/mnt/ssd/
├── var/lib/
│   ├── kubelet/          # Kubernetes kubelet data
│   ├── containerd/       # Container images and layers
│   ├── docker/           # Docker data (worker node)
│   └── etcd/             # Kubernetes cluster state (master only)
├── barns-data/           # BARNS application data
│   ├── postgres/
│   ├── rabbitmq/
│   ├── influxdb/
│   ├── redis/
│   └── cup_models/
├── barns-config/         # Configuration files
└── k8s-data/             # Additional K8s data
```

## What Gets Stored on the SSD

### Master Node
- **etcd database**: Entire Kubernetes cluster state
- **kubelet data**: Pod manifests, certificates, logs
- **containerd**: Container images for system pods
- **Application data**: BARNS services data

### Worker Node
- **Docker data**: All Docker images and containers
- **kubelet data**: Pod manifests, certificates, logs
- **containerd**: Container images for application pods
- **Application data**: BARNS PostgreSQL, RabbitMQ, InfluxDB, etc.

## Configuration File

You can customize the setup by editing `k8s-deployment/config/cluster-config.yaml`:

```yaml
# SSD mount point
ssd_mount: /mnt/ssd

# Network configuration
pod_cidr: 10.244.0.0/16
service_cidr: 10.96.0.0/12

# CNI plugin
cni: flannel

# Cluster name
name: barns-cluster

# Storage paths (relative to ssd_mount)
base_path: /mnt/ssd/barns-data
```

## Troubleshooting

### Issue: SSD Not Mounted

```bash
# Check if SSD is detected
lsblk

# Mount manually
sudo mount /dev/sda1 /mnt/ssd

# Add to fstab for auto-mount on boot
sudo blkid /dev/sda1  # Get UUID
echo "UUID=<uuid>  /mnt/ssd  ext4  defaults  0  2" | sudo tee -a /etc/fstab
```

### Issue: Node Not Ready

```bash
# Check kubelet logs
sudo journalctl -u kubelet -f

# Check node status
kubectl describe node <node-name>

# Restart kubelet
sudo systemctl restart kubelet
```

### Issue: Pods Not Starting

```bash
# Check pod status
kubectl get pods -A

# Check specific pod logs
kubectl logs <pod-name> -n <namespace>

# Describe pod for events
kubectl describe pod <pod-name> -n <namespace>

# Check containerd
sudo systemctl status containerd
```

### Issue: Out of Space Despite SSD

```bash
# Verify kubelet is using SSD
ps aux | grep kubelet | grep root-dir

# Should show: --root-dir=/mnt/ssd/var/lib/kubelet

# Verify containerd is using SSD
sudo cat /etc/containerd/config.toml | grep "root ="

# Should show: root = "/mnt/ssd/var/lib/containerd"

# If not, run cleanup and setup again
sudo ./cleanup.sh
sudo ./setup-master.sh  # or setup-worker.sh
```

### Issue: Network Problems After IP Change

On the master node:
```bash
# Update network configuration
sudo /usr/local/bin/k8s-update-network.sh
```

On worker nodes, the kubelet will automatically detect and use the new IP on restart.

## Next Steps

After your cluster is running:

1. **Deploy BARNS services**:
   ```bash
   cd ~/Barns/k8s-deployment/scripts
   ./deploy-k8s.sh
   ```

2. **Build and deploy robot images**:
   ```bash
   cd ~/Barns
   ./build-images-arm64.sh
   ```

3. **Access the dashboard**:
   ```
   http://<worker-ip>:30003
   ```

## Maintenance

### Regular Cleanup

```bash
# Clean up unused images (frees SSD space)
sudo crictl rmi --prune

# Clean up Docker images
docker image prune -a
```

### Check Disk Usage

```bash
# Overall SSD usage
df -h /mnt/ssd

# Detailed breakdown
du -h --max-depth=2 /mnt/ssd | sort -h
```

### Backup Important Data

```bash
# Backup application data
sudo tar -czf barns-data-backup-$(date +%Y%m%d).tar.gz /mnt/ssd/barns-data

# Backup Kubernetes configuration (master only)
sudo cp -r /etc/kubernetes ~/kubernetes-backup-$(date +%Y%m%d)
```

## Scripts Reference

- `setup-master.sh`: Set up master node with SSD storage
- `setup-worker.sh`: Set up worker node with SSD storage
- `cleanup.sh`: Clean up Kubernetes installation
- `deploy-k8s.sh`: Deploy BARNS applications
- `build-images.sh`: Build Docker images for BARNS

## Support

For issues or questions:
1. Check the logs: `sudo journalctl -u kubelet -f`
2. Check pod status: `kubectl get pods -A`
3. Review this guide
4. Check the deployment logs in `k8s-deployment/deployment.log`
