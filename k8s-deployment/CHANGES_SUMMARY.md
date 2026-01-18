# Kubernetes Setup Scripts - SSD Storage Changes

## Summary

Updated the BARNS Kubernetes deployment scripts to use the 2TB SSD mounted at `/mnt/ssd` for all storage instead of internal storage.

## Files Modified

### 1. `k8s-deployment/scripts/setup-master.sh`

**Changes:**
- Added SSD mount verification at startup
- Created SSD storage directories before installation:
  - `/mnt/ssd/var/lib/kubelet`
  - `/mnt/ssd/var/lib/containerd`
  - `/mnt/ssd/var/lib/etcd`
  - `/mnt/ssd/k8s-data`
  - `/mnt/ssd/barns-data`
- Updated containerd configuration to use `/mnt/ssd/var/lib/containerd` for storage
- Added kubelet systemd override to use `/mnt/ssd/var/lib/kubelet` as root directory
- Updated kubeadm configuration to use `/mnt/ssd/var/lib/etcd` for etcd data
- Added SSD paths to completion messages and saved configuration file

**Key Configuration:**

```bash
# Kubelet
Environment="KUBELET_EXTRA_ARGS=--root-dir=/mnt/ssd/var/lib/kubelet"

# Containerd
root = "/mnt/ssd/var/lib/containerd"

# etcd
dataDir: /mnt/ssd/var/lib/etcd
```

### 2. `k8s-deployment/scripts/setup-worker.sh`

**Changes:**
- Added SSD mount verification at startup
- Updated storage base path to use SSD: `$SSD_MOUNT/barns-data`
- Created comprehensive SSD storage directories:
  - Kubernetes: `/mnt/ssd/var/lib/{kubelet,containerd,docker}`
  - Application: `/mnt/ssd/barns-data/{postgres,rabbitmq,influxdb,redis,cup_models}`
  - Config: `/mnt/ssd/barns-config`
- Updated Docker daemon.json to use `/mnt/ssd/var/lib/docker` for data
- Updated containerd configuration to use `/mnt/ssd/var/lib/containerd` for storage
- Added kubelet systemd override to use `/mnt/ssd/var/lib/kubelet` as root directory
- Added SSD paths to completion messages and saved configuration file

**Key Configuration:**

```bash
# Docker
"data-root": "/mnt/ssd/var/lib/docker"

# Kubelet
Environment="KUBELET_EXTRA_ARGS=--root-dir=/mnt/ssd/var/lib/kubelet"

# Containerd
root = "/mnt/ssd/var/lib/containerd"
```

### 3. `k8s-deployment/scripts/cleanup.sh` (New File)

**Purpose:** Comprehensive cleanup script for Kubernetes installation

**Features:**
- Stops all Kubernetes services
- Resets kubeadm configuration
- Cleans up network settings (iptables)
- Optionally removes Kubernetes data from SSD
- Preserves application data (`/mnt/ssd/barns-data`)
- Removes systemd overrides
- Restarts container runtime

**Usage:**
```bash
sudo ./cleanup.sh
```

### 4. `k8s-deployment/SSD_SETUP_GUIDE.md` (New File)

**Purpose:** Comprehensive guide for setting up Kubernetes with SSD storage

**Contents:**
- Prerequisites and SSD mounting instructions
- Step-by-step setup for both master and worker nodes
- Storage layout explanation
- Troubleshooting common issues
- Maintenance and backup procedures
- Scripts reference

## What Changed Technically

### Storage Paths

**Before:**
- Kubelet: `/var/lib/kubelet` (internal storage)
- Containerd: `/var/lib/containerd` (internal storage)
- Docker: `/var/lib/docker` (internal storage)
- etcd: `/var/lib/etcd` (internal storage)

**After:**
- Kubelet: `/mnt/ssd/var/lib/kubelet` (SSD)
- Containerd: `/mnt/ssd/var/lib/containerd` (SSD)
- Docker: `/mnt/ssd/var/lib/docker` (SSD)
- etcd: `/mnt/ssd/var/lib/etcd` (SSD)

### Configuration Methods

1. **Kubelet**: Systemd drop-in file at `/etc/systemd/system/kubelet.service.d/20-ssd-root.conf`
2. **Containerd**: Updated `/etc/containerd/config.toml` with `root` directive
3. **Docker**: Updated `/etc/docker/daemon.json` with `data-root` directive
4. **etcd**: Kubeadm ClusterConfiguration with custom `dataDir`

## Benefits

1. **No Internal Storage Usage**: All Kubernetes data goes to the 2TB SSD
2. **Better Performance**: SSDs typically offer better I/O performance
3. **More Space**: 2TB available vs limited internal storage
4. **Organized Storage**: Clear separation of system, Kubernetes, and application data
5. **Easy Cleanup**: Can wipe Kubernetes data without affecting the OS
6. **Persistent Data**: Application data survives Kubernetes reinstallation

## Migration Path

If you have existing data on internal storage:

1. **Backup existing data** (if any):
   ```bash
   sudo tar -czf k8s-backup.tar.gz /var/lib/kubelet /var/lib/containerd /var/lib/etcd
   ```

2. **Run cleanup**:
   ```bash
   sudo ./cleanup.sh
   ```

3. **Run new setup**:
   ```bash
   # Master
   sudo ./setup-master.sh
   
   # Worker
   sudo ./setup-worker.sh
   ```

## Verification

After setup, verify SSD usage:

```bash
# Check that K8s is using SSD
du -sh /mnt/ssd/var/lib/*

# Check that internal storage is minimal
du -sh /var/lib/kubelet 2>/dev/null || echo "Not using internal ✓"

# Check kubelet config
ps aux | grep kubelet | grep root-dir

# Check containerd config
sudo cat /etc/containerd/config.toml | grep "^root"

# Check Docker config (worker only)
sudo cat /etc/docker/daemon.json | grep data-root
```

## Configuration File Support

Both scripts read from `k8s-deployment/config/cluster-config.yaml`:

```yaml
# Customize SSD mount point
ssd_mount: /mnt/ssd

# Other settings
pod_cidr: 10.244.0.0/16
service_cidr: 10.96.0.0/12
cni: flannel
name: barns-cluster
```

## Next Steps

1. Run the cleanup script on both nodes to remove the old crashed setup
2. Run setup-master.sh on the master Jetson
3. Run setup-worker.sh on the worker Jetson
4. Verify the cluster is using SSD storage
5. Deploy your BARNS applications

For detailed instructions, see `SSD_SETUP_GUIDE.md`.
