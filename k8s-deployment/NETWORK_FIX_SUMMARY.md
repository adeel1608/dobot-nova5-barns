# Network Change Fix Summary

## What Was Wrong

You encountered two related issues after your network changed from `10.0.0.78` to `192.168.8.107`:

### 1. **containerd Startup Failure**

**Error:**
```
containerd: creating temp mount location: mkdir /var/lib/containerd/tmpmounts: no such file or directory
```

**Root Cause:**
- The cleanup script removed `/var/lib/containerd` (symlink)
- The setup script created `/mnt/ssd/var/lib/containerd` directory
- But it **didn't create the symlink** from `/var/lib/containerd` → `/mnt/ssd/var/lib/containerd`
- containerd expected `/var/lib/containerd` to exist

### 2. **kubectl Connection Failure**

**Error:**
```
Get "https://10.0.0.78:6443/api?timeout=32s": dial tcp 10.0.0.78:6443: i/o timeout
```

**Root Cause:**
- Your kubeconfig files still pointed to the old IP (10.0.0.78)
- The cluster (if initialized) was configured with the old IP
- Network change made the old IP unreachable

## What Was Fixed

### ✅ Fixed Scripts

1. **`setup-master.sh`** - Now creates symlinks:
   ```bash
   ln -sf /mnt/ssd/var/lib/kubelet /var/lib/kubelet
   ln -sf /mnt/ssd/var/lib/containerd /var/lib/containerd
   ln -sf /mnt/ssd/var/lib/etcd /var/lib/etcd
   ```

2. **`setup-worker.sh`** - Now creates symlinks:
   ```bash
   ln -sf /mnt/ssd/var/lib/kubelet /var/lib/kubelet
   ln -sf /mnt/ssd/var/lib/containerd /var/lib/containerd
   ln -sf /mnt/ssd/var/lib/docker /var/lib/docker
   ```

3. **`cleanup.sh`** - Improved to:
   - Remove symlinks before cleaning SSD
   - Stop services properly before removal
   - Not restart services after cleanup (they'll be configured by setup script)

### ✅ New Scripts Created

1. **`fix-containerd.sh`** - Quick fix for containerd symlink issue
   - Stops containerd
   - Creates SSD directories
   - Creates proper symlinks
   - Starts containerd
   - Verifies it's working

2. **`update-network.sh`** - Handle network/IP changes
   - Detects new IP automatically
   - Updates all kubeconfig files
   - Updates kubelet configuration
   - Regenerates API server certificates
   - Restarts services
   - Updates cluster configuration

### ✅ Documentation Created

1. **`NETWORK_CHANGE_GUIDE.md`** - Comprehensive guide on:
   - Understanding the problem
   - Multiple solution approaches
   - Prevention strategies
   - Troubleshooting steps
   - Best practices

## How to Proceed Now

You have **two options** depending on what state your cluster is in:

### Option A: Fresh Start (Recommended for You)

Since your setup failed midway, this is the cleanest approach:

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Step 1: Fix containerd
sudo ./fix-containerd.sh

# Step 2: Setup master node (will use current IP automatically)
sudo ./setup-master.sh

# Step 3: Verify
kubectl get nodes
```

**Expected Result:**
- containerd starts successfully
- Kubernetes initializes with IP `192.168.8.107`
- Master node becomes Ready
- You get a join command for worker nodes

### Option B: Update Existing Cluster (If Already Initialized)

If you had a working cluster before the network change:

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Step 1: Fix containerd (if needed)
sudo ./fix-containerd.sh

# Step 2: Update network configuration
sudo ./update-network.sh

# Step 3: Verify
kubectl get nodes
```

**Expected Result:**
- Cluster updates to use new IP
- All nodes remain in Ready state
- Existing workloads continue running

## Verification Steps

After running the appropriate option:

### 1. Check containerd
```bash
sudo systemctl status containerd
# Should show: Active: active (running)
```

### 2. Check symlinks
```bash
ls -la /var/lib/ | grep -E '(containerd|kubelet|etcd|docker)'
# Should show symlinks pointing to /mnt/ssd/var/lib/*
```

### 3. Check Kubernetes
```bash
kubectl get nodes
# Should show nodes in Ready state

kubectl get pods -A
# Should show system pods running
```

### 4. Check IP configuration
```bash
kubectl get nodes -o wide
# Should show current IP (192.168.8.107)

grep "server:" ~/.kube/config
# Should show: https://192.168.8.107:6443
```

## Why This Won't Happen Again

The fixed scripts now:

1. **Always create symlinks** during setup
2. **Auto-detect current IP** during initialization
3. **Properly clean up** during teardown
4. **Provide recovery tools** (`fix-containerd.sh`, `update-network.sh`)

If your network changes again in the future:
- Just run `sudo ./update-network.sh` on the master
- Regenerate join command: `sudo kubeadm token create --print-join-command`
- Update worker nodes if needed

## Next Steps After Successful Setup

1. **Join Worker Nodes:**
   ```bash
   # On master
   sudo kubeadm token create --print-join-command
   
   # On worker
   sudo ./setup-worker.sh
   # Use the join command when prompted
   ```

2. **Deploy Your Application:**
   ```bash
   cd ~/Barns/BARNS/k8s
   sudo ./deploy.sh
   ```

3. **Verify Everything:**
   ```bash
   kubectl get all -A
   ```

## If You Still Have Issues

1. **containerd won't start:**
   ```bash
   sudo ./fix-containerd.sh
   sudo journalctl -xeu containerd -n 50
   ```

2. **API server not accessible:**
   ```bash
   sudo systemctl status kubelet
   sudo journalctl -xeu kubelet -n 50
   ```

3. **Wrong IP still showing:**
   ```bash
   sudo ./update-network.sh
   ```

4. **Complete reset needed:**
   ```bash
   sudo ./cleanup.sh
   sudo ./fix-containerd.sh
   sudo ./setup-master.sh
   ```

## Summary

| Issue | Root Cause | Fix |
|-------|------------|-----|
| containerd won't start | Missing symlinks | `fix-containerd.sh` + updated setup scripts |
| kubectl can't connect | Old IP in configs | `update-network.sh` or fresh setup |
| Network change breaks cluster | Hardcoded IPs | Auto-detect IP + update scripts |

All the fixes have been applied to your scripts. You can now proceed with a clean setup that will work regardless of network changes! 🎉
