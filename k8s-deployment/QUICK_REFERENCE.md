# Quick Reference Card

## 🚀 Your Immediate Next Steps

Based on your current situation (containerd failed to start after cleanup):

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Step 1: Fix containerd symlinks
sudo ./fix-containerd.sh

# Step 2: Setup master node
sudo ./setup-master.sh
```

That's it! The setup will now work properly and automatically detect your current IP (192.168.8.107).

---

## 📋 Script Reference

### Problem: "containerd won't start"
```bash
sudo ./fix-containerd.sh
```

### Problem: "Network/IP changed, cluster not working"
```bash
sudo ./update-network.sh
```

### Problem: "Need to start fresh"
```bash
sudo ./cleanup.sh
sudo ./fix-containerd.sh
sudo ./setup-master.sh    # or setup-worker.sh
```

### Setup: "New master node"
```bash
sudo ./setup-master.sh
```

### Setup: "New worker node"
```bash
# First, get join command from master:
# On master: sudo kubeadm token create --print-join-command

# Then on worker:
sudo ./setup-worker.sh
```

---

## 🔍 Quick Diagnostics

### Check if containerd is working
```bash
sudo systemctl status containerd
```
**Expected:** `Active: active (running)`  
**If not:** Run `sudo ./fix-containerd.sh`

### Check if Kubernetes is working
```bash
kubectl get nodes
```
**Expected:** Node(s) in `Ready` status  
**If timeout/error:** Check IP with `grep "server:" ~/.kube/config`

### Check current IP
```bash
hostname -I | awk '{print $1}'
```

### Check symlinks
```bash
ls -la /var/lib/ | grep -E '(containerd|kubelet)'
```
**Expected:** Symlinks pointing to `/mnt/ssd/var/lib/*`  
**If missing:** Run `sudo ./fix-containerd.sh`

---

## 🔄 Common Workflows

### Workflow: Network Changed
```bash
# If cluster was working before:
sudo ./update-network.sh

# If cluster wasn't initialized yet:
sudo ./setup-master.sh    # Will auto-detect new IP
```

### Workflow: Complete Reinstall
```bash
# Clean everything
sudo ./cleanup.sh
# Answer 'y' to remove SSD data

# Fix storage
sudo ./fix-containerd.sh

# Setup again
sudo ./setup-master.sh    # or setup-worker.sh
```

### Workflow: Join Worker to Cluster
```bash
# On MASTER node:
sudo kubeadm token create --print-join-command

# Copy the output, then on WORKER node:
sudo ./setup-worker.sh
# When prompted, paste the join command
```

### Workflow: Worker Network Changed
```bash
# On master, remove the worker:
kubectl drain <worker-name> --ignore-daemonsets --delete-emptydir-data
kubectl delete node <worker-name>

# On worker, clean up and rejoin:
sudo ./cleanup.sh
sudo ./fix-containerd.sh
sudo ./setup-worker.sh
```

---

## ⚠️ Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| `mkdir /var/lib/containerd/tmpmounts: no such file` | Symlinks missing | `sudo ./fix-containerd.sh` |
| `dial tcp 10.0.0.78:6443: i/o timeout` | Old IP in config | `sudo ./update-network.sh` |
| `The connection to the server was refused` | Cluster not initialized | `sudo ./setup-master.sh` |
| `Unable to connect to the server: x509` | Certificate/IP mismatch | `sudo ./update-network.sh` |
| `Error: Job for containerd.service failed` | Directories/symlinks wrong | `sudo ./fix-containerd.sh` |

---

## 📖 Documentation Files

- **NETWORK_FIX_SUMMARY.md** - Detailed explanation of what was wrong and how it was fixed
- **NETWORK_CHANGE_GUIDE.md** - Comprehensive guide for handling network changes
- **TROUBLESHOOTING.md** - General troubleshooting guide (if exists)
- **This file (QUICK_REFERENCE.md)** - Quick commands for common scenarios

---

## ✅ Post-Setup Verification

After running setup, verify everything:

```bash
# 1. Check nodes
kubectl get nodes

# 2. Check system pods
kubectl get pods -A

# 3. Check services
kubectl get svc -A

# 4. Verify IP is correct
kubectl get nodes -o wide

# 5. Test kubectl
kubectl version --short
```

All commands should work without errors!

---

## 💡 Pro Tips

1. **Before changing networks:** Note your current setup and be prepared to run `update-network.sh`
2. **Keep join command:** Save the join command from master in a secure place
3. **Test after changes:** Always run `kubectl get nodes` after network changes
4. **Use static IPs in production:** Avoids these issues entirely
5. **Bookmark this file:** Quick reference for when things go wrong!

---

## 🆘 Still Stuck?

1. Check the logs:
   ```bash
   sudo journalctl -xeu kubelet -n 100
   sudo journalctl -xeu containerd -n 100
   ```

2. Verify basics:
   ```bash
   ping 8.8.8.8                          # Internet
   sudo systemctl status containerd       # Runtime
   ls -la /var/lib/containerd             # Symlink
   df -h /mnt/ssd                         # Storage
   ```

3. Nuclear option (complete reset):
   ```bash
   sudo ./cleanup.sh
   sudo ./fix-containerd.sh
   sudo ./setup-master.sh
   ```
