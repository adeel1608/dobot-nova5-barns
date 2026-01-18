# FINAL Complete K8s Setup with SSD Storage

**STOP** all quick fixes. Follow these steps exactly for a clean, proper setup.

---

## Phase 1: Complete Cleanup (Both Nodes)

### On MASTER Node (qss-jetson):
```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Run cleanup script
sudo ./cleanup.sh
# Answer: y (delete K8s resources)
# Answer: y (delete data from SSD - we'll do fresh setup)

# Manually verify all old data removed
sudo rm -rf /var/lib/containerd* /var/lib/kubelet* /var/lib/etcd* /var/lib/docker*

# Clean apt cache to free space
sudo apt clean

# Verify clean state
df -h /
sudo du -sh /var/lib/* 2>/dev/null | sort -h | tail -5
```

### On WORKER Node (ubuntu):
```bash
cd /mnt/ssd/Barns/BARNS/k8s-deployment/scripts

# Run cleanup script
sudo ./cleanup.sh
# Answer: y (delete K8s resources)
# Answer: y (delete data - BUT application data at /mnt/ssd/barns-data is preserved)

# Manually verify and remove ALL old containerd/kubelet data
sudo rm -rf /var/lib/containerd* /var/lib/kubelet* /var/lib/docker*
sudo rm -rf /mnt/ssd/var/lib/containerd* /mnt/ssd/var/lib/kubelet* /mnt/ssd/var/lib/docker*

# Clean apt cache
sudo apt clean

# Verify clean state
df -h /
sudo du -sh /var/lib/* 2>/dev/null | sort -h | tail -5
```

---

## Phase 2: Fresh Master Setup

### On MASTER Node (qss-jetson):
```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Run master setup script (NOW INCLUDES BIND MOUNT FIX!)
sudo ./setup-master.sh

# When prompted:
# - Confirm configuration (y)
# - Let it complete fully

# The script will automatically:
# - Configure containerd for SSD with BIND MOUNT (forces SSD usage)
# - Set kubelet rootDirectory to SSD
# - Initialize cluster with etcd on SSD
# - Make bind mount persistent in /etc/fstab

# Verify master is ready
kubectl get nodes
kubectl get pods -A

# Verify SSD usage
sudo du -sh /mnt/ssd/var/lib/{containerd,kubelet,etcd}
df -h / /mnt/ssd

# Verify bind mount is active
df -h | grep containerd
# Should show: /dev/nvme0n1p1 mounted on /var/lib/containerd

# Verify kubelet config
sudo cat /var/lib/kubelet/config.yaml | grep rootDirectory
# Should show: rootDirectory: /mnt/ssd/var/lib/kubelet

# Check containerd config
sudo cat /etc/containerd/config.toml | grep "^root"
# Should show: root = "/mnt/ssd/var/lib/containerd"
```

### Copy Join Command to Worker:
```bash
# On master - the setup script creates this file
cat /tmp/k8s-join-command.sh

# Copy this to worker node at /tmp/k8s-join-command.sh
```

---

## Phase 3: Fresh Worker Setup

### Copy Join Command First:
```bash
# On MASTER - show join command
cat /tmp/k8s-join-command.sh

# Copy it to worker at /tmp/k8s-join-command.sh
# (Use scp or manually copy the command)
```

### On WORKER Node (ubuntu):
```bash
cd /mnt/ssd/Barns/BARNS/k8s-deployment/scripts

# Ensure join command is present
ls -la /tmp/k8s-join-command.sh

# Run worker setup script (NOW INCLUDES BIND MOUNT FIX!)
sudo ./setup-worker.sh

# When prompted:
# - Confirm configuration (y)
# - Answer y to join cluster now

# The script will automatically:
# - Configure containerd for SSD with BIND MOUNT (forces SSD usage)
# - Configure Docker for SSD
# - Set kubelet rootDirectory to SSD
# - Join the cluster
# - Make bind mount persistent in /etc/fstab

# Verify worker joined (check from master: kubectl get nodes)

# Verify SSD usage on worker
sudo du -sh /mnt/ssd/var/lib/{containerd,kubelet,docker}
df -h / /mnt/ssd

# Verify bind mount is active
df -h | grep containerd
# Should show: /dev/nvme0n1p1 mounted on /var/lib/containerd

# Verify kubelet config
sudo cat /var/lib/kubelet/config.yaml | grep rootDirectory
# Should show: rootDirectory: /mnt/ssd/var/lib/kubelet

# Check containerd config
sudo cat /etc/containerd/config.toml | grep "^root"
# Should show: root = "/mnt/ssd/var/lib/containerd"
```

---

## Phase 4: Build and Import Docker Images

### On WORKER Node (ubuntu):
```bash
cd /mnt/ssd/Barns/BARNS

# Build all images for ARM64
./build-images-arm64.sh

# Wait for build to complete (may take 10-20 minutes)

# Verify images built
docker images | grep barns

# Import images to containerd k8s.io namespace
for image in $(docker images --format "{{.Repository}}:{{.Tag}}" | grep barns); do
    echo "Importing $image..."
    docker save "$image" | sudo ctr -n k8s.io images import - --all-platforms
done

# Verify imported
sudo ctr -n k8s.io images ls | grep barns

# Check disk usage (should be on SSD)
df -h / /mnt/ssd
sudo du -sh /mnt/ssd/var/lib/containerd
```

### On MASTER Node (qss-jetson):
```bash
cd ~/Barns/BARNS

# Build robot1 image only (since it runs on master)
# Extract robot1 build section from build-images-arm64.sh or run full script

# If robot1 image exists in Docker
docker images | grep robot1

# Import to containerd
docker save barns-robot1:latest | sudo ctr -n k8s.io images import - --all-platforms

# Verify
sudo ctr -n k8s.io images ls | grep robot1
```

---

## Phase 5: Deploy Applications

### On MASTER Node:
```bash
cd ~/Barns/BARNS/k8s

# Apply all deployments
kubectl apply -f namespace.yaml
kubectl apply -f storage/
kubectl apply -f services/

# Wait for pods to come up
watch kubectl get pods -n barns

# Check for any issues
kubectl get pods -n barns | grep -v Running
kubectl describe pod -n barns <failing-pod-name>
```

---

## Phase 6: Final Verification

### On MASTER Node:
```bash
# Check cluster health
kubectl get nodes -o wide
kubectl get pods -A

# Verify storage usage
df -h / /mnt/ssd
sudo du -sh /mnt/ssd/var/lib/{containerd,kubelet,etcd}

# Should see:
# - Internal storage: ~25-30GB used (40-50%)
# - SSD: All K8s data (containerd, kubelet, etcd)

# Verify no disk pressure
kubectl describe node qss-jetson | grep Taints
kubectl describe node ubuntu | grep Taints
# Should show: Taints: <none>
```

### On WORKER Node:
```bash
# Verify storage usage
df -h / /mnt/ssd
sudo du -sh /mnt/ssd/var/lib/{containerd,kubelet,docker}

# Should see:
# - Internal storage: ~35-45GB used (60-75%)
# - SSD: All K8s + Docker data

# Verify services running
sudo systemctl status containerd | grep Active
sudo systemctl status kubelet | grep Active
```

---

## Troubleshooting

### If containerd creates /var/lib/containerd on internal storage:

**This should NOT happen with the updated scripts (they include bind mount fix).**

But if it does:

```bash
# Stop services
sudo systemctl stop kubelet
sudo systemctl stop containerd

# Move data to SSD
sudo rsync -a /var/lib/containerd/ /mnt/ssd/var/lib/containerd/
sudo rm -rf /var/lib/containerd
sudo mkdir -p /var/lib/containerd

# Create bind mount
sudo mount --bind /mnt/ssd/var/lib/containerd /var/lib/containerd

# Make it persistent
echo "/mnt/ssd/var/lib/containerd /var/lib/containerd none bind 0 0" | sudo tee -a /etc/fstab

# Restart services
sudo systemctl start containerd
sudo systemctl start kubelet

# Verify
df -h | grep containerd
```

### If pods show ErrImageNeverPull or ImagePullBackOff:

Images not in containerd's k8s.io namespace. Import them:
```bash
docker save <image-name>:latest | sudo ctr -n k8s.io images import - --all-platforms
```

### If disk pressure taint persists:

```bash
# Force remove taint
kubectl taint nodes <node-name> node.kubernetes.io/disk-pressure-

# Free up more internal storage if needed
sudo apt clean
sudo apt autoremove -y
```

---

## Success Criteria

✅ Both nodes show **Ready** with no taints
✅ All BARNS pods **Running**
✅ Master: containerd (~1GB), kubelet (minimal), etcd (~128MB) on SSD
✅ Worker: containerd (~25GB), kubelet (minimal), docker (~23GB) on SSD
✅ Master internal storage: <50% used
✅ Worker internal storage: <80% used
✅ No /var/lib/containerd growing on internal storage

---

## Notes

- The setup scripts (`setup-master.sh`, `setup-worker.sh`) now directly modify `/var/lib/kubelet/config.yaml` to set `rootDirectory` after kubeadm runs
- Containerd config sets `root = "/mnt/ssd/var/lib/containerd"`
- If containerd still misbehaves, use bind mount as workaround
- Keep Docker images on worker for rebuilding, or delete with `docker system prune -a` after confirming pods work
