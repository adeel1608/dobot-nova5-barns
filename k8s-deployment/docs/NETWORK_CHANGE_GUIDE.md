# Network Change Handling Guide

This guide explains how to handle network/IP changes in your BARNS Kubernetes cluster.

## Problem

When your network changes (e.g., switching from ethernet to WiFi, different network segment), the Kubernetes cluster configuration becomes invalid because:

1. **API Server**: Configured to advertise on the old IP address
2. **Kubeconfig files**: Point to the old API server address
3. **kubelet**: Registered with the old node IP
4. **Certificates**: May include the old IP in SANs

## Solutions

### Option 1: Quick Fix - Update Existing Cluster (Recommended)

Use this if your cluster is already initialized and you just need to update the IP:

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts
sudo ./update-network.sh
```

This script will:
- Detect the new IP address
- Update all kubeconfig files
- Update kubelet configuration
- Regenerate API server certificates with the new IP
- Restart services
- Update node internal IP

**When to use**: Cluster is initialized, network changed, you want to keep existing workloads

### Option 2: Clean Reinstall

Use this if the cluster is broken or you want a fresh start:

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts

# Step 1: Clean up
sudo ./cleanup.sh

# Step 2: Fix containerd (if needed)
sudo ./fix-containerd.sh

# Step 3: Reinitialize
sudo ./setup-master.sh    # For master node
# OR
sudo ./setup-worker.sh    # For worker node
```

**When to use**: Major issues, want fresh start, cluster not properly initialized

## Immediate Fix for Your Current Issue

You're seeing this error:
```
containerd: creating temp mount location: mkdir /var/lib/containerd/tmpmounts: no such file or directory
```

This means the symlinks from `/var/lib` to SSD storage weren't created. Run:

```bash
cd ~/Barns/BARNS/k8s-deployment/scripts
sudo ./fix-containerd.sh
```

Then continue with:

```bash
sudo ./setup-master.sh
```

## Prevention - Making Cluster Network-Resilient

### For Master Nodes

The scripts now include automatic IP detection and will:
- Always detect the current active IP
- Configure API server to advertise on current IP
- Set up kubelet with current node IP

### For Worker Nodes

When joining a worker to the master:

1. On **master node**, generate a fresh join command (which will include the current IP):
   ```bash
   sudo kubeadm token create --print-join-command
   ```

2. On **worker node**, use the fresh join command:
   ```bash
   sudo kubeadm join <current-master-ip>:6443 --token <token> --discovery-token-ca-cert-hash <hash>
   ```

## Network Change Workflow

### Master Node Network Changed

```bash
# Step 1: Run network update script
sudo ./update-network.sh

# Step 2: Generate new join command for workers
sudo kubeadm token create --print-join-command

# Step 3: Update all worker nodes with new master IP (see below)
```

### Worker Node Network Changed

```bash
# Step 1: Leave the cluster (from master or worker)
kubectl drain <worker-node-name> --ignore-daemonsets --delete-emptydir-data
kubectl delete node <worker-node-name>

# Step 2: On worker, clean up
sudo ./cleanup.sh

# Step 3: On master, generate join command
sudo kubeadm token create --print-join-command

# Step 4: On worker, rejoin with new IP
sudo ./setup-worker.sh
# When prompted for join command, use the one from step 3
```

## Troubleshooting

### API Server Not Responding

```bash
# Check API server status
sudo systemctl status kubelet
sudo journalctl -xeu kubelet -n 50

# Check API server pod
sudo crictl ps -a | grep kube-apiserver
sudo crictl logs <container-id>
```

### Certificate Issues

```bash
# Check certificate SANs
openssl x509 -in /etc/kubernetes/pki/apiserver.crt -text -noout | grep -A1 "Subject Alternative Name"

# If wrong IP is there, regenerate:
sudo ./update-network.sh
```

### kubectl Not Working

```bash
# Check kubeconfig
grep "server:" ~/.kube/config

# If wrong IP, either:
# - Run update-network.sh
# - Manually copy new config:
sudo cp /etc/kubernetes/admin.conf ~/.kube/config
sudo chown $(id -u):$(id -g) ~/.kube/config
```

### containerd Not Starting

```bash
# Check containerd logs
sudo journalctl -xeu containerd.service -n 50

# If you see "no such file or directory" errors:
sudo ./fix-containerd.sh
```

## Best Practices

### 1. Use Hostnames (Not Implemented Yet)

In future versions, we can configure the cluster to use hostname instead of IP, making it network-agnostic.

### 2. Keep Network Stable

For production:
- Use static IP addresses
- Keep the Jetson on the same network
- Use a dedicated network interface for the cluster

### 3. Document Your Network

Keep track of:
- Current node IPs
- Network segments
- Port forwarding rules (if any)

### 4. Test After Changes

After any network change:

```bash
# Verify nodes are ready
kubectl get nodes

# Verify pods are running
kubectl get pods -A

# Verify services are accessible
kubectl get svc -A
```

## Scripts Reference

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `fix-containerd.sh` | Fix containerd symlink issues | containerd won't start |
| `update-network.sh` | Update cluster to new IP | Network/IP changed |
| `cleanup.sh` | Clean up Kubernetes | Before reinstall |
| `setup-master.sh` | Initialize master node | Fresh install or after cleanup |
| `setup-worker.sh` | Initialize worker node | Fresh install or after cleanup |

## Advanced: Manual Network Update

If scripts don't work, manual steps:

```bash
# 1. Stop kubelet
sudo systemctl stop kubelet

# 2. Update kubeconfig files
NEW_IP="<your-new-ip>"
sudo sed -i "s|https://.*:6443|https://${NEW_IP}:6443|g" /etc/kubernetes/admin.conf
sudo sed -i "s|https://.*:6443|https://${NEW_IP}:6443|g" /etc/kubernetes/kubelet.conf
sudo sed -i "s|https://.*:6443|https://${NEW_IP}:6443|g" ~/.kube/config

# 3. Update kubelet node-ip
sudo mkdir -p /etc/systemd/system/kubelet.service.d
sudo tee /etc/systemd/system/kubelet.service.d/20-node-ip.conf > /dev/null <<EOF
[Service]
Environment="KUBELET_EXTRA_ARGS=--node-ip=${NEW_IP}"
EOF

# 4. Reload and restart
sudo systemctl daemon-reload
sudo systemctl start kubelet

# 5. Verify
kubectl get nodes
```

## Support

If you continue to have issues after following this guide:

1. Check the logs: `sudo journalctl -xeu kubelet -n 100`
2. Verify network connectivity: `ping 8.8.8.8`
3. Verify DNS: `nslookup google.com`
4. Check if ports are open: `sudo netstat -tulpn | grep -E ':(6443|2379|2380)'`
