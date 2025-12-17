# BARNS Kubernetes Troubleshooting Guide

Common issues and their solutions.

## 🔍 Quick Diagnostics

```bash
# Run validation script
cd k8s-deployment/scripts
./validate.sh

# Check overall cluster health
kubectl get nodes
kubectl get pods -n barns
kubectl get events -n barns --sort-by='.lastTimestamp'
```

## 🐛 Common Issues

### 1. Pod Stuck in Pending State

**Symptoms:**
- Pod shows `Pending` status
- Doesn't start after several minutes

**Diagnosis:**
```bash
kubectl describe pod <pod-name> -n barns
```

**Common Causes & Solutions:**

#### A. Insufficient Resources
```
Events: FailedScheduling: 0/2 nodes are available: insufficient memory
```
**Solution:** Reduce resource limits or add more nodes

#### B. PVC Not Bound
```
Events: FailedMount: PersistentVolumeClaim is not bound
```
**Solution:**
```bash
# Check PVCs
kubectl get pvc -n barns

# Check PVs
kubectl get pv

# Recreate storage
kubectl delete pvc <pvc-name> -n barns
kubectl apply -f ../../../k8s/storage/
```

#### C. Node Selector Issues
**Solution:** Check node labels and deployment nodeSelector match

### 2. Pod in CrashLoopBackOff

**Symptoms:**
- Pod restarts repeatedly
- Status shows `CrashLoopBackOff`

**Diagnosis:**
```bash
kubectl logs <pod-name> -n barns
kubectl logs <pod-name> -n barns --previous  # Previous crash
```

**Common Causes & Solutions:**

#### A. Database Connection Failed
```
Error: connection to server at "localhost" failed
```
**Solution:**
```bash
# Check if postgres is running
kubectl get pods -n barns | grep postgres

# Fix database
./fix-database.sh

# Update service to use correct hostname
# In deployment YAML: POSTGRES_HOST should be "postgres" not "localhost"
```

#### B. Image Architecture Mismatch
```
exec /usr/local/bin/python: exec format error
```
**Solution:**
```bash
# Rebuild images on correct architecture
# On worker node (AMD64):
cd ~/git-BARNS/BARNS
docker build --no-cache -t barns-validation:latest -f services/validation/Dockerfile.rabbitmq .

# Import to containerd
docker save barns-validation:latest | sudo ctr -n k8s.io image import -

# Restart pod
kubectl delete pod <pod-name> -n barns
```

#### C. Missing Dependencies / Inventory Errors
```
ERROR: No inventory found for cups:H7
```
**Solution:**
```bash
./fix-database.sh
```

### 3. ImagePullBackOff / ErrImagePull

**Symptoms:**
- Pod shows `ImagePullBackOff` or `ErrImagePull`
- Cannot pull image

**Diagnosis:**
```bash
kubectl describe pod <pod-name> -n barns | grep -A5 Events
```

**Solutions:**

#### A. Image Not in Local containerd
```bash
# Check if image exists
ssh worker-node "sudo ctr -n k8s.io images ls | grep barns-"

# If missing, rebuild
cd k8s-deployment/scripts
./build-images.sh
```

#### B. Image Pull Policy Issue
Edit deployment YAML:
```yaml
spec:
  containers:
  - name: validation
    image: barns-validation:latest
    imagePullPolicy: IfNotPresent  # Use local images first
```

Apply changes:
```bash
kubectl apply -f k8s/services/validation-service.yaml
```

### 4. Validation Service Crashes (ML Model Issues)

**Symptoms:**
- Validation service crashes with `FATAL: exception not rethrown`
- SIGSEGV errors

**Diagnosis:**
```bash
kubectl logs -l app=validation-service -n barns | grep FATAL
```

**Solutions:**

#### A. Pre-download ML Model
```bash
# SSH to worker node
ssh worker-node

# Download model
sudo mkdir -p /mnt/barns-data/cup_models
cd /mnt/barns-data/cup_models
sudo wget https://huggingface.co/omeryagmur/rf-detr-large/resolve/main/rf-detr-large.pth

# Set permissions
sudo chmod 644 rf-detr-large.pth
```

#### B. Increase Memory Limits
Edit `k8s/services/validation-service.yaml`:
```yaml
resources:
  limits:
    memory: 4Gi  # Increase from 2Gi
    cpu: 2000m
```

Apply:
```bash
kubectl apply -f k8s/services/validation-service.yaml
```

### 5. Dashboard Not Loading / Connection Refused

**Symptoms:**
- Dashboard URL times out
- Connection refused errors

**Solutions:**

#### A. Check Dashboard Pod
```bash
kubectl get pods -n barns | grep dashboard
kubectl logs <dashboard-pod> -n barns
```

#### B. Check NodePort Service
```bash
kubectl get svc dashboard -n barns

# Should show:
# NAME        TYPE       CLUSTER-IP      EXTERNAL-IP   PORT(S)        AGE
# dashboard   NodePort   10.96.xxx.xxx   <none>        80:30003/TCP   10m
```

#### C. Check Worker Node Firewall
```bash
# On worker node
sudo ufw status
sudo ufw allow 30003/tcp  # If using ufw
```

#### D. Access from Same Network
Dashboard is exposed on worker node's IP. Ensure you're accessing from the same network or have proper routing.

### 6. Camera Stream Not Working

**Symptoms:**
- Dashboard shows "Stream Unavailable"
- Video stream returns 404

**Diagnosis:**
```bash
# Check video-stream service
kubectl logs -l app=video-stream-service -n barns

# Test camera from worker node
ssh worker-node
ffmpeg -i rtsp://192.168.200.41:8554/ceiling -frames:v 1 test.jpg
```

**Solutions:**

#### A. Update Dashboard Configuration
The dashboard may be trying to connect to `localhost:8001`. Check:
```bash
# Dashboard should use NodePort URL
# In UnifiedCameraPanel.jsx:
# const streamUrl = `http://192.168.200.142:30001/stream/${cameraId}`;
```

#### B. Check Camera Network
Ensure camera is reachable from worker node:
```bash
ssh worker-node
ping 192.168.200.41
nc -zv 192.168.200.41 8554
```

#### C. Check Service Configuration
```bash
kubectl get svc video-stream-service -n barns
# Should expose port 30001
```

### 7. Database Not Initializing

**Symptoms:**
- Postgres pod runs but tables don't exist
- "Skipping initialization" in logs

**Diagnosis:**
```bash
kubectl logs postgres-0 -n barns | grep initialization
```

**Solution:**
```bash
# Complete database reset
kubectl scale statefulset postgres --replicas=0 -n barns
kubectl delete pvc postgres-pvc -n barns
kubectl delete pv postgres-pv

# On worker node, clean data
ssh worker-node "sudo rm -rf /mnt/barns-data/postgres/pgdata/*"

# Recreate
kubectl apply -f k8s/storage/postgres-pv-pvc.yaml
kubectl scale statefulset postgres --replicas=1 -n barns

# Wait and fix
sleep 30
./fix-database.sh
```

### 8. Network Changed / Cluster Not Working After IP Change

**Symptoms:**
- Pods not starting after network change
- API server unreachable

**Solution:**

#### On Master Node:
```bash
# Update API server configuration
sudo /usr/local/bin/k8s-update-network.sh

# Or manually:
NEW_IP=$(ip route get 8.8.8.8 | grep -oP 'src \K\S+')
sudo sed -i "s/--advertise-address=.*/--advertise-address=${NEW_IP}/" \
    /etc/kubernetes/manifests/kube-apiserver.yaml
sudo systemctl restart kubelet
```

#### On Worker Node:
```bash
# Kubelet will automatically use new IP if configured with dynamic node-ip
sudo systemctl restart kubelet
```

#### Update Pods:
```bash
kubectl rollout restart deployment -n barns
```

### 9. Node Not Ready

**Symptoms:**
- `kubectl get nodes` shows `NotReady`
- Pods stuck in `Pending`

**Diagnosis:**
```bash
kubectl describe node <node-name>
sudo journalctl -u kubelet -n 100
```

**Common Causes & Solutions:**

#### A. CNI Plugin Not Ready
```bash
# Check CNI pods
kubectl get pods -n kube-flannel  # or kube-system for other CNIs

# Reinstall if needed
kubectl delete -f https://github.com/flannel-io/flannel/releases/latest/download/kube-flannel.yml
kubectl apply -f https://github.com/flannel-io/flannel/releases/latest/download/kube-flannel.yml
```

#### B. Containerd/Kubelet Issues
```bash
sudo systemctl restart containerd
sudo systemctl restart kubelet
```

### 10. Service Not Accessible via NodePort

**Symptoms:**
- NodePort service created but not accessible
- Connection timeout

**Solutions:**

#### A. Check Service
```bash
kubectl get svc <service-name> -n barns
kubectl describe svc <service-name> -n barns
```

#### B. Check if Pod is Ready
```bash
kubectl get pods -n barns | grep <service-name>
# Pod must be in Running state
```

#### C. Check Node IP
```bash
kubectl get nodes -o wide
# Use INTERNAL-IP of worker node, not master
```

#### D. Test from Master Node
```bash
curl http://<worker-ip>:<nodeport>
```

## 🔧 Useful Commands

### View All Resources
```bash
kubectl get all -n barns
```

### Get Events (Sorted)
```bash
kubectl get events -n barns --sort-by='.lastTimestamp'
```

### Force Delete Pod
```bash
kubectl delete pod <pod-name> -n barns --force --grace-period=0
```

### Restart Deployment
```bash
kubectl rollout restart deployment <deployment-name> -n barns
```

### Watch Pods (Live)
```bash
kubectl get pods -n barns -w
```

### Get Pod YAML
```bash
kubectl get pod <pod-name> -n barns -o yaml
```

### Execute in Pod
```bash
kubectl exec -it <pod-name> -n barns -- /bin/bash
```

### Port Forward (Local Testing)
```bash
kubectl port-forward svc/dashboard 8080:80 -n barns
# Access at http://localhost:8080
```

## 📊 Performance Issues

### High CPU Usage
```bash
# Check resource usage
kubectl top nodes
kubectl top pods -n barns

# Increase limits if needed
kubectl edit deployment <deployment-name> -n barns
```

### High Memory Usage
Check for memory leaks in logs, increase limits, or investigate application code.

### Slow Response Times
- Check database performance
- Review RabbitMQ queue lengths
- Check network latency

## 🆘 Last Resort

If all else fails:

```bash
# Complete cleanup and redeploy
cd k8s-deployment/scripts
./cleanup.sh
./deploy-k8s.sh
./fix-database.sh
```

## 📞 Getting Help

1. Check logs: `kubectl logs <pod-name> -n barns`
2. Describe resources: `kubectl describe pod/deployment/service <name> -n barns`
3. Check events: `kubectl get events -n barns`
4. Run validation: `./scripts/validate.sh`
5. Review this troubleshooting guide

If issues persist, gather:
- Output of `./validate.sh`
- Pod logs
- Events
- Node status

And seek help from the BARNS team.

