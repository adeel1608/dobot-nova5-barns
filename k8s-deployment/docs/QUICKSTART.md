# BARNS Kubernetes Quick Start Guide

Get your BARNS deployment up and running in minutes!

## 🎯 Prerequisites

- **Master Node** (ARM64 or AMD64): Ubuntu 20.04+, 2+ cores, 4GB+ RAM
- **Worker Node** (AMD64): Ubuntu 20.04+, 4+ cores, 8GB+ RAM, 50GB+ disk
- SSH access to both nodes
- Git installed on both nodes

## 🚀 Quick Deploy (5 Steps)

### Step 1: Configure Your Cluster (2 minutes)

```bash
cd k8s-deployment
nano config/cluster-config.yaml
```

Update these values:
- `master.ip`: Your master node IP (or leave as `auto`)
- `worker.ip`: Your worker node IP (or leave as `auto`)
- `master.ssh_user`: SSH username for master
- `worker.ssh_user`: SSH username for worker

### Step 2: Setup Master Node (5 minutes)

```bash
# Copy deployment package to master node
scp -r k8s-deployment/ user@master-node:~/

# SSH to master
ssh user@master-node

# Run setup
cd k8s-deployment/scripts
chmod +x *.sh
sudo ./setup-master.sh
```

**Save the join command** displayed at the end!

### Step 3: Setup Worker Node (5 minutes)

```bash
# Copy deployment package to worker node
scp -r k8s-deployment/ user@worker-node:~/

# Copy join command from master
scp user@master-node:/tmp/k8s-join-command.sh /tmp/

# SSH to worker
ssh user@worker-node

# Run setup
cd k8s-deployment/scripts
chmod +x *.sh
sudo ./setup-worker.sh
```

When prompted, join the cluster.

### Step 4: Clone BARNS Repository (2 minutes)

On worker node:

```bash
# Clone BARNS source code
mkdir -p ~/git-BARNS
cd ~/git-BARNS
git clone <your-barns-repo-url> BARNS

# Or if you already have it, just pull latest
cd ~/git-BARNS/BARNS
git pull
```

### Step 5: Build & Deploy (10-15 minutes)

Back on master node:

```bash
cd k8s-deployment/scripts

# Build Docker images (builds on worker via SSH)
./build-images.sh

# Deploy to Kubernetes
./deploy-k8s.sh

# Initialize database
./fix-database.sh

# Validate deployment
./validate.sh
```

## ✅ Access Your Deployment

Once deployed, access BARNS at:

- **Dashboard**: http://YOUR-WORKER-IP:30003
- **API**: http://YOUR-WORKER-IP:30000
- **Video Stream**: http://YOUR-WORKER-IP:30001

Replace `YOUR-WORKER-IP` with your worker node's IP address.

## 🔧 Troubleshooting Quick Fixes

### Pods Not Starting

```bash
# Check pod status
kubectl get pods -n barns

# Check specific pod logs
kubectl logs <pod-name> -n barns

# Restart failed pod
kubectl delete pod <pod-name> -n barns
```

### Database Errors

```bash
# Re-run database fix
cd k8s-deployment/scripts
./fix-database.sh
```

### Image Architecture Errors

```bash
# On worker node, verify architecture
docker inspect barns-validation:latest | grep Architecture
# Should show: "Architecture": "amd64"

# If wrong, rebuild on worker node
cd ~/git-BARNS/BARNS
docker build --no-cache -t barns-validation:latest -f services/validation/Dockerfile.rabbitmq .
```

### Network Changed / New IPs

```bash
# On master node
sudo /usr/local/bin/k8s-update-network.sh

# Update config
cd k8s-deployment
nano config/cluster-config.yaml
# Update IPs

# Restart pods
kubectl rollout restart deployment -n barns
```

## 📋 Common Commands

```bash
# View all pods
kubectl get pods -n barns

# Watch pods (live updates)
kubectl get pods -n barns -w

# View pod logs
kubectl logs <pod-name> -n barns

# Follow logs
kubectl logs -f <pod-name> -n barns

# Restart a service
kubectl rollout restart deployment <service-name> -n barns

# Restart all services
kubectl rollout restart deployment -n barns

# Get service URLs
kubectl get svc -n barns

# Check nodes
kubectl get nodes -o wide

# Validate deployment
cd k8s-deployment/scripts
./validate.sh
```

## 🔄 Update BARNS Code

```bash
# On worker node
cd ~/git-BARNS/BARNS
git pull

# On master node
cd k8s-deployment/scripts
./build-images.sh
kubectl rollout restart deployment -n barns
```

## 🆘 Getting Help

1. **Check logs**: `kubectl logs <pod-name> -n barns`
2. **Run validation**: `./scripts/validate.sh`
3. **Check events**: `kubectl get events -n barns --sort-by='.lastTimestamp'`
4. **Describe pod**: `kubectl describe pod <pod-name> -n barns`

## 📚 Next Steps

- Read [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for detailed solutions
- Check [NETWORKING.md](docs/NETWORKING.md) for network configuration
- See [MAINTENANCE.md](docs/MAINTENANCE.md) for ongoing maintenance

## 🎉 Success!

If everything is working, you should see:
- All pods in `Running` state
- Dashboard accessible in browser
- Camera streams working
- Orders processing successfully

Enjoy your BARNS deployment! ☕🤖

