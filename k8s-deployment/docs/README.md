# BARNS Kubernetes Deployment Package

**Version:** 1.0.0  
**Last Updated:** December 2024

## 📋 Overview

This deployment package provides automated setup and deployment of the BARNS (Barista Autonomous Robotic Navigation System) on Kubernetes clusters. It's designed to be:

- **Network Agnostic**: Automatically detects and adapts to network changes
- **Reusable**: Can be used to deploy multiple clusters for different branches
- **Clean & Maintainable**: Minimal dependencies, clear structure
- **Production Ready**: Includes validation, monitoring, and troubleshooting tools

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Master Node (ARM64)                   │
│  - Kubernetes Control Plane                             │
│  - kubectl, kubeadm, kubelet                            │
│  - Deployment Scripts                                   │
└─────────────────────────────────────────────────────────┘
                           │
                           │ Pod Network (Flannel)
                           │
┌─────────────────────────────────────────────────────────┐
│                   Worker Node (AMD64)                    │
│  - Docker Engine                                        │
│  - containerd (K8s runtime)                             │
│  - BARNS Service Pods                                   │
│  - Infrastructure Pods (Postgres, RabbitMQ, Redis)     │
└─────────────────────────────────────────────────────────┘
```

## 📁 Directory Structure

```
k8s-deployment/
├── README.md                 # This file
├── QUICKSTART.md            # Quick start guide
├── config/
│   ├── cluster-config.yaml  # Cluster configuration
│   └── inventory.yaml       # Node inventory
├── scripts/
│   ├── common.sh           # Shared functions
│   ├── setup-master.sh     # Master node setup
│   ├── setup-worker.sh     # Worker node setup
│   ├── build-images.sh     # Build Docker images
│   ├── deploy-k8s.sh       # Deploy to K8s
│   ├── fix-database.sh     # Initialize database
│   ├── validate.sh         # Validate deployment
│   └── cleanup.sh          # Cleanup/uninstall
├── templates/              # Kubernetes YAML templates
└── docs/                   # Additional documentation
    ├── TROUBLESHOOTING.md
    ├── NETWORKING.md
    └── MAINTENANCE.md
```

## 🚀 Quick Start

### Prerequisites

**Master Node:**
- Ubuntu 20.04+ or similar Linux distribution
- 2+ CPU cores, 4GB+ RAM
- Network connectivity
- sudo access

**Worker Node:**
- Ubuntu 20.04+ or similar Linux distribution
- 4+ CPU cores, 8GB+ RAM
- 50GB+ free disk space
- Docker installed
- sudo access

### Step 1: Configure Cluster

Edit `config/cluster-config.yaml` with your environment details:

```bash
cd k8s-deployment
nano config/cluster-config.yaml
```

### Step 2: Setup Master Node

On your master node:

```bash
# Copy deployment package to master node
scp -r k8s-deployment/ user@master-node:~/

# SSH to master node
ssh user@master-node

# Run master setup
cd k8s-deployment/scripts
chmod +x *.sh
./setup-master.sh
```

### Step 3: Setup Worker Node

After master is ready, on your worker node:

```bash
# Copy deployment package to worker node
scp -r k8s-deployment/ user@worker-node:~/

# SSH to worker node
ssh user@worker-node

# Run worker setup
cd k8s-deployment/scripts
chmod +x *.sh
./setup-worker.sh
```

### Step 4: Build & Deploy

Back on master node:

```bash
# Build images (will SSH to worker)
./build-images.sh

# Deploy to Kubernetes
./deploy-k8s.sh

# Initialize database
./fix-database.sh

# Validate deployment
./validate.sh
```

## 🌐 Network Change Handling

This deployment package is designed to handle network changes gracefully:

1. **Dynamic IP Detection**: Scripts automatically detect node IPs
2. **Configuration Updates**: Network settings can be updated without reinstall
3. **Service Reconfiguration**: Services adapt to new network topology

### Handling Network Changes

If your network changes (different subnet, IP addresses):

```bash
# 1. Update configuration
nano config/cluster-config.yaml

# 2. Update network settings
./scripts/update-network.sh

# 3. Restart pods
kubectl rollout restart deployment -n barns
```

**No need to reinstall Kubernetes!**

## 🔄 Multi-Branch Deployment

To deploy multiple BARNS instances for different branches:

```bash
# Clone deployment package
cp -r k8s-deployment k8s-deployment-branch-dev
cd k8s-deployment-branch-dev

# Update configuration
sed -i 's/namespace: barns/namespace: barns-dev/' config/cluster-config.yaml

# Deploy with different namespace
./scripts/deploy-k8s.sh --namespace barns-dev
```

## 📊 Service Ports

| Service | Internal Port | NodePort | Description |
|---------|--------------|----------|-------------|
| Dashboard | 80 | 30003 | Web UI |
| API Bridge | 8080 | 30000 | REST API |
| Video Stream | 8001 | 30001 | Camera streams |
| OMS Service | 8000 | 30002 | Order Management |
| RabbitMQ Mgmt | 15672 | 30672 | Message Queue |
| InfluxDB | 8086 | 30086 | Time Series DB |

## 🛠️ Maintenance

### Update Code

```bash
# Pull latest changes
cd ~/BARNS
git pull

# Rebuild images
cd ~/k8s-deployment/scripts
./build-images.sh

# Update deployment
./deploy-k8s.sh --update
```

### Restart Services

```bash
# Restart all services
kubectl rollout restart deployment -n barns

# Restart specific service
kubectl rollout restart deployment validation-service -n barns
```

### Backup Database

```bash
./scripts/backup-database.sh
```

### View Logs

```bash
# All pods
kubectl logs -l app=validation-service -n barns --tail=100

# Specific pod
kubectl logs validation-service-xxxxx -n barns --follow
```

## 🔍 Troubleshooting

See [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for common issues and solutions.

### Quick Diagnostics

```bash
# Run validation
./scripts/validate.sh

# Check pod status
kubectl get pods -n barns

# Check pod events
kubectl describe pod <pod-name> -n barns

# Check logs
kubectl logs <pod-name> -n barns
```

## 📚 Documentation

- [QUICKSTART.md](QUICKSTART.md) - Quick start guide
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) - Common issues
- [docs/NETWORKING.md](docs/NETWORKING.md) - Network configuration
- [docs/MAINTENANCE.md](docs/MAINTENANCE.md) - Maintenance procedures

## 🔐 Security Notes

**Default Credentials (Change in Production):**
- RabbitMQ: `guest` / `guest`
- PostgreSQL: See `config/cluster-config.yaml`

**Recommendations:**
1. Change all default passwords
2. Enable network policies
3. Use TLS for external services
4. Implement RBAC
5. Regular security updates

## 🤝 Support

For issues or questions:
1. Check [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)
2. Run `./scripts/validate.sh`
3. Review pod logs
4. Check GitHub issues

## 📝 License

Copyright © 2024 BARNS Project

---

**Made with ☕ by the BARNS Team**

