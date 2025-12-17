# BARNS Kubernetes Deployment - File Index

Quick reference to all files in this deployment package.

## 📁 Root Files

| File | Description |
|------|-------------|
| `README.md` | Main documentation and overview |
| `QUICKSTART.md` | Quick start guide (5 steps to deployment) |
| `INDEX.md` | This file - complete file index |
| `deploy.sh` | Main deployment script with menu |

## 📂 config/

| File | Description |
|------|-------------|
| `cluster-config.yaml` | **Main configuration file** - Edit this first! |

## 📂 scripts/

| Script | Purpose | Run On | Requires Root |
|--------|---------|--------|---------------|
| `common.sh` | Shared functions (sourced by other scripts) | Any | No |
| `setup-master.sh` | Setup Kubernetes master node | Master | Yes |
| `setup-worker.sh` | Setup Kubernetes worker node | Worker | Yes |
| `build-images.sh` | Build Docker images for all services | Master | No |
| `deploy-k8s.sh` | Deploy BARNS to Kubernetes | Master | No |
| `fix-database.sh` | Initialize and fix database | Master | No |
| `validate.sh` | Validate deployment health | Master | No |
| `cleanup.sh` | Remove BARNS deployment | Master | No |

## 📂 docs/

| Document | Content |
|----------|---------|
| `TROUBLESHOOTING.md` | Common issues and solutions |
| `NETWORKING.md` | Network configuration guide *(to be created)* |
| `MAINTENANCE.md` | Maintenance procedures *(to be created)* |

## 📂 templates/

*(Reserved for Kubernetes YAML templates - to be added if needed)*

## 🚀 Quick Start Flow

```
1. Edit config/cluster-config.yaml
   └─> Set your node IPs and credentials

2. Run on Master: sudo scripts/setup-master.sh
   └─> Sets up Kubernetes master node

3. Run on Worker: sudo scripts/setup-worker.sh
   └─> Sets up Kubernetes worker node

4. Run on Master: scripts/build-images.sh
   └─> Builds Docker images on worker via SSH

5. Run on Master: scripts/deploy-k8s.sh
   └─> Deploys BARNS to Kubernetes

6. Run on Master: scripts/fix-database.sh
   └─> Initializes database

7. Run on Master: scripts/validate.sh
   └─> Validates deployment
```

## 📝 Script Details

### setup-master.sh
**Purpose:** Automates Kubernetes master node setup
**Features:**
- Installs Kubernetes components
- Initializes cluster
- Installs CNI plugin (Flannel)
- Generates join command for worker
- Network-agnostic (handles IP changes)

**Usage:**
```bash
sudo ./scripts/setup-master.sh
```

### setup-worker.sh
**Purpose:** Automates Kubernetes worker node setup
**Features:**
- Installs Docker and Kubernetes
- Creates storage directories
- Joins cluster (if join command available)
- Configures dynamic node IP

**Usage:**
```bash
sudo ./scripts/setup-worker.sh
```

### build-images.sh
**Purpose:** Builds all BARNS Docker images on worker node
**Features:**
- Builds for correct architecture (AMD64 on worker)
- Cleans old images
- Imports to containerd automatically
- SSH-based remote execution

**Usage:**
```bash
./scripts/build-images.sh
```

**Services Built:**
- api-bridge
- validation
- automation
- routine
- robot-arm
- scheduler
- oms
- video-stream
- dashboard

### deploy-k8s.sh
**Purpose:** Deploys BARNS to Kubernetes
**Features:**
- Creates namespace
- Applies secrets and configmaps
- Creates storage
- Deploys infrastructure (Postgres, RabbitMQ, Redis, InfluxDB)
- Deploys application services
- Waits for pods to be ready

**Usage:**
```bash
./scripts/deploy-k8s.sh
```

### fix-database.sh
**Purpose:** Initializes database and fixes inventory
**Features:**
- Waits for PostgreSQL to be ready
- Fixes inventory subtypes to match `inventory_rules.json`
- Verifies database integrity
- Restarts validation service

**Usage:**
```bash
./scripts/fix-database.sh
```

**What it fixes:**
- Adds `cup_` prefix to cup types (H7 → cup_H7)
- Adds `_milk` suffix to milk types
- Adds `_syrup` suffix to syrups
- Moves sauces to syrups category
- Fixes sauce names (white_chocolate_sauce, etc.)

### validate.sh
**Purpose:** Comprehensive deployment validation
**Features:**
- Checks all resources (nodes, pods, services)
- Tests database connectivity
- Validates infrastructure services
- Tests external access
- Provides detailed status report

**Usage:**
```bash
./scripts/validate.sh
```

**Tests Performed:**
1. Namespace existence
2. Node status
3. Pod health
4. Service accessibility
5. Storage (PV/PVC) status
6. Database connectivity
7. RabbitMQ status
8. Redis status
9. External endpoint access
10. Recent error logs

### cleanup.sh
**Purpose:** Removes BARNS deployment
**Features:**
- Deletes all Kubernetes resources
- Optional data deletion
- Can preserve namespace
- Can clean worker node storage

**Usage:**
```bash
./scripts/cleanup.sh
```

**Warning:** This is destructive! Always backs up if needed.

## 🔧 Configuration File

### cluster-config.yaml

**Key Sections:**

#### Cluster Settings
```yaml
cluster:
  name: barns-cluster
  namespace: barns
```

#### Node Configuration
```yaml
master:
  ip: auto  # or specific IP
  ssh_user: plus
  architecture: arm64

worker:
  ip: auto  # or specific IP
  ssh_user: barns
  architecture: amd64
```

#### Network Settings
```yaml
network:
  pod_cidr: 10.244.0.0/16
  service_cidr: 10.96.0.0/12
  cni: flannel
  auto_detect_ips: true
```

#### Service Ports
```yaml
services:
  nodeports:
    dashboard: 30003
    api_bridge: 30000
    video_stream: 30001
    oms: 30002
```

## 🎯 Common Use Cases

### First Time Deployment
```bash
# 1. Configure
nano config/cluster-config.yaml

# 2. Setup nodes (on each node)
sudo scripts/setup-master.sh   # on master
sudo scripts/setup-worker.sh   # on worker

# 3. Deploy (from master)
scripts/build-images.sh
scripts/deploy-k8s.sh
scripts/fix-database.sh
scripts/validate.sh
```

### Update Code & Redeploy
```bash
# On worker: pull latest code
cd ~/git-BARNS/BARNS
git pull

# On master: rebuild and update
cd k8s-deployment
scripts/build-images.sh
kubectl rollout restart deployment -n barns
```

### Network Changed
```bash
# On master node
sudo /usr/local/bin/k8s-update-network.sh

# Update config
nano config/cluster-config.yaml

# Restart pods
kubectl rollout restart deployment -n barns
```

### Troubleshooting Issues
```bash
# Run validation
scripts/validate.sh

# Check specific issues in docs
less docs/TROUBLESHOOTING.md

# Fix database if needed
scripts/fix-database.sh
```

### Complete Cleanup
```bash
scripts/cleanup.sh
# Follow prompts to delete everything
```

## 📊 Service Architecture

```
Master Node (ARM64)
├── Kubernetes Control Plane
├── kubectl
└── Deployment Scripts

Worker Node (AMD64)
├── Docker Engine
├── containerd
├── Pod Network (Flannel)
└── BARNS Pods:
    ├── Infrastructure:
    │   ├── postgres-0 (StatefulSet)
    │   ├── rabbitmq-0 (StatefulSet)
    │   ├── influxdb-0 (StatefulSet)
    │   ├── redis
    │   └── telegraf
    └── Services:
        ├── api-bridge
        ├── validation-service
        ├── automation-service
        ├── routine-service
        ├── robot-arm-service
        ├── scheduler-service
        ├── oms-service
        ├── video-stream-service
        └── dashboard
```

## 🔐 Security Notes

**Default Credentials** (Change in production!):
- RabbitMQ: `guest` / `guest`
- PostgreSQL: See `cluster-config.yaml`

**Recommendations:**
1. Update all passwords in `config/cluster-config.yaml`
2. Generate secrets: `./scripts/generate-secrets.sh` *(to be created)*
3. Enable RBAC policies
4. Use TLS for external services
5. Implement network policies

## 📈 Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | Dec 2024 | Initial release with automated deployment |

## 🆘 Support

**Documentation:**
- `README.md` - Overview
- `QUICKSTART.md` - Quick start
- `docs/TROUBLESHOOTING.md` - Common issues

**Commands:**
```bash
# Validate deployment
./scripts/validate.sh

# View logs
kubectl logs <pod-name> -n barns

# Get status
kubectl get pods -n barns
kubectl get events -n barns
```

## 📝 License

Copyright © 2024 BARNS Project

---

**Last Updated:** December 2024
**Package Version:** 1.0.0

