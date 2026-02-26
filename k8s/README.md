# BARNS Kubernetes Deployment Guide

This directory contains Kubernetes manifests for deploying the BARNS (Business Automation & Robotics Network System) to a Kubernetes cluster.

## Overview

The BARNS system has been converted from Docker Compose to Kubernetes with the following architecture:

- **Infrastructure Services**: RabbitMQ, Redis, PostgreSQL, InfluxDB, Telegraf
- **Application Services**: Validation, Automation, Routine, Robot Arm, Scheduler, OMS
- **Frontend Services**: API Bridge, Video Stream, Dashboard

## Directory Structure

```
k8s/
├── namespace.yaml              # Barns namespace definition
├── secrets.yaml                # All secrets (passwords, tokens)
├── configmaps/                 # Configuration files
│   ├── telegraf-config.yaml
│   ├── routine-tasks-config.yaml
│   └── postgres-init-configmap.yaml
├── storage/                    # Persistent storage (PV & PVC)
│   ├── rabbitmq-pv-pvc.yaml
│   ├── redis-pv-pvc.yaml
│   ├── postgres-pv-pvc.yaml
│   └── influxdb-pv-pvc.yaml
├── infrastructure/             # Infrastructure services
│   ├── rabbitmq.yaml
│   ├── redis.yaml
│   ├── postgres.yaml
│   ├── influxdb.yaml
│   └── telegraf.yaml
├── services/                   # Application services
│   ├── validation-service.yaml
│   ├── automation-service.yaml
│   ├── routine-service.yaml
│   ├── robot-arm-service.yaml
│   ├── scheduler-service.yaml
│   └── oms-service.yaml
├── frontend/                   # Frontend services
│   ├── api-bridge.yaml
│   ├── video-stream-service.yaml
│   └── dashboard.yaml
├── deploy.sh                   # Automated deployment script
└── README.md                   # This file
```

## Prerequisites

1. **Kubernetes Cluster**: Running cluster (v1.24+)
2. **kubectl**: Configured to access your cluster
3. **Storage**: Local storage available at `/mnt/ssd/barns-data/` on worker nodes
4. **Docker Images**: Built and tagged BARNS service images
5. **Bash shell** (recommended): `deploy.sh` is a bash script (use Linux/macOS, WSL, or Git Bash on Windows)
6. **gcloud CLI** (only if pulling from Google Artifact Registry): Used by `deploy.sh` to create the `gcr-json-key` imagePullSecret
7. **Repo files referenced by ConfigMaps**:
   - `config/tasks.json` (mounted into `routine-service`)
   - `data/` directory (mounted into `scheduler-service` via `scheduler-data` ConfigMap)

## Pre-Deployment Setup

### 0. Review Site-Specific Settings (Required)

Several manifests are **environment/hardware specific** and should be reviewed before deploying:

- **Node pinning (`nodeSelector`)**:
  - `services/validation-service.yaml` (camera/RTSP access via `hostNetwork: true`)
  - `services/robot1-deployment.yaml`, `services/robot2-deployment.yaml` (robot hardware access)
- **Hardware/network assumptions**:
  - `robot1` / `robot2` include static `IP_ADDRESS` values and camera serial numbers.
  - `video-stream-service` mounts host `/dev` and runs privileged.
- **Security posture**:
  - `robot1` / `robot2` use `privileged: true` plus `hostPID`/`hostIPC`/`hostNetwork`.
  - `validation-service` uses `hostNetwork: true` and hostPath volumes under `/mnt/ssd/barns-data/`.

If these do not match your environment, update them before applying manifests.

### 1. Create Storage Directories on Nodes

On each Kubernetes node where services will run, create the required directories:

```bash
# On each node
sudo mkdir -p /mnt/ssd/barns-data/rabbitmq
sudo mkdir -p /mnt/ssd/barns-data/redis
sudo mkdir -p /mnt/ssd/barns-data/postgres
sudo mkdir -p /mnt/ssd/barns-data/influxdb
sudo mkdir -p /mnt/ssd/barns-data/debug_frames_coffee
sudo mkdir -p /mnt/ssd/barns-data/debug_frames_cup
sudo mkdir -p /mnt/ssd/barns-data/cup_models
sudo mkdir -p /mnt/barns-config
sudo chmod -R 777 /mnt/ssd/barns-data
sudo chmod -R 755 /mnt/barns-config
```

### 2. Copy PostgreSQL Schema Files

Copy the schema files to the config directory on the node where PostgreSQL will run:

```bash
# On the PostgreSQL node
sudo cp services/oms/schema.sql /mnt/barns-config/oms-schema.sql
sudo cp services/validation/validation_schema.sql /mnt/barns-config/validation-schema.sql
```

### 3. Build Docker Images

Build all BARNS service images using the appropriate Dockerfiles:

```bash
# From the BARNS root directory
cd services/validation
docker build -t barns-validation:latest -f Dockerfile.rabbitmq .

cd ../automation
docker build -t barns-automation:latest -f Dockerfile.rabbitmq .

cd ../routine
docker build -t barns-routine:latest -f Dockerfile.rabbitmq .

cd ../robot_arm
docker build -t barns-robot-arm:latest -f Dockerfile.rabbitmq .

cd ../scheduler
docker build -t barns-scheduler:latest -f Dockerfile.rabbitmq .

cd ../oms
docker build -t barns-oms:latest -f Dockerfile.rabbitmq .

cd ../api-bridge
docker build -t barns-api-bridge:latest -f Dockerfile .

cd ../video-stream
docker build -t barns-video-stream:latest -f Dockerfile .

cd ../barns-dashboard
docker build -t barns-dashboard:latest -f Dockerfile.rabbitmq .
```

**Note**: If using a container registry, tag and push images:

```bash
# Example for Docker Hub
docker tag barns-validation:latest your-registry/barns-validation:latest
docker push your-registry/barns-validation:latest
# Repeat for all images
```

Then update the `image:` field in each YAML file to reference your registry.

## Deployment

### Quick Deployment

Use the automated deployment script:

```bash
cd k8s
chmod +x deploy.sh
./deploy.sh
```

**Note**: `deploy.sh` creates/updates the `gcr-json-key` imagePullSecret (Google Artifact Registry). If you are not using GAR, remove `imagePullSecrets` from the manifests or create an equivalent secret for your registry.

### Manual Deployment

Deploy components in order:

```bash
# 1. Create namespace
kubectl apply -f namespace.yaml

# 2. Create secrets
kubectl apply -f secrets.yaml

# 3. Create ConfigMaps
kubectl apply -f configmaps/

# 4. Create storage
kubectl apply -f storage/

# 5. Deploy infrastructure
kubectl apply -f infrastructure/

# Wait for infrastructure to be ready
kubectl wait --for=condition=ready pod -l app=rabbitmq -n barns --timeout=120s
kubectl wait --for=condition=ready pod -l app=postgres -n barns --timeout=120s
kubectl wait --for=condition=ready pod -l app=redis -n barns --timeout=120s
kubectl wait --for=condition=ready pod -l app=influxdb -n barns --timeout=120s

# 6. Deploy services
kubectl apply -f services/

# 7. Deploy frontend
kubectl apply -f frontend/
```

## Verification

Check deployment status:

```bash
# Check all resources
kubectl get all -n barns

# Check pods status
kubectl get pods -n barns

# Check services
kubectl get svc -n barns

# Check persistent volumes
kubectl get pv,pvc -n barns
```

Check logs for any service:

```bash
kubectl logs -n barns deployment/rabbitmq
kubectl logs -n barns deployment/validation-service
kubectl logs -n barns deployment/api-bridge
```

## Accessing Services

### NodePort Services

External access is provided through NodePort services. Replace `<NODE-IP>` with your Kubernetes node IP:

- **RabbitMQ Management**: http://\<NODE-IP\>:30672 (admin/admin123)
- **RabbitMQ MQTT**: `<NODE-IP>:30673` (TCP 1883)
- **InfluxDB**: http://\<NODE-IP\>:30086
- **API Bridge**: http://\<NODE-IP\>:30000
- **Video Stream**: http://\<NODE-IP\>:30001
- **OMS Service**: http://\<NODE-IP\>:30002
- **Dashboard**: http://\<NODE-IP\>:30003

### Internal Services (ClusterIP)

Services communicate internally using Kubernetes DNS:

- **Infrastructure**
  - `rabbitmq.barns.svc.cluster.local:5672` (AMQP)
  - `rabbitmq.barns.svc.cluster.local:1883` (MQTT)
  - `rabbitmq.barns.svc.cluster.local:15672` (management UI/API)
  - `postgres.barns.svc.cluster.local:5432`
  - `redis.barns.svc.cluster.local:6379`
  - `influxdb.barns.svc.cluster.local:8086` (InfluxDB v2; API base path `/api/v2`)
  - `telegraf.barns.svc.cluster.local:8094` (UDP)
- **HTTP services**
  - `api-bridge.barns.svc.cluster.local:8000`
  - `oms-service.barns.svc.cluster.local:8000`
  - `automation-service.barns.svc.cluster.local:8080`
  - `routine-service.barns.svc.cluster.local:8080`
  - `robot-arm-service.barns.svc.cluster.local:8080`
  - `scheduler-service.barns.svc.cluster.local:8080`
  - `video-stream-service.barns.svc.cluster.local:8000` (NodePort service also has a ClusterIP)
  - `dashboard.barns.svc.cluster.local:80` (NodePort service also has a ClusterIP)

## API Surface (Inbound and Outbound)

This section documents the **network-level API surface** implied by the Kubernetes manifests (Services, NodePorts, and environment variables). For detailed HTTP route lists per service, refer to each service’s own README/source.

### Inbound (external-to-cluster)

All inbound traffic is via **NodePort** (no Ingress manifests are included in `k8s/`):

| Component | Protocol | External (NodePort) | In-cluster target |
|---|---:|---|---|
| RabbitMQ Management UI/API | HTTP | `http://<NODE-IP>:30672` | `rabbitmq:15672` |
| RabbitMQ MQTT | TCP | `<NODE-IP>:30673` | `rabbitmq:1883` |
| InfluxDB UI/API | HTTP | `http://<NODE-IP>:30086` | `influxdb:8086` |
| API Bridge | HTTP | `http://<NODE-IP>:30000` | `api-bridge:8000` |
| Video Stream | HTTP | `http://<NODE-IP>:30001` | `video-stream-service:8000` |
| OMS Service | HTTP | `http://<NODE-IP>:30002` | `oms-service:8000` |
| Dashboard | HTTP | `http://<NODE-IP>:30003` | `dashboard:80` |

### Inbound (cluster-internal)

| Component | Protocol | Address |
|---|---:|---|
| RabbitMQ AMQP | TCP | `rabbitmq:5672` |
| RabbitMQ MQTT | TCP | `rabbitmq:1883` |
| PostgreSQL | TCP | `postgres:5432` |
| Redis | TCP | `redis:6379` |
| InfluxDB | HTTP | `http://influxdb:8086` |
| Telegraf StatsD | UDP | `telegraf:8094` |

| Service | Protocol | Address |
|---|---:|---|
| API Bridge | HTTP | `http://api-bridge:8000` |
| OMS Service | HTTP | `http://oms-service:8000` |
| Automation Service | HTTP | `http://automation-service:8080` |
| Routine Service | HTTP | `http://routine-service:8080` |
| Robot Arm Service | HTTP | `http://robot-arm-service:8080` |
| Scheduler Service | HTTP | `http://scheduler-service:8080` |
| Video Stream | HTTP | `http://video-stream-service:8000` |
| Dashboard | HTTP | `http://dashboard:80` |

### Outbound (service-to-service dependencies)

Below is what each workload is configured to call/produce based on its environment variables and runtime mode.

| Workload | Outbound APIs / dependencies |
|---|---|
| `api-bridge` | RabbitMQ AMQP (`RABBITMQ_URL`), Telegraf StatsD (`telegraf:8094/UDP`), HTTP health check at `/health` (used by kubelet liveness probe) |
| `oms-service` | RabbitMQ AMQP, Redis (`redis:6379`), PostgreSQL (`postgres:5432`), Telegraf StatsD (`telegraf:8094/UDP`) |
| `automation-service` | RabbitMQ AMQP, Telegraf StatsD (`telegraf:8094/UDP`) |
| `routine-service` | RabbitMQ AMQP, Telegraf StatsD (`telegraf:8094/UDP`), reads routine tasks from ConfigMap mount (`/app/config/tasks.json`) |
| `robot-arm-service` | RabbitMQ AMQP, Telegraf StatsD (`telegraf:8094/UDP`) |
| `scheduler-service` | RabbitMQ AMQP, Telegraf StatsD (`telegraf:8094/UDP`), reads scheduler data from ConfigMap (`scheduler-data`) mounted at `/app/data` |
| `validation-service` | RabbitMQ AMQP, PostgreSQL (`postgres:5432`), writes debug frames/models to hostPath mounts under `/mnt/ssd/barns-data/`, **uses `hostNetwork: true`** (RTSP camera access), sends metrics to Telegraf using `localhost:8094/UDP` due to host networking |
| `video-stream-service` | Telegraf StatsD (`telegraf:8094/UDP`), **runs privileged** with `hostPath: /dev` (device access) |
| `dashboard` | Configured with `RABBITMQ_URL`, `API_BRIDGE_URL`, `VIDEO_STREAM_URL`, and InfluxDB v2 settings (`VITE_INFLUX_*`). Review the `localhost`-based URLs in `k8s/frontend/dashboard.yaml`: when served over NodePort, `localhost` resolves on the end user’s machine (browser), not the cluster node. Prefer using the same `<NODE-IP>:<NodePort>` origin or relative URLs if the frontend expects to call these from the browser. |
| `robot1`, `robot2` | **Privileged + hostNetwork/hostPID/hostIPC**. Connects to RabbitMQ (`rabbitmq.barns.svc.cluster.local:5672`) and to physical robot controllers/cameras (static `IP_ADDRESS` in the manifests). These deployments are hardware/environment specific (nodeSelector hostnames, device mounts under `/dev`, etc.). |

### Port-forward (alternative to NodePort)

If you do not want to expose NodePorts, you can access services via `kubectl port-forward`:

```bash
kubectl -n barns port-forward svc/api-bridge 8000:8000
kubectl -n barns port-forward svc/oms-service 8000:8000
kubectl -n barns port-forward svc/influxdb 8086:8086
kubectl -n barns port-forward svc/rabbitmq 15672:15672
```

## Configuration

### Secrets

All passwords and tokens are stored in `secrets.yaml`. To update:

```bash
kubectl edit secret -n barns rabbitmq-secret
kubectl edit secret -n barns postgres-secret
kubectl edit secret -n barns influxdb-secret
```

### Environment Variables

Service-specific environment variables are defined in each deployment YAML:

- **Validation Service**: `services/validation-service.yaml`
- **OMS Service**: `services/oms-service.yaml`
- etc.

### Scaling

Scale any service:

```bash
# Scale validation service to 2 replicas
kubectl scale deployment validation-service -n barns --replicas=2

# Scale API bridge
kubectl scale deployment api-bridge -n barns --replicas=3
```

**Note**: StatefulSets (RabbitMQ, PostgreSQL, InfluxDB) should remain at 1 replica for data consistency.

## Troubleshooting

### Pod Not Starting

Check pod events and logs:

```bash
kubectl describe pod -n barns <pod-name>
kubectl logs -n barns <pod-name>
```

### Storage Issues

Check PV and PVC status:

```bash
kubectl get pv
kubectl get pvc -n barns
kubectl describe pvc -n barns <pvc-name>
```

Ensure storage directories exist on nodes and have correct permissions.

### Network Issues

Check service endpoints:

```bash
kubectl get endpoints -n barns
kubectl describe svc -n barns <service-name>
```

### Image Pull Errors

If using a private registry:

```bash
# Create image pull secret
kubectl create secret docker-registry regcred \
  --docker-server=<registry> \
  --docker-username=<username> \
  --docker-password=<password> \
  -n barns

# Add to deployment
# imagePullSecrets:
#   - name: regcred
```

### Database Initialization

If PostgreSQL schemas aren't initialized:

```bash
# Get PostgreSQL pod name
POD=$(kubectl get pod -n barns -l app=postgres -o jsonpath='{.items[0].metadata.name}')

# Execute SQL manually
kubectl exec -it -n barns $POD -- psql -U postgres -f /docker-entrypoint-initdb.d/01-init.sql
kubectl exec -it -n barns $POD -- psql -U postgres -d barns_oms -f /docker-entrypoint-initdb.d/02-oms-schema.sql
kubectl exec -it -n barns $POD -- psql -U postgres -d barns_validation -f /docker-entrypoint-initdb.d/03-validation-schema.sql
```

## Maintenance

### Backup

Backup persistent data:

```bash
# Backup PostgreSQL
kubectl exec -n barns deployment/postgres -- pg_dumpall -U postgres > backup.sql

# Backup volumes (from node)
sudo tar -czf barns-data-backup.tar.gz /mnt/ssd/barns-data/
```

### Updates

Update service image:

```bash
kubectl set image deployment/validation-service validation-service=barns-validation:v2 -n barns
```

### Restart Service

```bash
kubectl rollout restart deployment/validation-service -n barns
```

### Delete Deployment

```bash
# Delete all BARNS resources
kubectl delete namespace barns

# Or delete selectively
kubectl delete -f frontend/
kubectl delete -f services/
kubectl delete -f infrastructure/
kubectl delete -f storage/
```

## Key Differences from Docker Compose

1. **Networking**: Services use Kubernetes DNS instead of Docker network names
2. **Storage**: PersistentVolumes instead of Docker volumes
3. **Secrets**: Kubernetes Secrets instead of environment variables
4. **Service Discovery**: Automatic via Kubernetes services
5. **Validation Service**: Uses `hostNetwork: true` for RTSP camera access
6. **Video Stream**: Uses privileged mode for device access
7. **Robotics Pods** (`robot1`, `robot2`): Use privileged + host networking and are node/hardware specific (update `nodeSelector` and device/IP env vars per site)

## Security Considerations

1. **Change default passwords** in `secrets.yaml` before production deployment
2. **Use TLS/SSL** for external services (configure ingress)
3. **Network Policies**: Implement network policies to restrict pod-to-pod communication
4. **RBAC**: Configure Role-Based Access Control
5. **Image Security**: Scan images for vulnerabilities before deployment

## Production Recommendations

1. **Use an Ingress Controller** (e.g., NGINX Ingress) instead of NodePort
2. **Configure resource limits** for each service
3. **Set up monitoring** (Prometheus + Grafana)
4. **Configure log aggregation** (ELK stack or Loki)
5. **Use a proper StorageClass** (e.g., Ceph, NFS) instead of hostPath
6. **Implement backup automation**
7. **Set up High Availability** for critical services

## Support

For issues or questions:
- Check pod logs: `kubectl logs -n barns <pod-name>`
- Check events: `kubectl get events -n barns --sort-by='.lastTimestamp'`
- Describe resources: `kubectl describe pod -n barns <pod-name>`

## License

Same as main BARNS project.






