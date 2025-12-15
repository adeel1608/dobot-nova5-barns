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
3. **Storage**: Local storage available at `/mnt/barns-data/` on worker nodes
4. **Docker Images**: Built and tagged BARNS service images

## Pre-Deployment Setup

### 1. Create Storage Directories on Nodes

On each Kubernetes node where services will run, create the required directories:

```bash
# On each node
sudo mkdir -p /mnt/barns-data/rabbitmq
sudo mkdir -p /mnt/barns-data/redis
sudo mkdir -p /mnt/barns-data/postgres
sudo mkdir -p /mnt/barns-data/influxdb
sudo mkdir -p /mnt/barns-data/debug_frames_coffee
sudo mkdir -p /mnt/barns-data/debug_frames_cup
sudo mkdir -p /mnt/barns-data/cup_models
sudo mkdir -p /mnt/barns-config
sudo chmod -R 777 /mnt/barns-data
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
- **InfluxDB**: http://\<NODE-IP\>:30086
- **API Bridge**: http://\<NODE-IP\>:30000
- **Video Stream**: http://\<NODE-IP\>:30001
- **OMS Service**: http://\<NODE-IP\>:30002
- **Dashboard**: http://\<NODE-IP\>:30003

### Internal Services (ClusterIP)

Services communicate internally using Kubernetes DNS:

- `rabbitmq.barns.svc.cluster.local:5672`
- `postgres.barns.svc.cluster.local:5432`
- `redis.barns.svc.cluster.local:6379`
- `telegraf.barns.svc.cluster.local:8094`

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
sudo tar -czf barns-data-backup.tar.gz /mnt/barns-data/
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






