# Accessing BARNS UI in Kubernetes

## Dashboard Access

The dashboard is accessible via **NodePort 30003**.

### Access Methods

#### Option 1: Using Node IP (Recommended)

```bash
# Get the node IP
kubectl get nodes -o wide

# Access dashboard
http://<NODE-IP>:30003
```

For example, if your node IP is `192.168.8.101`:
```
http://192.168.8.101:30003
```

#### Option 2: Using Port Forward (Alternative)

If you can't access via NodePort, use port forwarding:

```bash
# Forward local port 3000 to dashboard service
kubectl port-forward -n barns service/dashboard 3000:80

# Then access at:
http://localhost:3000
```

## Other Services

### API Bridge
- **NodePort**: 30000
- **URL**: `http://<NODE-IP>:30000`

### Video Stream Service
- **NodePort**: 30001
- **URL**: `http://<NODE-IP>:30001`

### OMS Service
- **NodePort**: 30002
- **URL**: `http://<NODE-IP>:30002`

### RabbitMQ Management
- **NodePort**: 30672
- **URL**: `http://<NODE-IP>:30672`
- **Credentials**: admin/admin123

### InfluxDB
- **NodePort**: 30086
- **URL**: `http://<NODE-IP>:30086`

## Quick Access Commands

```bash
# Get all NodePort services
kubectl get svc -n barns | grep NodePort

# Get node IPs
kubectl get nodes -o wide

# Port forward dashboard
kubectl port-forward -n barns service/dashboard 3000:80
```

