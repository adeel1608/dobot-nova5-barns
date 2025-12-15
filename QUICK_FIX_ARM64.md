# Quick Fix: ARM64 Image Build for Kubernetes

## The Problem

Your pods are crashing with `exec format error` because:
- Images were built for x86_64/amd64
- Your Kubernetes nodes are ARM64 (aarch64)
- Pods run on `barns-nuc15crsu7` (worker node)
- Images need to be on the worker node where pods run

## Quick Solution

### Step 1: Build Images on Worker Node

SSH to the worker node where pods run:

```bash
ssh user@barns-nuc15crsu7
# or
ssh user@192.168.8.101
```

Then on the worker node:

```bash
cd ~/BARNS  # or wherever your BARNS code is

# Make build script executable
chmod +x build-images-arm64.sh

# Build all images
./build-images-arm64.sh
```

### Step 2: Restart Pods

After building, go back to control plane and restart deployments:

```bash
# Restart all deployments
kubectl rollout restart deployment -n barns

# Watch pods come up
kubectl get pods -n barns -w
```

### Step 3: Verify

Check that pods are running:

```bash
kubectl get pods -n barns
```

Check logs to confirm no more exec format errors:

```bash
kubectl logs api-bridge-<pod-id> -n barns
```

## Alternative: Build Locally and Copy

If you can't build on worker node, build locally and copy:

```bash
# On control plane (ubuntu)
./build-images-arm64.sh

# Copy to worker node (if using Docker on worker)
docker save barns-api-bridge:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-validation:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-automation:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-routine:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-robot-arm:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-scheduler:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-oms:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-video-stream:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-dashboard:latest | ssh user@barns-nuc15crsu7 'docker load'
```

If worker node uses containerd (not Docker):

```bash
# Copy using containerd
docker save barns-api-bridge:latest | ssh user@barns-nuc15crsu7 'ctr -n k8s.io images import -'
# ... repeat for all images
```

## Verify Image Architecture

After building, verify images are ARM64:

```bash
# On worker node
docker inspect barns-api-bridge:latest | grep Architecture
# Should show: "Architecture": "arm64"
```

## Troubleshooting

### Can't SSH to worker node

1. Check if you have network access: `ping barns-nuc15crsu7`
2. Check SSH access: `ssh user@barns-nuc15crsu7`
3. If no SSH, you may need to use a container registry

### Images still not working

1. Verify image is ARM64: `docker inspect <image> | grep Architecture`
2. Check if image exists on worker: `docker images | grep barns-`
3. Force delete pod to force image pull: `kubectl delete pod <pod-name> -n barns`
4. Check pod events: `kubectl describe pod <pod-name> -n barns`

### Using Container Registry

If you can't access worker node directly, use a registry:

```bash
# Build and tag
docker buildx build --platform linux/arm64 -t your-registry/barns-api-bridge:latest -f services/api-bridge/Dockerfile --push .

# Update deployment to pull from registry
kubectl set image deployment/api-bridge api-bridge=your-registry/barns-api-bridge:latest -n barns
```

