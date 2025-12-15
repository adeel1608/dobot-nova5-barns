# Building BARNS Images for ARM64

This guide explains how to build BARNS Docker images for ARM64 architecture (aarch64).

## Problem

If you see the error `exec /usr/local/bin/python: exec format error` in your Kubernetes pods, it means the Docker images were built for a different architecture (likely x86_64/amd64) than your Kubernetes nodes (ARM64/aarch64).

## Solution

Rebuild all images for ARM64 architecture using `docker buildx`.

## Quick Build (Single Service)

To build a single service quickly, run from the BARNS root directory:

```bash
# Example: Build api-bridge
docker buildx build \
  --platform linux/arm64 \
  -t barns-api-bridge:latest \
  -f services/api-bridge/Dockerfile \
  --load \
  .
```

## Automated Build (All Services)

Use the provided build script to build all services:

```bash
# Make script executable (if not already)
chmod +x build-images-arm64.sh

# Run the build script
./build-images-arm64.sh
```

The script will:
1. Check for docker and docker buildx
2. Create a buildx builder if needed
3. Build all BARNS service images for ARM64
4. Tag them with `:latest`

## Services Built

The script builds the following images:
- `barns-api-bridge:latest`
- `barns-validation:latest`
- `barns-automation:latest`
- `barns-routine:latest`
- `barns-robot-arm:latest`
- `barns-scheduler:latest`
- `barns-oms:latest`
- `barns-video-stream:latest`
- `barns-dashboard:latest`

## After Building

After building the images, restart your Kubernetes deployments to use the new ARM64 images:

```bash
# Restart all deployments
kubectl rollout restart deployment/api-bridge -n barns
kubectl rollout restart deployment/automation-service -n barns
kubectl rollout restart deployment/dashboard -n barns
kubectl rollout restart deployment/oms-service -n barns
kubectl rollout restart deployment/robot-arm-service -n barns
kubectl rollout restart deployment/routine-service -n barns
kubectl rollout restart deployment/scheduler-service -n barns
kubectl rollout restart deployment/validation-service -n barns
kubectl rollout restart deployment/video-stream-service -n barns
```

Or restart all at once:

```bash
kubectl rollout restart deployment -n barns
```

## Verify Build

Check that images are built for ARM64:

```bash
docker inspect barns-api-bridge:latest | grep Architecture
```

Should show: `"Architecture": "arm64"`

## Important: Images Must Be on Worker Node

Since your pods run on `barns-nuc15crsu7` (worker node) and Kubernetes uses `imagePullPolicy: IfNotPresent`, the images must be available on that node.

### Option 1: Build on Worker Node (Recommended)

SSH to the worker node and build there:

```bash
# SSH to worker node
ssh user@barns-nuc15crsu7

# Copy BARNS directory or clone it
cd ~/BARNS

# Run build script
chmod +x build-images-arm64.sh
./build-images-arm64.sh
```

### Option 2: Build Locally and Copy

Build on control plane, then copy to worker node:

```bash
# Build locally
./build-images-arm64.sh

# Copy each image to worker node
docker save barns-api-bridge:latest | ssh user@barns-nuc15crsu7 'docker load'
docker save barns-validation:latest | ssh user@barns-nuc15crsu7 'docker load'
# ... repeat for all images
```

### Option 3: Use Containerd (if worker uses containerd)

If the worker node uses containerd instead of Docker:

```bash
# Build locally
./build-images-arm64.sh

# Copy to worker node using containerd
docker save barns-api-bridge:latest | ssh user@barns-nuc15crsu7 'ctr -n k8s.io images import -'
# ... repeat for all images
```

### Option 4: Use Container Registry

If you're using a container registry, tag and push the images:

```bash
# Set your registry
REGISTRY="your-registry.com/barns"

# Tag and push each image
docker tag barns-api-bridge:latest $REGISTRY/api-bridge:latest
docker push $REGISTRY/api-bridge:latest

# Repeat for all services...
```

Then update the `image:` field in your Kubernetes YAML files to reference your registry.

## Troubleshooting

### Buildx not available

Install buildx:
```bash
docker buildx install
```

### Build fails with "no such file or directory"

Make sure you're running the build command from the BARNS root directory (where `docker-compose.yml` is located).

### Images still crash after rebuild

1. Verify the image architecture: `docker inspect <image> | grep Architecture`
2. Check pod logs: `kubectl logs <pod-name> -n barns`
3. Ensure you restarted the deployments after building
4. Check if the image is being pulled from a registry (may need to push ARM64 images)

