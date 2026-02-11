#!/bin/bash

# BARNS Kubernetes Deployment Script
# This script deploys all BARNS services to Kubernetes in the correct order

set -e  # Exit on error

echo "========================================="
echo "BARNS Kubernetes Deployment"
echo "========================================="
echo ""

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Check if kubectl is available
if ! command -v kubectl &> /dev/null; then
    print_error "kubectl is not installed or not in PATH"
    exit 1
fi

print_status "kubectl found"

# Step 1: Create Namespace
echo ""
echo "Step 1: Creating namespace..."
kubectl apply -f namespace.yaml
print_status "Namespace created"


# 1. Create a key for the service account
gcloud iam service-accounts keys create gcr-key.json \
  --iam-account=barns-gcr-reader@qss-development-project.iam.gserviceaccount.com


# Create the image pull secret
kubectl create secret docker-registry gcr-json-key \
  --docker-server=me-central2-docker.pkg.dev \
  --docker-username=_json_key \
  --docker-password="$(gcloud auth print-access-token)" \
  --docker-email=qssairobotics@gcpqss.com \
  -n barns \
  --dry-run=client -o yaml | kubectl apply -f -



# Step 2: Create Secrets
echo ""
echo "Step 2: Creating secrets..."
kubectl apply -f secrets.yaml
print_status "Secrets created"

# Step 3: Create ConfigMaps
echo ""
echo "Step 3: Creating ConfigMaps..."
# Generate ConfigMaps from actual data files
kubectl create configmap routine-tasks-config \
  --from-file=tasks.json=../config/tasks.json \
  -n barns --dry-run=client -o yaml | kubectl apply -f -

kubectl create configmap scheduler-data \
  --from-file=../data/ \
  -n barns --dry-run=client -o yaml | kubectl apply -f -
  
kubectl apply -f configmaps/
print_status "ConfigMaps created"

# Step 4: Create Storage (PV and PVCs)
echo ""
echo "Step 4: Creating persistent storage..."
kubectl apply -f storage/
print_status "Storage created"

# Wait a moment for PVCs to bind
echo "Waiting for PVCs to bind..."
sleep 5

# Step 5: Deploy Infrastructure Services
echo ""
echo "Step 5: Deploying infrastructure services..."
kubectl apply -f infrastructure/

print_status "Infrastructure services deployed"
echo "Waiting for infrastructure services to be ready..."

# Wait for RabbitMQ
echo -n "  - Waiting for RabbitMQ..."
kubectl wait --for=condition=ready pod -l app=rabbitmq -n barns --timeout=120s 2>/dev/null && echo " Ready" || echo " Timeout (continuing anyway)"

# Wait for PostgreSQL
echo -n "  - Waiting for PostgreSQL..."
kubectl wait --for=condition=ready pod -l app=postgres -n barns --timeout=120s 2>/dev/null && echo " Ready" || echo " Timeout (continuing anyway)"

# Wait for Redis
echo -n "  - Waiting for Redis..."
kubectl wait --for=condition=ready pod -l app=redis -n barns --timeout=120s 2>/dev/null && echo " Ready" || echo " Timeout (continuing anyway)"

# Wait for InfluxDB
echo -n "  - Waiting for InfluxDB..."
kubectl wait --for=condition=ready pod -l app=influxdb -n barns --timeout=120s 2>/dev/null && echo " Ready" || echo " Timeout (continuing anyway)"

print_status "Infrastructure services are ready"

# Step 6: Deploy Application Services
echo ""
echo "Step 6: Deploying application services..."
kubectl apply -f services/
print_status "Application services deployed"

# Step 7: Deploy Frontend Services
echo ""
echo "Step 7: Deploying frontend services..."
kubectl apply -f frontend/
print_status "Frontend services deployed"

# Step 8: Show deployment status
echo ""
echo "========================================="
echo "Deployment Summary"
echo "========================================="
echo ""

kubectl get all -n barns

echo ""
echo "========================================="
echo "NodePort Services (External Access)"
echo "========================================="
echo ""
kubectl get svc -n barns | grep NodePort

echo ""
print_status "Deployment complete!"
echo ""
echo "Access URLs (replace <NODE-IP> with your Kubernetes node IP):"
echo "  - RabbitMQ Management: http://<NODE-IP>:30672"
echo "  - InfluxDB:           http://<NODE-IP>:30086"
echo "  - API Bridge:         http://<NODE-IP>:30000"
echo "  - Video Stream:       http://<NODE-IP>:30001"
echo "  - OMS Service:        http://<NODE-IP>:30002"
echo "  - Dashboard:          http://<NODE-IP>:30003"
echo ""
print_warning "Note: Make sure to build and push Docker images before deployment"
print_warning "Note: PostgreSQL init scripts require manual setup - see README.md"
echo ""






