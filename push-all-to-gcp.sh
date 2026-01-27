#!/bin/bash
set -e

export REGISTRY_PATH="me-central2-docker.pkg.dev/qss-development-project/barns"

echo "=========================================="
echo "Pushing All BARNS Images to GCP"
echo "Registry: ${REGISTRY_PATH}"
echo "=========================================="
echo ""

# Array of all images
images=(
    "barns-robot1"
    "barns-robot2"
    "barns-routine"
    "barns-scheduler"
    "barns-validation"
    "barns-oms"
    "barns-automation"
    "barns-robot-arm"
    "barns-api-bridge"
    "barns-dashboard"
    "barns-video-stream"
)

# Tag and push each image
for image in "${images[@]}"; do
    echo "Processing: ${image}"
    
    if docker images | grep -q "^${image} "; then
        echo "  → Tagging ${image}:latest"
        docker tag ${image}:latest ${REGISTRY_PATH}/${image}:latest
        
        echo "  → Pushing to GCP..."
        docker push ${REGISTRY_PATH}/${image}:latest
        
        echo "  ✓ ${image} pushed successfully"
    else
        echo "  ⚠ ${image} not found locally, skipping"
    fi
    echo ""
done

echo "=========================================="
echo "✓ Push Complete!"
echo "=========================================="