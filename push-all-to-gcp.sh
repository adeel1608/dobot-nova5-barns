#!/bin/bash
set -e

export REGISTRY_PATH="me-central2-docker.pkg.dev/qss-development-project/barns"
REGISTRY_HOST="me-central2-docker.pkg.dev"

echo "=========================================="
echo "Pushing All BARNS Images to GCP"
echo "Registry: ${REGISTRY_PATH}"
echo "=========================================="
echo ""

# Authenticate Docker with GCP before pushing
echo "Authenticating with GCP Artifact Registry..."
if ! gcloud auth print-access-token 2>/dev/null | docker login -u oauth2accesstoken --password-stdin "$REGISTRY_HOST" 2>/dev/null; then
    echo "ERROR: Failed to authenticate with GCP. Run: gcloud auth login"
    exit 1
fi
echo "Authentication successful."
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
        # Find the local tag (latest1 for robot1, latest for robot2)
        LOCAL_TAG=$(docker images --format '{{.Tag}}' "${image}" | head -1)
        echo "  → Tagging ${image}:${LOCAL_TAG}"
        docker tag ${image}:${LOCAL_TAG} ${REGISTRY_PATH}/${image}:${LOCAL_TAG}
        
        echo "  → Pushing to GCP..."
        docker push ${REGISTRY_PATH}/${image}:${LOCAL_TAG}
        
        echo "  ✓ ${image} pushed successfully"
    else
        echo "  ⚠ ${image} not found locally, skipping"
    fi
    echo ""
done

echo "=========================================="
echo "✓ Push Complete!"
echo "=========================================="