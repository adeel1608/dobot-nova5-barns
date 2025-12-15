#!/bin/bash

# Quick script to install docker buildx

set -e

echo "Installing docker buildx..."

# Check if already installed
if docker buildx version &> /dev/null; then
    echo "docker buildx is already installed"
    docker buildx version
    exit 0
fi

# Try different installation methods
if command -v apt-get &> /dev/null; then
    echo "Installing via apt-get..."
    sudo apt-get update
    sudo apt-get install -y docker-buildx-plugin
elif command -v yum &> /dev/null; then
    echo "Installing via yum..."
    sudo yum install -y docker-buildx-plugin
else
    echo "Manual installation required"
    echo "Visit: https://github.com/docker/buildx#installing"
    exit 1
fi

# Verify installation
if docker buildx version &> /dev/null; then
    echo "Successfully installed docker buildx"
    docker buildx version
else
    echo "Installation completed but buildx not working"
    echo "Try: docker buildx install"
    exit 1
fi

