#!/bin/bash
set -e
docker compose -f docker-compose.arms.yml down
echo "Setting up X11 forwarding for Docker containers..."
xhost +local:root

echo "Starting BARNS services..."
docker compose -f docker-compose.arms.yml up -d --build