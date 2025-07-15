#!/bin/bash
set -e

echo "Setting up X11 forwarding for Docker containers..."
xhost +local:root

echo "Stopping existing BARNS services..."
docker compose -f docker-compose.arms.yml down

echo "Starting BARNS services..."
# Start all services normally - Docker Compose will use the cached robot image
# and build other services as needed
docker compose -f docker-compose.arms.yml up -d --build

echo "BARNS services started successfully!"
echo "Dashboard: http://localhost:3000"
echo "RabbitMQ Management: http://localhost:15672 (admin/admin123)"
