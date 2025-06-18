#!/bin/bash

# BARNS RabbitMQ-based Microservices Startup Script

echo "🚀 Starting BARNS RabbitMQ-based Microservices..."

# Check if Docker and Docker Compose are installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

if ! command -v docker-compose &> /dev/null; then
    echo "❌ Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Create necessary directories
echo "📁 Creating necessary directories..."
mkdir -p config
mkdir -p services/scheduler/data

# Create default task configuration if it doesn't exist
if [ ! -f "config/tasks.json" ]; then
    echo "📝 Creating default task configuration..."
    cat > config/tasks.json << 'EOF'
{
  "make_coffee": {
    "steps": [
      {"type": "validation", "function": "check_cup_present", "params": {}},
      {"type": "validation", "function": "check_ingredient_availability", "params": {"ingredient": "beans", "amount_needed": 1}},
      {"type": "automation", "function": "grind_beans", "params": {"amount": 15}},
      {"type": "automation", "function": "brew_coffee", "params": {"temperature": 85}},
      {"type": "validation", "function": "update_inventory", "params": {"ingredient": "beans", "amount_used": 1}}
    ]
  },
  "add_milk": {
    "steps": [
      {"type": "validation", "function": "check_ingredient_availability", "params": {"ingredient": "milk", "amount_needed": 1}},
      {"type": "automation", "function": "dispense_milk", "params": {"amount": 100}},
      {"type": "validation", "function": "update_inventory", "params": {"ingredient": "milk", "amount_used": 1}}
    ]
  },
  "add_syrup": {
    "steps": [
      {"type": "validation", "function": "check_ingredient_availability", "params": {"ingredient": "syrup", "amount_needed": 1}},
      {"type": "automation", "function": "dispense_syrup", "params": {"amount": 30}},
      {"type": "validation", "function": "update_inventory", "params": {"ingredient": "syrup", "amount_used": 1}}
    ]
  }
}
EOF
fi

# Create default recipes if they don't exist
if [ ! -f "services/scheduler/data/recipes.json" ]; then
    echo "📝 Creating default recipes..."
    cat > services/scheduler/data/recipes.json << 'EOF'
{
  "espresso": {
    "tasks": [
      {"function": "make_coffee", "arm": 1}
    ]
  },
  "latte": {
    "tasks": [
      {"function": "make_coffee", "arm": 1},
      {"function": "add_milk", "arm": 2}
    ]
  },
  "cappuccino": {
    "tasks": [
      {"function": "make_coffee", "arm": 1},
      {"function": "add_milk", "arm": 2}
    ]
  },
  "mocha": {
    "tasks": [
      {"function": "make_coffee", "arm": 1},
      {"function": "add_milk", "arm": 2},
      {"function": "add_syrup", "arm": 1}
    ]
  }
}
EOF
fi

# Stop any existing services
echo "🛑 Stopping existing services..."
docker-compose -f docker-compose.rabbitmq.yml down

# Build and start services
echo "🔨 Building and starting RabbitMQ-based services..."
docker-compose -f docker-compose.rabbitmq.yml up --build -d

# Wait for services to be healthy
echo "⏳ Waiting for services to be healthy..."
sleep 30

# Check service status
echo "🔍 Checking service status..."
docker-compose -f docker-compose.rabbitmq.yml ps

echo ""
echo "✅ BARNS RabbitMQ-based Microservices started successfully!"
echo ""
echo "🌐 Access Points:"
echo "   • RabbitMQ Management UI: http://localhost:15672 (admin/admin123)"
echo "   • Dashboard: http://localhost:3000"
echo "   • Video Stream: http://localhost:8001"
echo ""
echo "📊 Service Status:"
echo "   • RabbitMQ: Message broker for inter-service communication"
echo "   • Validation Service: Handles sensor validation and inventory"
echo "   • Automation Service: Controls automation equipment"
echo "   • Routine Service: Orchestrates task execution"
echo "   • Scheduler Service: Manages order scheduling and coordination"
echo "   • OMS Service: Order management and business logic"
echo "   • Video Stream: Camera feeds (HTTP-based)"
echo "   • Dashboard: Web interface (HTTP-based)"
echo ""
echo "🔧 Useful Commands:"
echo "   • View logs: docker-compose -f docker-compose.rabbitmq.yml logs -f [service-name]"
echo "   • Stop services: docker-compose -f docker-compose.rabbitmq.yml down"
echo "   • Restart service: docker-compose -f docker-compose.rabbitmq.yml restart [service-name]"
echo ""
echo "📝 Note: All core services now communicate via RabbitMQ messages instead of HTTP APIs" 