@echo off
REM BARNS RabbitMQ-based Microservices Startup Script for Windows

echo 🚀 Starting BARNS RabbitMQ-based Microservices...

REM Check if Docker is installed
docker --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker is not installed. Please install Docker first.
    pause
    exit /b 1
)

REM Check if Docker Compose is installed
docker-compose --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker Compose is not installed. Please install Docker Compose first.
    pause
    exit /b 1
)

REM Create necessary directories
echo 📁 Creating necessary directories...
if not exist "config" mkdir config
if not exist "services\scheduler\data" mkdir services\scheduler\data

REM Create default task configuration if it doesn't exist
if not exist "config\tasks.json" (
    echo 📝 Creating default task configuration...
    echo {> config\tasks.json
    echo   "make_coffee": {>> config\tasks.json
    echo     "steps": [>> config\tasks.json
    echo       {"type": "validation", "function": "check_cup_present", "params": {}},>> config\tasks.json
    echo       {"type": "validation", "function": "check_ingredient_availability", "params": {"ingredient": "beans", "amount_needed": 1}},>> config\tasks.json
    echo       {"type": "automation", "function": "grind_beans", "params": {"amount": 15}},>> config\tasks.json
    echo       {"type": "automation", "function": "brew_coffee", "params": {"temperature": 85}},>> config\tasks.json
    echo       {"type": "validation", "function": "update_inventory", "params": {"ingredient": "beans", "amount_used": 1}}>> config\tasks.json
    echo     ]>> config\tasks.json
    echo   },>> config\tasks.json
    echo   "add_milk": {>> config\tasks.json
    echo     "steps": [>> config\tasks.json
    echo       {"type": "validation", "function": "check_ingredient_availability", "params": {"ingredient": "milk", "amount_needed": 1}},>> config\tasks.json
    echo       {"type": "automation", "function": "dispense_milk", "params": {"amount": 100}},>> config\tasks.json
    echo       {"type": "validation", "function": "update_inventory", "params": {"ingredient": "milk", "amount_used": 1}}>> config\tasks.json
    echo     ]>> config\tasks.json
    echo   }>> config\tasks.json
    echo }>> config\tasks.json
)

REM Create default recipes if they don't exist
if not exist "services\scheduler\data\recipes.json" (
    echo 📝 Creating default recipes...
    echo {> services\scheduler\data\recipes.json
    echo   "espresso": {>> services\scheduler\data\recipes.json
    echo     "tasks": [>> services\scheduler\data\recipes.json
    echo       {"function": "make_coffee", "arm": 1}>> services\scheduler\data\recipes.json
    echo     ]>> services\scheduler\data\recipes.json
    echo   },>> services\scheduler\data\recipes.json
    echo   "latte": {>> services\scheduler\data\recipes.json
    echo     "tasks": [>> services\scheduler\data\recipes.json
    echo       {"function": "make_coffee", "arm": 1},>> services\scheduler\data\recipes.json
    echo       {"function": "add_milk", "arm": 2}>> services\scheduler\data\recipes.json
    echo     ]>> services\scheduler\data\recipes.json
    echo   }>> services\scheduler\data\recipes.json
    echo }>> services\scheduler\data\recipes.json
)

REM Stop any existing services
echo 🛑 Stopping existing services...
docker-compose -f docker-compose.rabbitmq.yml down

REM Build and start services
echo 🔨 Building and starting RabbitMQ-based services...
docker-compose -f docker-compose.rabbitmq.yml up --build -d

REM Wait for services to be healthy
echo ⏳ Waiting for services to be healthy...
timeout /t 30 /nobreak >nul

REM Check service status
echo 🔍 Checking service status...
docker-compose -f docker-compose.rabbitmq.yml ps

echo.
echo ✅ BARNS RabbitMQ-based Microservices started successfully!
echo.
echo 🌐 Access Points:
echo    • RabbitMQ Management UI: http://localhost:15672 (admin/admin123)
echo    • Dashboard: http://localhost:3000
echo    • Video Stream: http://localhost:8001
echo.
echo 📊 Service Status:
echo    • RabbitMQ: Message broker for inter-service communication
echo    • Validation Service: Handles sensor validation and inventory
echo    • Automation Service: Controls automation equipment
echo    • Routine Service: Orchestrates task execution
echo    • Scheduler Service: Manages order scheduling and coordination
echo    • OMS Service: Order management and business logic
echo    • Video Stream: Camera feeds (HTTP-based)
echo    • Dashboard: Web interface (HTTP-based)
echo.
echo 🔧 Useful Commands:
echo    • View logs: docker-compose -f docker-compose.rabbitmq.yml logs -f [service-name]
echo    • Stop services: docker-compose -f docker-compose.rabbitmq.yml down
echo    • Restart service: docker-compose -f docker-compose.rabbitmq.yml restart [service-name]
echo.
echo 📝 Note: All core services now communicate via RabbitMQ messages instead of HTTP APIs
echo.
pause 