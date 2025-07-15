"""
Configuration module for Validation Service
Handles database, detection, and other service configurations
"""

import os
from dataclasses import dataclass
from typing import Optional

@dataclass
class DatabaseConfig:
    """Database connection configuration"""
    host: str = os.getenv("POSTGRES_HOST", "localhost")
    port: str = os.getenv("POSTGRES_PORT", "5432")
    database: str = os.getenv("POSTGRES_DB", "barns_validation")
    user: str = os.getenv("POSTGRES_USER", "validation_user")
    password: str = os.getenv("POSTGRES_PASSWORD", "validation_pass")
    
    @property
    def connection_string(self) -> str:
        return f"dbname={self.database} user={self.user} password={self.password} host={self.host} port={self.port}"

@dataclass
class DetectionConfig:
    """Coffee beans detection configuration"""
    # Periodic detection interval in seconds (default: 600 = 10 minutes)
    periodic_interval_seconds: int = int(os.getenv("DETECTION_INTERVAL_SECONDS", "600"))
    
    # Detection timeout in seconds
    detection_timeout_seconds: int = int(os.getenv("DETECTION_TIMEOUT_SECONDS", "30"))
    
    # Enable/disable periodic detection
    enable_periodic_detection: bool = os.getenv("ENABLE_PERIODIC_DETECTION", "true").lower() == "true"
    
    # Thread pool workers for detection
    max_detection_workers: int = int(os.getenv("MAX_DETECTION_WORKERS", "3"))
    
    @property
    def periodic_interval_minutes(self) -> float:
        """Get interval in minutes for logging/display"""
        return self.periodic_interval_seconds / 60

@dataclass
class RabbitMQConfig:
    """RabbitMQ connection configuration"""
    url: str = os.getenv("RABBITMQ_URL", "amqp://admin:admin123@rabbitmq:5672/")
    
@dataclass
class ValidationConfig:
    """Main validation service configuration"""
    # Sub-configurations
    database: DatabaseConfig
    detection: DetectionConfig
    rabbitmq: RabbitMQConfig
    
    # Service-level settings
    service_name: str = "validation"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    
    # Health check settings
    health_check_timeout: int = int(os.getenv("HEALTH_CHECK_TIMEOUT", "5"))
    
    def __init__(self):
        self.database = DatabaseConfig()
        self.detection = DetectionConfig()
        self.rabbitmq = RabbitMQConfig()

# Singleton instance for easy access throughout the service
config = ValidationConfig()

# Helper functions for easy access
def get_db_connection_string() -> str:
    """Get database connection string"""
    return config.database.connection_string

def get_detection_interval() -> int:
    """Get detection interval in seconds"""
    return config.detection.periodic_interval_seconds

def get_rabbitmq_url() -> str:
    """Get RabbitMQ URL"""
    return config.rabbitmq.url