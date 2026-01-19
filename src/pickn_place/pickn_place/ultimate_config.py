#!/usr/bin/env python3
"""
ULTIMATE PERFORMANCE CONFIGURATION
Optimized settings for maximum speed and reliability
"""

from .optimized_robot_controller import RobotConfig

# ULTIMATE PERFORMANCE SETTINGS
ULTIMATE_CONFIG = RobotConfig(
    # Aggressive caching for maximum speed
    tf_cache_duration=0.5,          # 500ms cache for high-frequency operations
    cache_size=500,                 # Large cache for maximum hit rate
    
    # Maximum parallel processing
    max_workers=8,                  # Use all available CPU cores
    
    # Optimized motion settings
    velocity_scaling=1.0,           # Maximum safe speed
    acceleration_scaling=1.0,       # Maximum safe acceleration
    cartesian_max_step=0.002,       # Fine resolution for precision
    cartesian_fraction_threshold=0.9, # High success rate
    
    # Ultra-fast service calls
    service_timeout=3.0,            # Faster timeouts for responsiveness
    motion_timeout=45.0,            # Reasonable motion timeout
    
    # Optimized TF stability
    num_consecutive_tf=5,           # Fewer samples for speed (vs 9 default)
    translation_threshold=0.003,    # Slightly relaxed for speed
    rotation_threshold=3.0,         # Slightly relaxed for speed
    sample_delay=0.05,              # Faster sampling
    
    # Performance monitoring
    enable_performance_monitoring=True,
    enable_advanced_logging=True,
    enable_parallel_validation=True,
)

def get_ultimate_config():
    """Get the ultimate performance configuration"""
    return ULTIMATE_CONFIG
