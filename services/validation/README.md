# Validation Service

## Purpose and Workflow

The Validation Service provides quality control and verification functionality for the BARNS system. It performs various checks to ensure ingredients are available, equipment is properly positioned, and quality standards are met throughout the coffee-making process.

### Core Responsibilities
- **Ingredient Verification**: Check availability and quantity of ingredients (beans, milk, etc.)
- **Equipment Status**: Verify proper positioning and status of robotic equipment
- **Quality Control**: Ensure drinks meet quality standards (temperature, weight, etc.)
- **Safety Checks**: Validate safe operating conditions before proceeding with tasks
- **Sensor Integration**: Interface with hardware sensors and monitoring equipment
- **AI-Based Validation**: Integrate with computer vision and AI systems for advanced quality checks

### Workflow
1. **Validation Request**: Receives validation requests from Routine service
2. **Function Lookup**: Maps validation function names to implementations
3. **Parameter Processing**: Processes validation parameters and thresholds
4. **Hardware/AI Integration**: Interfaces with sensors, cameras, or AI systems
5. **Result Evaluation**: Determines pass/fail status based on criteria
6. **Response Delivery**: Returns detailed validation results with measurements

### Validation Flow
```
Validation Request → Function Mapping → Hardware/AI Interface → Result Evaluation → Response
```

## API Structure

### Core Endpoints

#### Validate Function (Called by Routine Service)
```http
POST /validate
Content-Type: application/json

{
  "function": "check_weight",
  "params": {
    "min_weight": 30
  }
}
```

### Request Structure

#### ValidationRequest Model
```python
{
  "function": str,        # Validation function name
  "params": dict         # Parameters for the validation function
}
```

### Response Structures

#### Successful Validation
```json
{
  "passed": true,
  "weight": 35,
  "details": "Weight check passed"
}
```

#### Failed Validation
```json
{
  "passed": false,
  "weight": 25,
  "details": "Insufficient weight detected",
  "required": 30,
  "actual": 25
}
```

#### Error Response
```json
{
  "error": "No such validation function 'invalid_function'",
  "passed": false,
  "available_functions": ["check_cup_present", "check_weight", "check_temperature"]
}
```

## Available Validation Functions

### Ingredient and Supply Checks
- **check_cup_present**: Verify cup is properly positioned
- **check_beans**: Verify sufficient coffee beans are available
- **check_milk**: Verify sufficient milk is available
- **check_water**: Verify sufficient water is available

### Quality Control Checks
- **check_weight**: Verify drink weight meets minimum requirements
- **check_temperature**: Verify temperature meets minimum requirements
- **check_volume**: Verify liquid volume is within acceptable range
- **check_color**: Verify drink color matches expected profile

### Equipment Status Checks
- **check_arm_position**: Verify robotic arm is in correct position
- **check_grinder_status**: Verify grinder is ready and functional
- **check_steam_pressure**: Verify steam system has adequate pressure
- **check_pump_pressure**: Verify espresso pump has adequate pressure

## Adding New Modules

### 1. Adding New Validation Functions

**Step 1**: Implement validation function:
```python
def check_new_parameter(params: dict):
    """New validation function."""
    threshold = params.get("threshold", 0)
    
    # Interface with hardware/AI system
    current_value = read_sensor_value()  # Your implementation
    
    # Evaluate result
    passed = current_value >= threshold
    
    return {
        "passed": passed,
        "current_value": current_value,
        "threshold": threshold,
        "details": f"{'Passed' if passed else 'Failed'} threshold check"
    }
```

**Step 2**: Register function in VALIDATORS mapping:
```python
VALIDATORS = {
    # ... existing validators
    "check_new_parameter": check_new_parameter,
}
```

### 2. Adding Hardware Sensor Integration

**Step 1**: Create sensor interface:
```python
# validation/sensors.py
class SensorInterface:
    def __init__(self, sensor_config: dict):
        self.config = sensor_config
        self.connection = self.establish_connection()
    
    def read_weight_sensor(self) -> float:
        """Read weight from hardware sensor."""
        # Hardware interface implementation
        return self.connection.get_weight()
    
    def read_temperature_sensor(self) -> float:
        """Read temperature from hardware sensor."""
        # Hardware interface implementation
        return self.connection.get_temperature()
```

**Step 2**: Integrate sensor interface:
```python
# In app.py
from .sensors import SensorInterface

sensor_interface = SensorInterface(config)

def check_weight(params: dict):
    required = params.get("min_weight", 0)
    current_weight = sensor_interface.read_weight_sensor()  # Real hardware
    return {"passed": current_weight >= required, "weight": current_weight}
```

### 3. Adding Computer Vision Integration

**Step 1**: Create vision interface:
```python
# validation/vision.py
import cv2
import numpy as np

class VisionInterface:
    def __init__(self, camera_config: dict):
        self.camera_id = camera_config.get("camera_id", 0)
        self.models = self.load_ai_models()
    
    def detect_cup_presence(self) -> dict:
        """Use computer vision to detect cup presence."""
        frame = self.capture_frame()
        detection_result = self.models.cup_detector(frame)
        
        return {
            "detected": detection_result.confidence > 0.8,
            "confidence": detection_result.confidence,
            "bounding_box": detection_result.bbox
        }
    
    def analyze_drink_color(self) -> dict:
        """Analyze drink color for quality control."""
        frame = self.capture_frame()
        color_analysis = self.models.color_analyzer(frame)
        
        return {
            "color_profile": color_analysis.profile,
            "quality_score": color_analysis.score,
            "expected_color": color_analysis.expected
        }
```

**Step 2**: Integrate vision system:
```python
# In app.py
from .vision import VisionInterface

vision = VisionInterface(vision_config)

def check_cup_present(params: dict):
    """AI-powered cup detection."""
    result = vision.detect_cup_presence()
    return {
        "passed": result["detected"],
        "confidence": result["confidence"],
        "details": f"Cup {'detected' if result['detected'] else 'not detected'}"
    }
```

### 4. Adding Advanced Analytics

**Step 1**: Create analytics module:
```python
# validation/analytics.py
class ValidationAnalytics:
    def __init__(self):
        self.validation_history = []
        self.performance_metrics = {}
    
    def record_validation(self, function: str, result: dict, params: dict):
        """Record validation results for analytics."""
        record = {
            "timestamp": time.time(),
            "function": function,
            "result": result,
            "params": params
        }
        self.validation_history.append(record)
        self.update_metrics(function, result["passed"])
    
    def get_success_rate(self, function: str) -> float:
        """Calculate success rate for a validation function."""
        function_results = [r for r in self.validation_history if r["function"] == function]
        if not function_results:
            return 0.0
        
        successes = sum(1 for r in function_results if r["result"]["passed"])
        return successes / len(function_results)
    
    def detect_anomalies(self) -> list:
        """Detect anomalous validation patterns."""
        anomalies = []
        for function in self.performance_metrics:
            success_rate = self.get_success_rate(function)
            if success_rate < 0.8:  # Threshold for anomaly
                anomalies.append({
                    "function": function,
                    "success_rate": success_rate,
                    "issue": "Low success rate detected"
                })
        return anomalies
```

**Step 2**: Integrate analytics:
```python
# In app.py
from .analytics import ValidationAnalytics

analytics = ValidationAnalytics()

@app.post("/validate")
def validate(request: ValidationRequest):
    # ... existing validation logic
    result = VALIDATORS[func_name](request.params or {})
    
    # Record for analytics
    analytics.record_validation(func_name, result, request.params)
    
    return result

@app.get("/analytics")
def get_validation_analytics():
    """Get validation analytics and anomaly detection."""
    return {
        "success_rates": {func: analytics.get_success_rate(func) for func in VALIDATORS.keys()},
        "anomalies": analytics.detect_anomalies(),
        "total_validations": len(analytics.validation_history)
    }
```

### 5. Adding Custom Validation Rules

**Step 1**: Create rule engine:
```python
# validation/rules.py
class ValidationRuleEngine:
    def __init__(self):
        self.rules = {}
        self.load_rules()
    
    def load_rules(self):
        """Load validation rules from configuration."""
        self.rules = {
            "temperature_rules": {
                "espresso": {"min": 85, "max": 95},
                "milk": {"min": 60, "max": 70}
            },
            "weight_rules": {
                "espresso": {"min": 25, "max": 35},
                "latte": {"min": 200, "max": 250}
            }
        }
    
    def get_temperature_threshold(self, drink_type: str) -> dict:
        """Get temperature thresholds for drink type."""
        return self.rules["temperature_rules"].get(drink_type, {"min": 60, "max": 100})
    
    def get_weight_threshold(self, drink_type: str) -> dict:
        """Get weight thresholds for drink type."""
        return self.rules["weight_rules"].get(drink_type, {"min": 100, "max": 300})
```

**Step 2**: Use rules in validation:
```python
# In app.py
from .rules import ValidationRuleEngine

rule_engine = ValidationRuleEngine()

def check_temperature_contextual(params: dict):
    """Context-aware temperature checking."""
    drink_type = params.get("drink_type", "default")
    current_temp = read_temperature_sensor()
    
    thresholds = rule_engine.get_temperature_threshold(drink_type)
    passed = thresholds["min"] <= current_temp <= thresholds["max"]
    
    return {
        "passed": passed,
        "temperature": current_temp,
        "thresholds": thresholds,
        "drink_type": drink_type
    }
```

## Environment Variables

```env
# Hardware Configuration
SENSOR_PORT=/dev/ttyUSB0              # Serial port for sensors
CAMERA_ID=0                          # Camera device ID
SCALE_CALIBRATION_FACTOR=1000        # Scale calibration

# AI/Vision Configuration
MODEL_PATH=/models/                  # Path to AI models
VISION_CONFIDENCE_THRESHOLD=0.8      # Minimum confidence for detections

# Quality Thresholds
DEFAULT_MIN_WEIGHT=25               # Default minimum weight (grams)
DEFAULT_MIN_TEMP=60                 # Default minimum temperature (°C)
```

## Configuration Files

### Validation Rules
```json
{
  "temperature_rules": {
    "espresso": {"min": 85, "max": 95},
    "milk": {"min": 60, "max": 70}
  },
  "weight_rules": {
    "espresso": {"min": 25, "max": 35},
    "latte": {"min": 200, "max": 250}
  }
}
```

## Development Setup

1. **Install Dependencies**:
   ```bash
   pip install fastapi uvicorn opencv-python numpy
   ```

2. **Hardware Setup** (if using real sensors):
   ```bash
   # Configure sensor interfaces
   sudo modprobe usbserial vendor=0x1234 product=0x5678
   ```

3. **Run Service**:
   ```bash
   uvicorn services.validation.app:app --host 0.0.0.0 --port 8000 --reload
   ```

## Testing

### Unit Testing
```bash
# Test validation functions
python -m pytest services/validation/tests/test_validators.py

# Test sensor interfaces
python -m pytest services/validation/tests/test_sensors.py
```

### Manual Testing
```bash
# Test validation endpoint
curl -X POST "http://localhost:8000/validate" \
  -H "Content-Type: application/json" \
  -d '{"function": "check_weight", "params": {"min_weight": 30}}'
```

## Integration Points

### With Routine Service
- **Receives**: Validation requests during task execution
- **Provides**: Pass/fail results with detailed measurements

### With Hardware Systems
- **Sensors**: Weight scales, temperature probes, pressure sensors
- **Cameras**: Computer vision for visual quality control
- **AI Systems**: Machine learning models for advanced validation

## Error Handling

### Hardware Failures
- Sensor communication timeouts
- Camera connection failures
- Calibration errors

### Validation Failures
- Threshold violations
- Quality control failures
- Safety condition violations

## Performance Considerations

- **Fast Response Times**: Critical for real-time validation during production
- **Hardware Interface Optimization**: Efficient sensor communication protocols
- **Caching**: Cache stable measurements to reduce hardware calls
- **Parallel Processing**: Support concurrent validation requests
- **Graceful Degradation**: Fallback to basic validation if advanced systems fail 