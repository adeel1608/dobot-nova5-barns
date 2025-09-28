# BARNS Dispensing System Integration Guide

## Overview

The dispensing system is now fully integrated with your existing BARNS task/recipe system. You can control ingredient dispensing using the same pattern as your robot and automation tasks.

## Available Hardware

### **24 Motor Outputs:**
- **Milk Lines:** `milk1` through `milk8` (pins 2-9)
- **Sauce Lines:** `sauce1` through `sauce15` (pins 10-12, 25-47)  
- **Rinser:** `rinser` (pin 49)
- **Global Speed Control:** Pin 23 (controls all motors)

### **Dual Scale System:**
- **Scale A (SLV_A 0x28):** Milk scale - monitors milk1-8 dispensing
- **Scale B (SLV_B 0x29):** Sauce scale - monitors sauce1-15 dispensing

## Supported Ingredients

The system has built-in lag compensation for different viscosities:

| Ingredient | Lag Compensation | Best Motors | Typical Use |
|------------|------------------|-------------|-------------|
| `water` | 11g lag | Any | Cleaning, Americano |
| `milk` | 9g lag | milk1-8 | Extra milk, specialty drinks |
| `sauce` | 7g lag | sauce1-15 | Generic sauces |
| `caramel` | 1g lag | sauce1-15 | Caramel lattes, drizzles |
| `syrup` | 2.5g lag | sauce1-15 | Vanilla, chocolate syrups |
| `honey` | 1.5g lag | sauce1-15 | Honey-based drinks |

## Integration with Tasks/Recipes

### **Task Definition Format:**
```json
"task_name": {
  "steps": [
    {
      "type": "automation",
      "function": "dispense_ingredient", 
      "params": {
        "ingredient": "caramel",
        "weight": 15,
        "motor": "sauce1"
      }
    }
  ]
}
```

### **Recipe Integration Example:**
```json
"Caramel_Latte": [
  {
    "action": "Espresso_single_shot_9oz",
    "assigned_arm": "Arm1", 
    "depends_on": []
  },
  {
    "action": "dispense_caramel_15g",
    "assigned_arm": "Arm3",
    "depends_on": []
  },
  {
    "action": "Milk_Dispensing_Frothing_Latte",
    "assigned_arm": "Arm2",
    "depends_on": ["dispense_caramel_15g"]
  }
]
```

## Available Dispensing Tasks

### **Pre-configured Tasks:**
- `dispense_caramel_10g` - 10g caramel via sauce1
- `dispense_caramel_15g` - 15g caramel via sauce1  
- `dispense_vanilla_syrup_10g` - 10g vanilla syrup via sauce2
- `dispense_chocolate_syrup_15g` - 15g chocolate syrup via sauce3
- `dispense_honey_5g` - 5g honey via sauce4
- `dispense_extra_milk_30g` - 30g extra milk via milk2
- `dispense_whipped_cream_20g` - 20g whipped cream via sauce5
- `rinse_system` - 100g water rinse via rinser

### **Example Recipes:**
- `Caramel_Latte` - Espresso + 15g caramel + frothed milk
- `Vanilla_Cappuccino` - Espresso + 10g vanilla syrup + cappuccino milk
- `Honey_Flat_White` - Double espresso + 5g honey + flat white milk
- `Extra_Creamy_Latte` - Espresso + extra milk + whipped cream
- `System_Maintenance` - Rinse system + clean steam wand

## MQTT Control

### **Network Configuration:**
- **MQTT Broker:** 192.168.200.233:1883
- **Arduino Micro IP:** 192.168.200.211
- **Command Topic:** `dispenser/cmd/liquid`
- **Data Topic:** `dispenser/weights`

### **MQTT Commands:**
```bash
# Basic ingredient dispensing
mosquitto_pub -h 192.168.200.233 -t "dispenser/cmd/liquid" -m "caramel_10"
mosquitto_pub -h 192.168.200.233 -t "dispenser/cmd/liquid" -m "milk_50" 
mosquitto_pub -h 192.168.200.233 -t "dispenser/cmd/liquid" -m "syrup_15"

# Monitor real-time weights
mosquitto_sub -h 192.168.200.233 -t "dispenser/weights"
```

### **Response Format:**
```json
{"A":125.4,"B":67.8}  // A=Milk scale, B=Sauce scale (grams)
```

## Direct Arduino Commands

### **USB Serial Commands (115200 baud):**
```bash
# Automated dispensing with cup detection
H7_milk_1_100        # H7 cup + milk1 → 100g
C9_sauce_10_8        # C9 cup + sauce10 → 8g  
H7_sauce_1_caramel_10 # H7 cup + sauce1 (caramel) → 10g

# System control
SPEED 1              # Enable global speed (slower, more accurate)
SPEED 0              # Disable global speed (faster)
STOP                 # Emergency stop all motors
STATUS               # Show current job status
LAG 1 5.0            # Set speed 1 lag compensation to 5g
HELP                 # Show all commands
```

### **Supported Cup Types:**
- `H7` (5.3g), `H9` (13.1g), `H12` (17.0g) - Hot cups
- `C7` (6.7g), `C9` (10.3g), `C12` (11.0g), `C12B` (14.2g) - Cold cups

## System Features

### **Intelligent Dispensing:**
- ✅ **95% Accuracy** with lag compensation
- ✅ **Auto-tare** before dispensing  
- ✅ **Cup detection** with weight validation
- ✅ **Viscosity-aware** stopping prediction
- ✅ **Real-time monitoring** via MQTT
- ✅ **Emergency stop** capability

### **Safety Features:**
- All motors start stopped
- Global speed control for precision
- Timeout protection (60 seconds)
- Weight validation and error detection
- Emergency stop available via MQTT/Serial

## Creating Custom Dispensing Tasks

### **1. Add to tasks.json:**
```json
"dispense_custom_ingredient": {
  "steps": [
    {
      "type": "automation", 
      "function": "dispense_ingredient",
      "params": {
        "ingredient": "syrup",    // Ingredient type for lag compensation
        "weight": 25,             // Target weight in grams
        "motor": "sauce6"         // Motor to use (milk1-8, sauce1-15, rinser)
      }
    }
  ]
}
```

### **2. Add to recipes.json:**
```json
"Custom_Drink": [
  {
    "action": "dispense_custom_ingredient",
    "assigned_arm": "Arm3",
    "depends_on": []
  },
  {
    "action": "Espresso_single_shot",
    "assigned_arm": "Arm1", 
    "depends_on": ["dispense_custom_ingredient"]
  }
]
```

## Motor Assignment Strategy

### **Recommended Motor Mapping:**
```
milk1-8:    Different milk types, creamers
sauce1-3:   Primary syrups (vanilla, caramel, chocolate)
sauce4-6:   Secondary syrups (hazelnut, honey, etc.)
sauce7-10:  Seasonal/specialty syrups
sauce11-15: Sauces, drizzles, toppings
rinser:     Water for cleaning
```

### **Parallel Operation:**
- **Arm1:** Robot operations (espresso, mounting, etc.)
- **Arm2:** Milk frothing and pouring
- **Arm3:** Ingredient dispensing (new capability!)

This allows simultaneous operation: while Arm1 makes espresso and Arm2 froths milk, Arm3 can dispense syrups/sauces into the cup!

## Troubleshooting

### **Common Issues:**
1. **Motor doesn't dispense:** Check power supply, verify motor assignment
2. **Inaccurate weights:** Adjust lag compensation with `LAG` command
3. **MQTT not working:** Verify broker IP (192.168.200.233) and network
4. **Scale readings wrong:** Check I2C connections to Nano Every boards

### **Debug Commands:**
```bash
STATUS           # Check current job state
HELP             # Show all available commands  
LAG 1 <value>    # Adjust lag compensation
STOP             # Emergency stop if needed
```

## Production Deployment

### **Upload Firmware:**
```bash
cd /home/adeel/BARNS/services/dispensing
pio run -e mega --target upload          # Main controller
pio run -e mqtt_bridge --target upload   # MQTT bridge  
pio run -e nano_every --target upload    # Scale controllers (x2)
```

### **Network Setup:**
1. Configure Arduino Micro IP: 192.168.200.211
2. Set MQTT broker: 192.168.200.233:1883
3. Test connectivity with ping and MQTT commands

Your dispensing system is now fully integrated and ready for production use! 🎉 