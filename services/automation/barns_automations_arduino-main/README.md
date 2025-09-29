# Automation Systems - README

## Overview
This project manages **7 automated machines** running on **ESP32 Dev Kits**, all integrated through MQTT messaging and RabbitMQ:

- Milk Dispenser  
- Syrup Dispenser  
- Ice Dispenser  
- Slush Dispenser  
- Tampering Machine  
- Grinding Machine  

Each machine operates independently with its own Arduino script and communicates over MQTT.

---

## MQTT Communication Structure

- **Topics**  
  - Request topic: `automation_<device>`  
    - Example: `automation_milk`, `automation_ice`, etc.  
  - Response topic (shared by all devices):  
    ```
    automation/response
    ```

- **Message Processing**  
  Each Arduino script contains:  
  - A **JSON decoder** to parse incoming requests.  
  - Logic to perform the requested task.  
  - A response mechanism that publishes results back to `automation/response`.

---

## Debugging Common Issues

1. **Issue:** Arduino performs the task but does not reply back.  
   - **Fix:** Restart the machine.

2. **Issue:** Arduino performs a task but then stops responding to new tasks.  
   - **Fix:**  
     - Re-upload the Arduino code to clear **EEPROM** and internal cache.  
     - Optionally, comment out unnecessary `Serial.print` statements to improve stability.
3. **Issue:** The esp does not do anything, not even connecting to network.
   - **Fix:** Re-upload the Arduino code to clear **EEPROM** and internal cache. 
---

## Running the Machines

Each machine has a **dedicated Python script** to trigger it.  
- All Python scripts follow the same structure, only differing in:  
  - The **message content**  
  - The **topic** they publish to  

### Requirements
Install the MQTT library:
```bash
pip install paho-mqtt
```

Ensure the **IP address** in each Python script points to the correct **RabbitMQ host**.

---

## Python Message Structure


```python
# Example message structure:
{
  "milk_type": "whole",
  "amount": 100
}

#another example (grinder)
{
        "shots_number": 2
}
```

---

## Arduino MQTT Connection
*(Add your ESP32 MQTT connection setup here)*

```cpp
// Example placeholder

  while (!client.connected()) {
//    Serial.print("Connecting to MQTT...");
    if (client.connect("Tampering Machine", mqttUser, mqttPassword)) {
      Serial.println(" connected");
    } else {
//      Serial.print(" failed, rc=");
      Serial.print(client.state());
      delay(1000);
    }
  }

  // Subscribe only to the tamping topic
  client.subscribe("automation_ice", 1);
  Serial.println("Subscribed to topic: automation_tampering");
```
