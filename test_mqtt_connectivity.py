#!/usr/bin/env python3
"""
MQTT Connectivity Test Script
Tests the connection between automation service and Arduino hardware
"""

import json
import time
import asyncio
import sys
import signal

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("❌ paho-mqtt not installed. Install with: pip install paho-mqtt")
    sys.exit(1)

class MQTTTester:
    def __init__(self):
        self.response_received = False
        self.response_data = None
        self.connected = False
        
    def on_connect(self, client, userdata, flags, rc, props=None):
        print(f"✅ Connected to MQTT broker with code {rc}")
        if rc == 0:
            self.connected = True
            client.subscribe("automation/response", qos=1)
            print("📡 Subscribed to automation/response")
        else:
            print(f"❌ Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            print(f"📨 Response received: {json.dumps(payload, indent=2)}")
            self.response_data = payload
            self.response_received = True
        except json.JSONDecodeError:
            print(f"❌ Invalid JSON received: {msg.payload.decode()}")

    def on_disconnect(self, client, userdata, rc):
        print(f"🔌 Disconnected from MQTT broker (code: {rc})")
        self.connected = False

    async def test_milk_dispenser(self):
        """Test milk dispenser communication"""
        print("\n🧪 Testing Milk Dispenser Communication")
        print("=" * 50)
        
        # Reset state
        self.response_received = False
        self.response_data = None
        
        # Create MQTT client
        client = mqtt.Client(protocol=mqtt.MQTTv311)
        client.username_pw_set("admin", "admin123")
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        
        try:
            # Connect to broker
            print("🔗 Connecting to MQTT broker at rabbitmq:1883...")
            client.connect("localhost", 1883, 60)
            client.loop_start()
            
            # Wait for connection
            connection_timeout = 10
            connection_start = time.time()
            while not self.connected and (time.time() - connection_start) < connection_timeout:
                await asyncio.sleep(0.1)
            
            if not self.connected:
                print("❌ Failed to connect to MQTT broker")
                return False
            
            # Give time for subscription to be processed
            await asyncio.sleep(0.5)
            
            # Send test message
            test_payload = {
                "milk_type": "whole",
                "amount": 50  # Small amount for testing
            }
            
            print(f"📤 Sending test message: {json.dumps(test_payload)}")
            client.publish("automation_milk", json.dumps(test_payload), qos=1)
            
            # Wait for response
            print("⏳ Waiting for Arduino response (30 seconds timeout)...")
            response_timeout = 30
            start_time = time.time()
            
            while not self.response_received and (time.time() - start_time) < response_timeout:
                await asyncio.sleep(0.1)
            
            if self.response_received:
                print("✅ Test successful! Arduino responded correctly.")
                return True
            else:
                print("❌ Test failed: No response from Arduino")
                return False
                
        except Exception as e:
            print(f"❌ Test failed with exception: {e}")
            return False
        finally:
            client.loop_stop()
            client.disconnect()

    async def run_diagnostics(self):
        """Run comprehensive MQTT diagnostics"""
        print("🩺 BARNS MQTT Diagnostics")
        print("=" * 50)
        
        # Test 1: Basic connectivity
        print("\n1️⃣  Testing basic MQTT connectivity...")
        
        # Test 2: Milk dispenser
        success = await self.test_milk_dispenser()
        
        # Summary
        print(f"\n📊 Diagnostic Summary")
        print("=" * 30)
        if success:
            print("✅ MQTT communication is working correctly")
            print("🎯 The timeout issue is likely in the routine service timing")
            print("\n💡 Recommendations:")
            print("   - Check routine service timeout settings")
            print("   - Verify network latency between services")
            print("   - Monitor Arduino execution time")
        else:
            print("❌ MQTT communication failed")
            print("\n🔧 Troubleshooting steps:")
            print("   1. Check if Arduino is connected and running")
            print("   2. Verify Arduino WiFi connection")
            print("   3. Check RabbitMQ MQTT plugin is enabled")
            print("   4. Verify network connectivity")
            print("   5. Check Arduino serial monitor for errors")

def signal_handler(sig, frame):
    print("\n🛑 Test interrupted by user")
    sys.exit(0)

async def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    tester = MQTTTester()
    await tester.run_diagnostics()

if __name__ == "__main__":
    asyncio.run(main()) 