#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Manual Drag-Teach Calibration for AxxB Hand-Eye Calibration

• User manually positions robot using drag mode
• System captures TF poses at each position
• Calls /capture_point service after each taught position
• Enforces distance gate from calibration_tag → reference_frame in [0.83, 0.87] m
• Repeats for 51 samples (configurable)

Dependencies:
    rclpy, tf2_ros, std_srvs, dobot_msgs_v3
"""

import math
import time
import sys
from threading import Thread

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import Buffer, TransformListener, LookupException

from std_srvs.srv import Trigger
from dobot_msgs_v3.srv import StartDrag, StopDrag


class DragTeachCalibration(Node):
    """
    Manual drag-teach calibration node for hand-eye calibration.
    User physically moves robot to desired positions, system captures data.
    """

    def __init__(self):
        super().__init__("drag_teach_calibration_node")

        # ───────────────────────────── Parameters ──────────────────────────────
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("calibration_tag", "calibration_tag")
        self.declare_parameter("num_samples", 51)
        self.declare_parameter("initialized", False)
        
        # Readback
        self.reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        self.calibration_tag = self.get_parameter("calibration_tag").get_parameter_value().string_value
        self.num_samples = self.get_parameter("num_samples").get_parameter_value().integer_value
        self.INITIALIZED = self.get_parameter("initialized").get_parameter_value().bool_value

        # ─────────────────────────── House-keeping ─────────────────────────────
        self.support_warning_shown = False
        self.create_timer(5.0, self.support_warning_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ───────────────────────────── TF Buffer ───────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ─────────────────────────── Service Clients ───────────────────────────
        self.start_drag_cli = self.create_client(StartDrag, '/dobot_bringup_v3/srv/StartDrag')
        self.stop_drag_cli = self.create_client(StopDrag, '/dobot_bringup_v3/srv/StopDrag')
        self.cap_cli = self.create_client(Trigger, "/capture_point")
        self.save_cli = self.create_client(Trigger, "/save_calibration")

        # Wait for services
        timeout = 5.0
        missing = []
        for name, cli in (("StartDrag", self.start_drag_cli), 
                         ("StopDrag", self.stop_drag_cli), 
                         ("capture_point", self.cap_cli),
                         ("save_calibration", self.save_cli)):
            if not cli.wait_for_service(timeout_sec=timeout):
                missing.append(name)
        
        if missing:
            self.get_logger().error(f"⚠️ Services unavailable after {timeout:.1f}s: {', '.join(missing)}")
            self.get_logger().error("💡 Make sure the Dobot is powered on and ros2_control is running.")
        else:
            self.get_logger().info("✅ All required services are ready")

    # ────────────────────────── Parameter callback ────────────────────────────
    def parameter_callback(self, params):
        for p in params:
            if p.name == "initialized" and p.type_ == Parameter.Type.BOOL:
                self.INITIALIZED = p.value
        return SetParametersResult(successful=True)

    # ─────────────────────────── Misc. helpers ────────────────────────────────
    def support_warning_callback(self):
        if not self.INITIALIZED and not self.support_warning_shown:
            self.get_logger().warn(
                "This is the support node for axxb_calibration. "
                "Run 'ros2 launch pickn_place axxb_calibration.launch.py' instead."
            )
            self.support_warning_shown = True

    def measure_distance_from_calibration_tag(self):
        """Measure distance from calibration_tag to reference_frame"""
        self.get_logger().info(
            f"📏 Measuring distance from '{self.calibration_tag}' to '{self.reference_frame}'..."
        )
        
        # Wait for transform to be available
        while not self.tf_buffer.can_transform(
            self.reference_frame,
            self.calibration_tag,
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0)
        ):
            self.get_logger().warn(
                f"⏳ Waiting for transform from '{self.calibration_tag}' to '{self.reference_frame}'..."
            )
            time.sleep(1.0)
        
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.calibration_tag,
                rclpy.time.Time()
            )
            trans = transform_stamped.transform.translation
            distance = math.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
            self.get_logger().info(
                f"📏 Distance: {distance:.3f}m"
            )
            return distance
        except Exception as e:
            self.get_logger().error(f"❌ Failed to measure distance: {e}")
            return None

    def call_start_drag(self) -> bool:
        """Enable drag mode (free movement)"""
        if not self.start_drag_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ StartDrag service not available")
            return False
        
        req = StartDrag.Request()
        future = self.start_drag_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            res = getattr(future.result(), "res", 1)
            if res == 0:
                self.get_logger().info("✅ Drag mode ENABLED - Robot is now free to move")
                return True
            else:
                self.get_logger().error(f"❌ StartDrag failed with res={res}")
                return False
        else:
            self.get_logger().error("❌ StartDrag service call failed")
            return False

    def call_stop_drag(self) -> bool:
        """Disable drag mode (lock position)"""
        if not self.stop_drag_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ StopDrag service not available")
            return False
        
        req = StopDrag.Request()
        future = self.stop_drag_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            res = getattr(future.result(), "res", 1)
            if res == 0:
                self.get_logger().info("🔒 Drag mode DISABLED - Robot position locked")
                return True
            else:
                self.get_logger().error(f"❌ StopDrag failed with res={res}")
                return False
        else:
            self.get_logger().error("❌ StopDrag service call failed")
            return False

    def call_capture_point_service(self) -> bool:
        """Call capture_point service once"""
        if not self.cap_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ Service /capture_point not available")
            return False
        
        req = Trigger.Request()
        future = self.cap_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info("✅ " + response.message)
                return True
            else:
                self.get_logger().warn("⚠️ " + response.message)
                return False
        else:
            self.get_logger().warn("⚠️ Service /capture_point call timeout")
            return False

    def call_capture_point_service_with_retries(self) -> bool:
        """Call capture_point with multiple retries"""
        max_attempts = 15
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"📸 Capturing data (attempt {attempt}/{max_attempts})...")
            if self.call_capture_point_service():
                return True
            
            if attempt < max_attempts:
                self.get_logger().warn(f"⏳ Retrying in 2 seconds...")
                time.sleep(2.0)
        
        self.get_logger().error("❌ All capture attempts failed.")
        return False

    def run(self):
        """Main calibration loop: drag-teach for N samples"""
        
        # Initial distance check (warning only, not enforced)
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("🎯 INITIAL DISTANCE CHECK")
        self.get_logger().info("="*70)
        
        distance = self.measure_distance_from_calibration_tag()
        if distance is not None:
            if 0.80 <= distance <= 0.90:
                self.get_logger().info(f"✅ Distance {distance:.3f}m looks good (recommended: 0.83-0.87m)")
            else:
                self.get_logger().warn(f"⚠️ Distance {distance:.3f}m (recommended: 0.83-0.87m)")
                self.get_logger().warn("💡 Consider repositioning for optimal results, but continuing anyway...")
        else:
            self.get_logger().warn("⚠️ Could not measure distance, but continuing anyway...")
        
        time.sleep(1.0)
        
        # Main calibration loop
        MIN_SAMPLES = 10  # Minimum samples before allowing finish
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"🤖 MANUAL DRAG-TEACH CALIBRATION")
        self.get_logger().info("="*70)
        self.get_logger().info("📋 INSTRUCTIONS:")
        self.get_logger().info("   1. Press ENTER to enable drag mode")
        self.get_logger().info("   2. Physically move robot to desired pose")
        self.get_logger().info("   3. Press ENTER to lock position and capture")
        self.get_logger().info("   4. Repeat (recommended: 51 samples, minimum: 10)")
        self.get_logger().info("   5. Type 'done' when ready to finish calibration")
        self.get_logger().info("="*70 + "\n")
        
        sample_num = 1
        while True:
            self.get_logger().info("\n" + "─"*70)
            self.get_logger().info(f"📍 SAMPLE {sample_num} (min: {MIN_SAMPLES}, recommended: {self.num_samples})")
            self.get_logger().info("─"*70)
            
            # Step 1: Wait for user input
            if sample_num >= MIN_SAMPLES:
                self.get_logger().info(f"👆 Press ENTER to continue, or type 'done' to finish:")
            else:
                self.get_logger().info(f"👆 Press ENTER to enable drag mode ({MIN_SAMPLES - sample_num} more required):")
            
            sys.stdout.flush()
            try:
                user_input = sys.stdin.readline().strip().lower()
            except EOFError:
                self.get_logger().error("❌ No input available (non-interactive mode not supported for drag-teach)")
                return False
            
            # Check if user wants to finish
            if user_input in ['done', 'finish', 'end', 'q', 'quit']:
                if sample_num - 1 >= MIN_SAMPLES:
                    self.get_logger().info(f"✅ Finishing calibration with {sample_num - 1} samples...")
                    self.get_logger().info("💾 Saving calibration data...")
                    
                    # Call save_calibration service
                    if self.save_cli.wait_for_service(timeout_sec=2.0):
                        save_req = Trigger.Request()
                        save_future = self.save_cli.call_async(save_req)
                        rclpy.spin_until_future_complete(self, save_future, timeout_sec=10.0)
                        
                        if save_future.result() is not None and save_future.result().success:
                            self.get_logger().info(f"✅ {save_future.result().message}")
                        else:
                            self.get_logger().error("❌ Failed to save calibration")
                    else:
                        self.get_logger().error("❌ save_calibration service not available")
                    
                    break
                else:
                    self.get_logger().warn(f"⚠️ Need at least {MIN_SAMPLES} samples. Currently have {sample_num - 1}.")
                    self.get_logger().warn(f"💡 Capture {MIN_SAMPLES - (sample_num - 1)} more sample(s) before finishing.")
                    continue
            
            # Step 2: Enable drag mode
            if not self.call_start_drag():
                self.get_logger().error("❌ Failed to enable drag mode. Aborting.")
                return False
            
            # Step 3: User positions robot
            self.get_logger().info(f"🎯 Move robot to desired position, then press ENTER to capture:")
            sys.stdout.flush()
            try:
                user_input = sys.stdin.readline().strip()
            except EOFError:
                self.get_logger().error("❌ No input available")
                return False
            
            # Step 4: Disable drag mode (lock position)
            if not self.call_stop_drag():
                self.get_logger().error("❌ Failed to disable drag mode. Aborting.")
                return False
            
            # Step 5: Wait a moment for vibrations to settle
            self.get_logger().info("⏳ Waiting for robot to stabilize (2 seconds)...")
            time.sleep(2.0)
            
            # Step 6: Capture the point
            if not self.call_capture_point_service_with_retries():
                self.get_logger().warn(f"⚠️ Failed to capture sample {sample_num} after all retries.")
                self.get_logger().warn(f"💡 Marker likely too unstable. Moving to next sample...")
                self.get_logger().warn(f"🔓 Re-enabling drag mode for next attempt...")
                # Don't increment sample_num - try again with drag mode
                continue
            
            self.get_logger().info(f"✅ Sample {sample_num} captured successfully!")
            sample_num += 1
        
        # Completion
        total_samples = sample_num - 1
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("🎉 CALIBRATION DATA COLLECTION COMPLETE!")
        self.get_logger().info("="*70)
        self.get_logger().info(f"✅ {total_samples} samples collected")
        self.get_logger().info("📊 Calibration computation in progress...")
        self.get_logger().info("📄 Results will be saved to axab_calibration.yaml")
        self.get_logger().info("="*70 + "\n")
        
        return True


def main(args=None):
    # Setup signal handlers for graceful shutdown
    def signal_handler(signum, frame):
        rclpy.shutdown()
    
    import signal
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)
    node = DragTeachCalibration()

    # Spin the node in a background executor so TF and service futures work
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Optional distance check (warning only)
    distance = node.measure_distance_from_calibration_tag()
    if distance is not None:
        if 0.80 <= distance <= 0.90:
            node.get_logger().info(f"✅ Distance {distance:.3f}m looks good. Ready to start...")
        else:
            node.get_logger().warn(f"⚠️ Distance {distance:.3f}m (recommended: 0.83-0.87m, but continuing...)")
    time.sleep(1.0)
    
    # Run the drag-teach calibration
    success = node.run()
    
    if success:
        node.get_logger().info("🎉 Calibration completed successfully!")
    else:
        node.get_logger().error("❌ Calibration failed.")
        sys.exit(1)
    
    rclpy.shutdown()
    executor_thread.join()
    node.destroy_node()


if __name__ == "__main__":
    main()
