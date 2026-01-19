#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from dobot_msgs_v3.srv import InverseSolution, ServoJ

class MoveRobotArm(Node):
    def __init__(self):
        super().__init__('move_robot_arm')
        # Create service client for InverseSolution
        self.ik_client = self.create_client(InverseSolution, '/dobot_bringup_v3/srv/InverseSolution')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for InverseSolution service...')
            
        # Create service client for ServoJ (for joint-angle based movement)
        self.servoj_client = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        while not self.servoj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ServoJ service...')

    def move_arm(self, x, y, z, rx, ry, rz, 
                 offset_x=0.0, offset_y=0.0, offset_z=0.0, 
                 offset_rx=0.0, offset_ry=0.0, offset_rz=0.0,
                 move_time=0.4):
        # Apply offsets to the target pose.
        target_x = x + offset_x
        target_y = y + offset_y
        target_z = z + offset_z
        target_rx = rx + offset_rx
        target_ry = ry + offset_ry
        target_rz = rz + offset_rz

        self.get_logger().info(
            f"Target Pose after offsets: X={target_x}, Y={target_y}, Z={target_z}, "
            f"Rx={target_rx}, Ry={target_ry}, Rz={target_rz}"
        )

        # Prepare and call InverseSolution to compute joint angles from the Cartesian pose.
        ik_req = InverseSolution.Request()
        ik_req.x = target_x
        ik_req.y = target_y
        ik_req.z = target_z
        ik_req.rx = target_rx
        ik_req.ry = target_ry
        ik_req.rz = target_rz
        ik_req.user = 0   # Adjust as needed
        ik_req.tool = 0   # Adjust as needed

        self.get_logger().info("Requesting IK solution...")
        future_ik = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future_ik)
        if future_ik.result() is not None:
            ik_response = future_ik.result()
            if ik_response.res == 0:
                # Parse the angle string (e.g., "{146.620207,39.025305,35.590541,195.015056,89.986529,146.613157}")
                angles_str = ik_response.angle.strip("{}")
                try:
                    angles = [float(a) for a in angles_str.split(",")]
                except ValueError as e:
                    self.get_logger().error(f"Failed to parse joint angles: {e}")
                    return

                self.get_logger().info(
                    f"IK solution found: J1={angles[0]}, J2={angles[1]}, J3={angles[2]}, "
                    f"J4={angles[3]}, J5={angles[4]}, J6={angles[5]}"
                )
                # Now, use ServoJ to move using the computed joint angles.
                servoj_req = ServoJ.Request()
                servoj_req.j1 = angles[0]
                servoj_req.j2 = angles[1]
                servoj_req.j3 = angles[2]
                servoj_req.j4 = angles[3]
                servoj_req.j5 = angles[4]
                servoj_req.j6 = angles[5]
                servoj_req.t = move_time  # Movement time in seconds

                self.get_logger().info("Sending ServoJ command to move the robot arm...")
                future_servoj = self.servoj_client.call_async(servoj_req)
                rclpy.spin_until_future_complete(self, future_servoj)
                if future_servoj.result() is not None:
                    self.get_logger().info(f"ServoJ response: {future_servoj.result()}")
                else:
                    self.get_logger().error("ServoJ service call failed")
            else:
                self.get_logger().error(f"Inverse solution failed with error code: {ik_response.res}")
        else:
            self.get_logger().error("InverseSolution service call failed")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotArm()

    # Read target pose from command-line if provided.
    # Expected arguments: x y z rx ry rz [offset_x offset_y offset_z offset_rx offset_ry offset_rz] [move_time]
    if len(sys.argv) >= 7:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        rx = float(sys.argv[4])
        ry = float(sys.argv[5])
        rz = float(sys.argv[6])
    else:
        # Default target pose; modify as needed.
        x, y, z, rx, ry, rz = -350.0, -90.0, 375.0, 90.0, 0.0, -90.0

    # Optional offsets
    if len(sys.argv) >= 13:
        offset_x = float(sys.argv[7])
        offset_y = float(sys.argv[8])
        offset_z = float(sys.argv[9])
        offset_rx = float(sys.argv[10])
        offset_ry = float(sys.argv[11])
        offset_rz = float(sys.argv[12])
    else:
        offset_x = offset_y = offset_z = offset_rx = offset_ry = offset_rz = 0.0

    # Optional move time (seconds)
    if len(sys.argv) >= 14:
        move_time = float(sys.argv[13])
    else:
        move_time = 3.0

    node.move_arm(x, y, z, rx, ry, rz, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, move_time)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

