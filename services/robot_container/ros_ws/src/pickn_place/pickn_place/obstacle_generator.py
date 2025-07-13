#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class AddObstacleNode(Node):
    def __init__(self):
        super().__init__('add_obstacle_node')
        # publisher for incremental scene updates
        self._scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)

        # give MoveIt a moment to wake up, then publish once
        self._timer = self.create_timer(1.0, self._publish_obstacle)

    def _publish_obstacle(self):
        # 1) Define the box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # half-extents in meters (e.g. 0.2×0.2×0.1)
        box.dimensions = [0.2, 0.2, 1.1]

        # 2) Pose of the box relative to base_link
        p = Pose()
        p.position.x = 0.0    # centered under base
        p.position.y = 0.0
        # the box sits on the ground, so if your box half‐height is 0.1, 
        # raise its center by -0.1 (i.e. half below origin) or whatever
        p.position.z = -0.555 
        p.orientation.w = 1.0

        # 3) Build the CollisionObject
        obj = CollisionObject()
        obj.id = 'base_block'
        obj.header.frame_id = 'base_link'
        obj.primitives = [box]
        obj.primitive_poses = [p]
        obj.operation = CollisionObject.ADD

        # 4) Wrap in a PlanningScene diff
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]

        # 5) Publish it
        self._scene_pub.publish(scene)
        self.get_logger().info('Published base_link obstacle.')
        # stop the timer so we only send this once
        self._timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = AddObstacleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

