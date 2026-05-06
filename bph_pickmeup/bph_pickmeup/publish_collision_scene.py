
#!/usr/bin/env python3
"""publish_collision_scene.py — adds static collision objects at startup."""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class CollisionScenePublisher(Node):
    def __init__(self):
        super().__init__('collision_scene_publisher')
        self._pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        # Retry every 2 seconds until confirmed
        self._timer = self.create_timer(2.0, self._publish_scene)
        self._published = False
        
    def _publish_scene(self):
        if self._published:
            return
        try:
            obj = self._make_table()
            if obj is None:
                self.get_logger().error("_make_table() returned None!")
                return
            self._pub.publish(obj)
            self.get_logger().info("Published table collision object.")
            self._published = True
        except Exception as e:
            self.get_logger().error(f"Failed to publish collision object: {e}")
    def _make_table(self):
        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'table'
        table.operation = CollisionObject.ADD
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.22, 0.6, 0.03]
        
        pose = Pose()
        pose.position.x = -0.1
        pose.position.y = -0.1
        pose.position.z = -0.215
        pose.orientation.w = 1.0
        
        table.primitives = [box]
        table.primitive_poses = [pose]
        return table  # just return, don't publish here


def main():
    rclpy.init()
    node = CollisionScenePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
