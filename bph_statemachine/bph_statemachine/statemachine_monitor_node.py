#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

## Minimal subscriber node to substitute for the SMACH display
## that does not work in Kilted (and also entirely written by hand
## and not debugged with AI, with reference to the ROS pub/sub tutorials
## https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html


from std_msgs.msg import String

class StateMachineDisplay(Node):
    def __init__(self):
        super().__init__('state_machine_display')
        self.subscription = self.create_subscription(
            String, 'smach_state', self.listener_callback, 10)
        self.subscription
        
    def listener_callback(self, msg):
        self.get_logger().info('Current state: "%s"'% msg.data)

def main(args=None):
    rclpy.init(args=args)
    sm_display = StateMachineDisplay()
    rclpy.spin(sm_display)

    
    sm_display.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
