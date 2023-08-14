import rclpy
from rclpy.node import Node

from tracking.lib import *
import tracking.PID as PID

from robot_interfaces.msg import Controller


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('PID_controller')        
        self.controller_input_publisher = self.create_publisher(Controller, 'controller/input',10)
        self.timer = self.create_timer(1.0/30, self.publish_input)
        
    def publish_input(self):
        self.controller_input_publisher.publish(Controller(controller_out=[254, 15814.5, 170]))
        self.get_logger().info("Publish input")
        

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
