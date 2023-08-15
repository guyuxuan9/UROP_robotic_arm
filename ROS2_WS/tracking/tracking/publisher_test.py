import rclpy
from rclpy.node import Node

from tracking.lib import *
import tracking.PID as PID

from robot_interfaces.msg import MultiRawIdPosDur
from robot_interfaces.msg import RawIdPosDur


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('Board_publisher')        
        self.controller_input_publisher = self.create_publisher(MultiRawIdPosDur, 'motion_topic/multi_id_pos_dur',10)
        self.timer = self.create_timer(1.0/30, self.publish_input)
        
    def publish_input(self):
        lst = [RawIdPosDur(id=3,position=30,duration=1000),
                           RawIdPosDur(id=4,position=40,duration=1000),
                           RawIdPosDur(id=5,position=50,duration=1000),
                           RawIdPosDur(id=6,position=60,duration=1000)]
        self.controller_input_publisher.publish(MultiRawIdPosDur(id_pos_dur_list = lst))
        self.get_logger().info("Publish input")
        

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
