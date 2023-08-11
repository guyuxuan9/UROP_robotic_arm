import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import RawIdPosDur
from robot_interfaces.msg import MultiRawIdPosDur
import cv2
import yaml
import math
import time
import numpy as np
from cv_bridge import CvBridge
from tracking.lib import *
import tracking.PID as PID
from tracking.ArmMoveIK import *
import tracking.Board as Board



AK = ArmIK()

class ArmMove(Node):
    def __init__(self):
        super().__init__('motion_publisher')
        self.motion_publisher = self.create_publisher(MultiRawIdPosDur, 'motion_topic', 10)
    
        self.timer = self.create_timer(1, self.publish_motion) 

          

    def publish_motion(self):
        (servos, alpha, movetime) = AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
        lst = [RawIdPosDur(id=3,position=servos["servo3"],duration=movetime),  RawIdPosDur(id=4,position=servos["servo4"],duration=movetime), RawIdPosDur(id=5,position=servos["servo5"],duration=movetime),  RawIdPosDur(id=6,position=servos["servo6"],duration=movetime)]
        
        self.motion_publisher.publish(MultiRawIdPosDur(id_pos_dur_list =lst))
        
        time.sleep(1)
        (servos, alpha, movetime) = AK.setPitchRangeMoving((0, 15, 15), -45, -90, 0, 1000)
        lst = [RawIdPosDur(id=3,position=servos["servo3"],duration=movetime),  RawIdPosDur(id=4,position=servos["servo4"],duration=movetime), RawIdPosDur(id=5,position=servos["servo5"],duration=movetime),  RawIdPosDur(id=6,position=servos["servo6"],duration=movetime)]
        
        self.motion_publisher.publish(MultiRawIdPosDur(id_pos_dur_list =lst))
        time.sleep(1)
        (servos, alpha, movetime) = AK.setPitchRangeMoving((0, 15, 15), 0, 0, -90, 1000)
        lst = [RawIdPosDur(id=3,position=servos["servo3"],duration=movetime),  RawIdPosDur(id=4,position=servos["servo4"],duration=movetime), RawIdPosDur(id=5,position=servos["servo5"],duration=movetime),  RawIdPosDur(id=6,position=servos["servo6"],duration=movetime)]
        
        self.motion_publisher.publish(MultiRawIdPosDur(id_pos_dur_list =lst))
        time.sleep(1)  
        
def main(args=None):
    rclpy.init(args=args)
    node = ArmMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
