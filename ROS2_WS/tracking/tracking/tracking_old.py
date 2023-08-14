import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import RawIdPosDur
from robot_interfaces.msg import MultiRawIdPosDur
import cv2
import yaml
import math
import numpy as np
from cv_bridge import CvBridge
from tracking.lib import *
import tracking.PID as PID
from tracking.ArmMoveIK import *
import tracking.Board as Board


size = (640, 480)

x_dis = 500
y_dis = 10
Z_DIS = 18
z_dis = Z_DIS
x_pid = PID.PID(P=0.1, I=0.00, D=0.008) 
y_pid = PID.PID(P=0.0001, I=0, D=0)
z_pid = PID.PID(P=0.005, I=0, D=0)
st = True

AK = ArmIK()

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.raw_publisher = self.create_publisher(Image, 'video_topic/raw_image', 10)
        self.processed_publisher = self.create_publisher(Image, 'video_topic/processed_image',10)
        self.motion_publisher = self.create_publisher(MultiRawIdPosDur,'motion_topic/multi_id_pos_dur',10)
        
        with open('/ros2/ros2_ws/src/testing/testing/lab_config2.yaml', 'r') as file:
            self.lab_data = yaml.safe_load(file)  # dictionary
            
        self.range_rgb = {  'red':   (0, 0, 255),
                            'blue':  (255, 0, 0),
                            'green': (0, 255, 0),
                            'black': (0, 0, 0),
                            'white': (255, 255, 255)}
        self.__target_color = ('blue')
        self.timer = self.create_timer(1.0/30, self.publish_frame) # frame rate: 30fps
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)  # Open the default camera (change the index if needed)   
          
    
    def img_proc(self,img):
        global x_dis, y_dis, z_dis
        global st
        size = (640, 480)
        img_copy = img.copy()
        img_h, img_w = img.shape[:2] # img_h = 480， img_w = 640


        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  

        area_max = 0
        areaMaxContour = 0

        for i in self.lab_data:
            if i in self.__target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                             (self.lab_data[detect_color]['min'][0],
                                              self.lab_data[detect_color]['min'][1],
                                              self.lab_data[detect_color]['min'][2]),
                                             (self.lab_data[detect_color]['max'][0],
                                              self.lab_data[detect_color]['max'][1],
                                              self.lab_data[detect_color]['max'][2])) 
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  
                areaMaxContour, area_max = getAreaMaxContour(contours)  
        if area_max > 500:  
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  

            center_x = int(rescale(center_x, 0, size[0], 0, img_w))
            center_y = int(rescale(center_y, 0, size[1], 0, img_h))
            radius = int(rescale(radius, 0, size[0], 0, img_w))
            # print(radius)
            if radius > 150:    # if the object is too close to the camera, do nothing
                return img

            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            cv2.circle(img,(center_x,center_y),radius,(0,255,0),2)
            cv2.drawContours(img, [box], -1, self.range_rgb[self.__target_color], 2)
            
            
            x_pid.SetPoint = img_w / 2.0  
            x_pid.update(center_x)  
            dx = x_pid.output
            x_dis += int(dx)  

            x_dis = 0 if x_dis < 0 else x_dis
            x_dis = 1000 if x_dis > 1000 else x_dis

            y_pid.SetPoint = 9000  
            # print(area_max)
            if abs(area_max - 9000) < 50:
                area_max = 9000
            y_pid.update(area_max)  
            dy = y_pid.output
            y_dis += dy  
            y_dis = 5.00 if y_dis < 5.00 else y_dis
            y_dis = 10.00 if y_dis > 10.00 else y_dis
            
            if abs(center_y - img_h/2.0) < 20:
                z_pid.SetPoint = center_y
            else:
                z_pid.SetPoint = img_h / 2.0
                
            z_pid.update(center_y)
            dy = z_pid.output
            z_dis += dy

            z_dis = 32.00 if z_dis > 32.00 else z_dis
            z_dis = 10.00 if z_dis < 10.00 else z_dis
            
            self.get_logger().info('x_dis: "%s" y_dis: "%s", z_dis: "%s"' % (x_dis,y_dis,z_dis))
            
            target = AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), 40, 90)
            if target:
                servo_data = target[0]
                if st:
                    Board.setBusServoPulse(3, servo_data['servo3'], 1000)
                    Board.setBusServoPulse(4, servo_data['servo4'], 1000)
                    Board.setBusServoPulse(5, servo_data['servo5'], 1000)
                    Board.setBusServoPulse(6, int(x_dis), 1000)
                    
                    # publish the pulse width data to the topic
                    lst = [RawIdPosDur(id=3,position=servo_data['servo3'],duration=1000),
                           RawIdPosDur(id=4,position=servo_data['servo4'],duration=1000),
                           RawIdPosDur(id=5,position=servo_data['servo5'],duration=1000),
                           RawIdPosDur(id=6,position=servo_data['servo6'],duration=1000)]
                    self.motion_publisher.publish(MultiRawIdPosDur(id_pos_dur_list=lst))
                    
                    time.sleep(1)
                    st = False
                else:
                    Board.setBusServoPulse(3, servo_data['servo3'], 20)
                    Board.setBusServoPulse(4, servo_data['servo4'], 20)
                    Board.setBusServoPulse(5, servo_data['servo5'], 20)
                    Board.setBusServoPulse(6, int(x_dis), 20)
                    
                    lst = [RawIdPosDur(id=3,position=servo_data['servo3'],duration=1000),
                           RawIdPosDur(id=4,position=servo_data['servo4'],duration=1000),
                           RawIdPosDur(id=5,position=servo_data['servo5'],duration=1000),
                           RawIdPosDur(id=6,position=servo_data['servo6'],duration=1000)]
                    self.motion_publisher.publish(MultiRawIdPosDur(id_pos_dur_list=lst))
                    
                    time.sleep(0.03)
            
           
        return img

    def publish_frame(self):
        ret, frame = self.capture.read()
        if ret:
            ros_image_msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            result = self.img_proc(frame)
            # Convert the frame to a ROS Image message using CvBridge
            ros_image_msg_result = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')

            # Publish the ROS Image message
            self.raw_publisher.publish(ros_image_msg_raw)
            
            self.processed_publisher.publish(ros_image_msg_result)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()