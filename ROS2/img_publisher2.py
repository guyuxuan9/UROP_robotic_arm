import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import yaml
import math
import numpy as np
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.raw_publisher = self.create_publisher(Image, 'video_topic/raw_image', 10)
        self.processed_publisher = self.create_publisher(Image, 'video_topic/processed_image',10)
        
        with open('/ros2/ros2_ws/src/testing/testing/lab_config2.yaml', 'r') as file:
            self.lab_data = yaml.safe_load(file)  # dictionary
            
        self.range_rgb = {  'red':   (0, 0, 255),
                            'blue':  (255, 0, 0),
                            'green': (0, 255, 0),
                            'black': (0, 0, 0),
                            'white': (255, 255, 255)}
        self.__target_color = ('blue',)
        self.timer = self.create_timer(1.0/30, self.publish_frame) # frame rate: 30fps
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)  # Open the default camera (change the index if needed)   
        
    
    
    def getAreaMaxContour(self,contours):
        """Find contour of the max area.

        Args:
            contours (_list_): collection of contours for comparison

        Returns:
           area_max_contour, contour_area_max
        """
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Filter out noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # return the max area and the corresponding contour
    
    def img_proc(self,img):
        img_h, img_w = img.shape[:2]
        
        # draw a cross at the centre
        cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)
        
        frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert to LAB space
        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0

        for i in self.lab_data:
            if i in self.__target_color:
                frame_mask = cv2.inRange(frame_lab,
                                             (self.lab_data[i]['min'][0],
                                              self.lab_data[i]['min'][1],
                                              self.lab_data[i]['min'][2]),
                                             (self.lab_data[i]['max'][0],
                                              self.lab_data[i]['max'][1],
                                              self.lab_data[i]['max'][2])) 
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) 
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  
                
                if areaMaxContour is not None:
                    if area_max > max_area: 
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        
        if max_area > 500:  
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            y = int((box[1][0]-box[0][0])/2+box[0][0])
            x = int((box[2][1]-box[0][1])/2+box[0][1])
            self.get_logger().info(f'X:{x} Y: {y}') 
            cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
           
        return img

    def publish_frame(self):
        ret, frame = self.capture.read()
        frame_resize = cv2.resize(frame, (320, 240))
        if ret:
            ros_image_msg_raw = self.bridge.cv2_to_imgmsg(frame_resize, encoding='bgr8')
            result = self.img_proc(frame_resize)
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
