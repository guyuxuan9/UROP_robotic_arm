import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher = self.create_publisher(Image, 'video_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)  # Open the default camera (change the index if needed)

    def publish_frame(self):
        ret, frame = self.capture.read()
        frame_resize = cv2.resize(frame, (320, 240))
        if ret:
            # Convert the frame to a ROS Image message using CvBridge
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame_resize, encoding='bgr8')

            # Publish the ROS Image message
            self.publisher.publish(ros_image_msg)
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
