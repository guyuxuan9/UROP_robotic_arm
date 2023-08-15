import rclpy
from rclpy.node import Node

from robot_interfaces.msg import MultiRawIdPosDur
from robot_interfaces.msg import RawIdPosDur


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Board_subscriber')
        self.subscription = self.create_subscription(
            MultiRawIdPosDur,
            "motion_topic/multi_id_pos_dur",
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        data = msg.id_pos_dur_list
        self.get_logger().info('First position: "%s"' % data[0].position)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()