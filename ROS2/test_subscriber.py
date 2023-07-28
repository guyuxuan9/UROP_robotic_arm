import rclpy
from rclpy.node import Node

from robot_interfaces.msg import MultiRawIdPosDur
from robot_interfaces.msg import RawIdPosDur


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MultiRawIdPosDur,
            "/servo_controllers/port_id_1/multi_id_pos_dur",
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.id_pos_dur_list)


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