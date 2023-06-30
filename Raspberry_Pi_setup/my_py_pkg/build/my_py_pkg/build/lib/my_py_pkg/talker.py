import rclpy
from rclpy.node import Node

class Talker(Node): #inherit from Node
    def __init__(self):     # initialisation(constructor)
        super().__init__("node_test") # super() refers to the base class
        self._counter = 0
        self.get_logger().info("Hello world")
        self.create_timer(0.5, self.timer_callback) # call the function every 0.5s, the function is defined later
    
    def timer_callback(self):
        self._counter += 1
        self.get_logger().info("Hello"+str(self._counter))
    
def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()