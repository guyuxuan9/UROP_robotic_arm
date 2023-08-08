import rclpy
from rclpy.node import Node
import yaml

class ColorRangeNode(Node):
    def __init__(self):
        super().__init__('color_range_node')

        with open('/ros2/ros2_ws/src/testing/testing/lab_config.yaml', 'r') as file:
            color_range_data = yaml.safe_load(file)

        for color_name, color_values in color_range_data['color_range_list'].items():
            self.declare_parameter(f'color_range_list.{color_name}.min', color_values['min'])
            self.declare_parameter(f'color_range_list.{color_name}.max', color_values['max'])

        self.get_logger().info('Color range parameters set.')

        
def main(args=None):
    rclpy.init(args=args)
    node = ColorRangeNode()
    blue_min_values = node.get_parameter('color_range_list.blue.min').value
    blue_max_values = node.get_parameter('color_range_list.blue.max').value
    
    # get_parameter tests
    print(blue_min_values)
    print("blue L: [",blue_min_values[0],blue_max_values[0],"]")
    print("blue A: [",blue_min_values[1],blue_max_values[1],"]")
    print("blue B: [",blue_min_values[2],blue_max_values[2],"]")
    
    rclpy.spin(node) 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
