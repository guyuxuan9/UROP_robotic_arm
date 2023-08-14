import rclpy
from rclpy.node import Node

from tracking.lib import *
import tracking.PID as PID

from robot_interfaces.msg import Controller


x_dis = 500
y_dis = 10
Z_DIS = 18
z_dis = Z_DIS
x_pid = PID.PID(P=0.1, I=0.00, D=0.008) 
y_pid = PID.PID(P=0.0001, I=0, D=0)
z_pid = PID.PID(P=0.005, I=0, D=0)

class ControllerClass(Node):
    def __init__(self):
        super().__init__('PID_controller')        
        self.controller_publisher = self.create_publisher(Controller, 'controller/output',10)
        self.subscription = self.create_subscription(
            Controller,
            'controller/input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        
    def listener_callback(self, msg):
        global x_dis, y_dis, z_dis
        
        img_h = 480
        img_w = 640
        
        # x_input: centre_x, y_input: area_max, z_input: centre_y
        x_input, y_input, z_input = msg.controller_out 

        x_pid.SetPoint = img_w / 2.0  
        x_pid.update(x_input)  
        dx = x_pid.output
        x_dis += int(dx)  
        x_dis = 0 if x_dis < 0 else x_dis
        x_dis = 1000 if x_dis > 1000 else x_dis
        
        y_pid.SetPoint = 9000  
        if abs(y_input - 9000) < 50:
            y_input = 9000
        y_pid.update(y_input)  
        dy = y_pid.output
        y_dis += dy  
        y_dis = 5.00 if y_dis < 5.00 else y_dis
        y_dis = 10.00 if y_dis > 10.00 else y_dis
        
        if abs(z_input - img_h/2.0) < 20:
            z_pid.SetPoint = z_input
        else:
            z_pid.SetPoint = img_h / 2.0
            
        z_pid.update(z_input)
        dy = z_pid.output
        z_dis += dy
        z_dis = 32.00 if z_dis > 32.00 else z_dis
        z_dis = 10.00 if z_dis < 10.00 else z_dis
        
        self.controller_publisher.publish(Controller(controller_out=[x_dis, y_dis, z_dis]))
        self.get_logger().info('x_dis: "%s" y_dis: "%s", z_dis: "%s"' % (x_dis,y_dis,z_dis))
        
    #     self.timer = self.create_timer(1.0/30, self.publish_controller_output) # frame rate: 30fps          
    
    # def publish_controller_output(self):
        
    

def main(args=None):
    rclpy.init(args=args)
    node = ControllerClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
