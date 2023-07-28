from robot_interfaces.msg import MultiRawIdPosDur
from robot_interfaces.msg import RawIdPosDur

import signal
import rclpy
import time

running = True

def shutdown(signum,frame):
    global running
    running = False
    print("shutdown")
    
signal.signal(signal.SIGINT, shutdown) # when Ctrl+C is pressed

def set_servos(pub, duration, pos_s):
    lst = []
    for pos in pos_s:
        lst.append(RawIdPosDur(id=pos[0], position=pos[1], duration=duration))
        #print(pos)
    msg = MultiRawIdPosDur(id_pos_dur_list = lst)
    pub.publish(msg)
    
def main():
    rclpy.init()
    node = rclpy.create_node("test_servo_control")
    
    joints_pub = node.create_publisher(MultiRawIdPosDur, "/servo_controllers/port_id_1/multi_id_pos_dur",1)
    
    while running:
        try:
            set_servos(joints_pub, 1000, ((6,350),(1,200)))
            time.sleep(1)
            set_servos(joints_pub, 1000, ((6,650),(1,500)))
            time.sleep(1)
        except Exception as e:
            print(e)
            break
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()