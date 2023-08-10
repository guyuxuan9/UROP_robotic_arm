# ROS2 Messages
**bus_servo_control.py** is a node () that publishes data to a topic called **/servo_controllers/port_id_1/multi_id_pos_dur**. The format of the message sent is specified in the **MultiRawIdPosDur.msg** file which is like 
```
[ [servo_id, position, duration], [servo_id, position, duration], [servo_id, position, duration], ...]
```
This acts like a server and the following shows the running results. Here, I did not write it into a class. Instead, simple print statement is used for testing. 

![VirtualBox_Ubuntu_yuxuan_28_07_2023_21_22_41](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/5cd78734-6197-44f2-8182-01f09cb0c49c)

In addition, **test_subscriber.py** is written to test the publisher and the result is shown below. From the message displayed, node id, topic id and message received can be clearly identified.

![VirtualBox_Ubuntu_yuxuan_28_07_2023_21_25_07](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/4d822d4e-4b23-4e56-8d59-3df3257c2561)

In the meantime, I can check the current active nodes and topics in another terminal:

![VirtualBox_Ubuntu_yuxuan_28_07_2023_21_26_01](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/44c913c4-df56-4a3a-8a5d-2104560c3891)

## Reference
- [Custom msg file](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)