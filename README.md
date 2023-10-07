# Overview
This is my Undergraduate Research Opportunities Program (UROP) in 2023 summer under the supervision of Prof. Thomas Parisini and Dr. Kaiwen Chen. The objective is to control the colour-tracking robotic arm remotely.

# Main work
## [Raspberry Pi setup](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/Raspberry_Pi_setup)
- Try to install different OS (Raspian and Ubuntu) to the SD card and compare them. Finally, Raspian (64-bit) is chosen.

    |        Raspian (64-bit)         |      Ubuntu (64-bit)      |
    |:-----------------------|:-----------------|
    |A docker container is needed to run ROS2                |   ROS2 can be directly installed on ubuntu            |
    | Upon connecting to WiFi,the VNC viewer can be used to instantly visualize the GUI.   | Employ remote desktop functionality, albeit with dynamically changing passwords (requires a monitor to see the password each time). 
    | Enhanced responsiveness with minimal delay when utilizing a mouse and keyboard | Significant delay observed between GUI changes and the execution of my actions|

- Configure the default WIFI connection
- Build a docker container
- Install ROS2 in the docker container

## [VM VirtualBox setup](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/ROS2)
- Install ROS2 (humble) in ubuntu 22.04
- Create ROS packages, nodes, topics to learn ROS2 basics
- [Reference (in Chinese)](https://blog.csdn.net/Amentos/article/details/127733864)
- Install VScode, type:
```
sudo snap install --classic code
```

## [Inverse Kinematics](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/Forward%26Inverse_Kinematics)
- Model the robotic arm, considering 4 DOF
- Consider different configurations and summarise them into two cases

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/5fd85552-cc9c-4477-b49c-ae81bcee92bb)

- Use geometric method to solve rotation angles of each motor based on the given end-effector coordinate and pitch angle

## [Communication of ROS nodes on multiple devices](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/RaspberryPi_PC_communication)
- Setup .bashrc files on VM and raspberry pi so that they "know" each other
- Change Network Connection mode from NAT to Bridged connection so that raspberry pi and the VM are in the same LAN 
- Write a publisher in raspberry pi docker container and a subscriber in VM. They successfully comminicate!

## [Bus servo motor control](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/RaspberryPi_roboticArm)
- Examine the servo motor communication protocols and decide to use UART protocol to send instructions from raspberry pi to the servo motor
- Follow the instruction package format and successfully move individual motor independently
  

https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/456c8372-d106-44ce-ba5e-76ee459810f1



## [Color Tracking](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/Camera)
- Use OpenCV to identify the blue object and draw a bounding box and a minimum enclosing circle around it in the live video

https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/ed90fef1-d914-4d23-9ec1-9ac7635570ad

## [Remote Control](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/ROS2_WS/tracking)

- Use **PID controllers** to track the object (More details [here](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/Camera#color-tracking)).

**x_pid**:

![55c41d88218b285b01990533fc2f970](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/71549ebd-265b-4964-96a7-b01b72d1e8b2)

**y_pid**:

![dffb8bbb5cf2e2d0c674889f27cb64d](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/1c131a4e-8722-4218-a77a-f6689bae93ad)

**z_pid**:

![0367e1b250c1cfc4ef9bdda46b77fb5](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/d8b1e7f7-07db-4de5-bbd3-9affdaf0f4b5)

**Video showing the remote control of the robotic arm** (More details [here](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/ROS2_WS/tracking#v30-remote-controller))
[![remote control](https://user-images.githubusercontent.com/58468284/261861693-7eafc284-9ef3-435d-8484-70260e5420ea.png)](https://www.youtube.com/watch?v=CL-HJ2HWKag)

## [Result Analysis](https://github.com/guyuxuan9/UROP_robotic_arm/tree/main/MatLab)
- 3D trajectory && Servo Motor Pulse Width

**Video**:
[![3D trajectory](https://user-images.githubusercontent.com/58468284/261761208-99dd782a-9e93-4c0a-a439-3c4fb1fe67ae.png)](https://youtu.be/4hUnGtIj8IU)

- Step response:

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/6d00197f-1b0b-49b6-8121-18e5eb406daf)

- $2^{nd}$ order model approximation of the kinematics of servo motor 6.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/81bb6454-8870-4937-aa39-33378e1d4633)

$$G(s) = \frac{34.03}{s^2 + 2.1s + 34.03},$$
where $\xi = 0.18$ and $\omega_n = 5.83$.



