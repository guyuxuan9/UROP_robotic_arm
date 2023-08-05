# Overview
This is my Undergraduate Research Opportunities Program (UROP) in 2023 summer. The objective is to control the robotic arm remotely.

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

- Use **PID controllers** to track the object.

![7a6ed6d79620f30066b6d5e61e47cb9](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/585e4179-ebe4-4afc-8aab-3482d3260578)
- Tune the P,I,D values to make the robot keeps tracking the object in x, y, z three dimensions

[![Color Tracking using robotic arm](https://user-images.githubusercontent.com/58468284/257811600-6ce9408b-ddb9-4418-8195-19763f5aa868.png)](https://youtu.be/yXHFDTK_ZeQ)



## TODO:
- ACK (test delay)