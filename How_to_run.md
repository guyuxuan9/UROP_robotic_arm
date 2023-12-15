## VS code + VNC viewer
- Go to View/Command Palette. Select remote ssh:
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/0a68d79c-1c13-492f-9121-211d4065a40d)

- Select add new ssh host and then enter the IP address of Raspberry Pi
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7b8244b4-bfcb-4fc2-95c0-c0114c61c092)

- Open VNC viewer and go to the docker directory (Remember to do this before bringing the container online).
```
xhost local:root
```
- Bring the container online:
```
docker-compose up -d
```
- Launch Bash in Container:
```
docker exec -it docker_ros2_1 bash
```
- Source the ros2 setup file in the container if it is not sourced:
```
source /opt/ros/humble/setup.bash
```
- Run the tracker and controller in two terminals:
```
ros2 run tracking tracking
ros2 run tracking controller_out
```
There might be an error been thrown at this time:
```
No module named serial
```
To resolve this problem, installs and downloads all the latest package information available for the packages currently installed on the system. 
```
sudo apt update
```
Install and download pip3 first.
```
sudo apt-get install python3-pip
```
Install and download pyserial.
```
pip3 install pyserial
```
Now, everything should work as expected.

## How to rebuild the packages?
```
colcon build --packages-select [PACKAGE NAME]
```
Replace **[PACKAGE NAME]** with the package that has been modified.