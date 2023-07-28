# ROS2 installation
- According to [This](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) official document, I have installed **ROS 2 Humble Hawksbill** on **Ubuntu WSL**(Windows Subsystem for Linux) - Jammy Jellyfish (22.04)
- During installation, I have encountered some problems and fixed the bugs:
    - E.g. This error message always showed up when I tried to re-installed ros2
    <img width="854" alt="cc038ed74636626ebb913a8b2238c1e" src="https://user-images.githubusercontent.com/58468284/210111009-4a8c45a0-0fab-46a3-afd4-4562c972a47d.png">
    
    This is solved by deleting **20snapd.conf** according to [This website](https://github.com/microsoft/WSL/issues/4640) described.
    - E.g. "The repository does not have a release file"
    <img width="748" alt="6af1b78b30c8235190a7508306e98bf" src="https://user-images.githubusercontent.com/58468284/210111425-a2511044-e536-4d5b-b129-b61232ba1217.png">
    
    This is solved by removing **.list** file from **/etc/apt/sources.list.d** according to [This website](https://answers.ros.org/question/402151/the-repository-does-not-have-a-release-file/) described.

- After installing **ros-humble-desktop**, I have tried the **talker-listener** example.
<img width="543" alt="e26f2c961d5e817c9e7ae09ff3763bd" src="https://user-images.githubusercontent.com/58468284/210111636-d7805080-f0f8-424c-aa78-3ff648f38468.png">
<img width="529" alt="3b65c860de27ed571c603663ab892ac" src="https://user-images.githubusercontent.com/58468284/210111657-3fe29041-9676-4a43-906d-de6b3f2716c0.png">
This verifies both the C++ and Python APIs are working properly.

# ROS2 workspace
Use the following command to create a ros2 workspace. I make the workspace in the Documents/ folder
```
cd ~/Documents
mkdir ros2_ws
```
After creating a workspace, packages need to be created.
> A package is an organizational unit for your ROS 2 code.

The structure of a ros2 workspace is like this:
```
├── ros2_ws
     ├── src
         ├── package_1
             ├── package.xml
             ├── setup.cfg
             ├── setup.py
             ├── package_1/
             ├── resource/package_1  

         ├── package_2
             ├── package.xml
             ├── setup.cfg
             ├── setup.py
             ├── package_2/
             ├── resource/package_2

         ...

     ├── install
     ├── build
     ├── log               

```

Before writing the source code, remember to write some custom msg or srv files if necessary. Please refer to [This website](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) for details.

The (python) package is built using the command:
```
ros2 pkg create --build-type ament_python PACKAGE_NAME --dependencies DEPENDENCY1 DEPENDENCY2
```
All the packages are in the ros2_ws/src folder. 
- **package.xml** file contains meta information about the package. E.g. the dependencies specified in the create package command are listed in this file
- **setup.py** contains instructions for how to install the package. E.g. the entry point of the file
- **<package_name>/** is a directory with the same name as the package, used by ROS 2 tools to find the package. It contains **__init__.py** and all the **source codes**
- **setup.cfg** is required when a package has executables, so ros2 run can find them

After writing the source code in the *ros2_ws/src/<package_name>/<package_name>/* and adding the entry point in the **setup.py** file, it's the time to build the package.

**Make sure to go back to the root of ros2 workspace first!!**

```
cd ~/....../ros2_ws
colcon build --packages-select <PACKAGE_NAME>
```
Then, the folders *ros2_ws/install*, *ros2_ws/build*, *ros2_ws/log* will be generated/updated.

```
source install/setup.bash
```
```
ros2 run <PACKAGE_NAME> NAME_SPECIFIED_AT_THE_ENTRY_POINT
```
## Test for command sent to the robot
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
