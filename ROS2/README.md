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

## Test for publishing video streams using rqt_image_view
The aim of this test is to show that the live video stream captured by the camera on the robotic arm can be viewed in the rqt_image_viewer in ros2.

First of all, change the **dockerfile** from ros core to desktop version, so that necessary packages are installed. This can be done by changing the *Install ros2 package* section 

from

```
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
```

to

```
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
```
**image_publisher.py** is the source file to continuously publish video frames to the topic *video_topic*. The publisher message type is described in **Image.msg** from the package **sensor_msgs.msg**. Please refer to the [code from official website](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg). The **Image.msg** file can also be found locally in the docker container:

![713dc1516859a01bd10ce2ed6c1204e](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/5333c233-1593-4e6f-93ae-d65c88a69c9c)

Additionally, **cv_bridge** library is used to convert between ROS Image messages and OpenCV images. The figure below illustrates this concept:

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7fe66b6c-1627-49dc-a716-f88f888e90c4)

Furthermore, the dependency **image_transport** is added to the dependency list. 

>It is a package that provides a flexible and efficient way to transport images and videos over the ROS communication infrastructure. It's designed to optimize the transmission of image data, reducing the overhead associated with sending images between nodes.     

Overall, the following dependencies are added to the dependency list in the **package.xml** file:
- **rclpy** (to use ros2 in python)
- **image_transport** (standard to transport images in ros2 infrasturcture)
- **cv_bridge** (convert between ros2 Image msg and opencv image)
- **sensor_msgs** (to include Image.msg file)
- **std_msgs** (standard message types for basic data types, e.g. Int, String)
- **opencv2** (image processing lib)

Before running the ros2 package, don't forget to map the camera device from outside to inside the container. This can be done by adding the following lines:

```
    devices:
      - /dev/video0:/dev/video0
```

Finally, in order to make GUI work in the docker container, we need to redirect any graphics calls to outside our container into the hosts’s X11 environment.  This can be done by following the following procedure:

```
xhost local:root
```

```
docker-compose up -d
```

```
docker exec -it IMAGE_NAMES bash
```

Now, we are in the docker container. Build the package. Run the package. Then in another terminal in the VNC viewer, type the following in the docker container:

```
ros2 run rqt_image_view rqt_image_view
```
and select the video_topic. Then, the live video stream should be shown:

![ea0dfc06171fc15bed3490caf1880f3](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/8853035a-1724-42f3-bde7-8ced0abdf552)


#### Some references:
- [Publish images to a topic in ROS2](https://automaticaddison.comgetting-started-with-opencv-in-ros-2-foxy-fitzroy-python/) 
- [rqt_image_view in ROS2](https://husarion.com/tutorials/ros2-tutorials1-ros2-introduction/)
- [Error: "can't open camera by index" in docker container](https://gitlab.com/voxl-publicvoxl-docker-images/roskinetic-docker/-/issues/4)

## Test for ROS2 parameters
My initial aim is to create a node that reads the **LAB** ranges from a **yaml** config file and store them as its parameters for the colour detection. **paramNode.py** file is the test file created for this objective.

Note that the path to the *yaml* file is the path in the docker container. Therefore, in my case, it starts with */ros2/ros2_ws/*. The **lab_config.yaml** file is like this:

```
color_range_list:
  blue:
    min: [66, 0, 28]
    max: [159, 179, 104]
```

Hence, two parameters are declared, which are **color_range_list.blue.max** and **color_range_list.blue.min** respectively. The parameter type is List. After running this node, the following is shown in the terminal:

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/88a57f45-5bd3-4888-98ca-c1ca92d53cf0)

The active parameter can also be viewed in another terminal. Also, the value of the parameter can be accessed as well.

![9e2698df3ac6d62ff804123387efe55](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/818984d0-43ec-42f8-8f82-71f533e9f44e)

Howerver, when I try to access the parameter from another node, it seems that this is not allowed in ROS2.

|                   ROS1                        | ROS2|
|:----------------------------------------------|:------------------------------|
|Parameters are handled by the parameter server (ROS master)|No global parameter server|
|Parameters are global and can be accessed by any node in the system | Parameters are defined as node-specific |

**ROS1:**
>rosparam allows you to store and manipulate data on the ROS Parameter Server. 

**ROS2:**
>A parameter is a configuration value of a node. You can think of parameters as node settings.

Therefore, I gave up using a param node to store the **LAB** ranges. Instead, a member variable called **lab_data** is declared in the class to store the **LAB** ranges. Additionally, the image processing part has been added to the class as a member function called **img_proc**. Finally, the processed image and raw image are published to the topics *video_topic/processed_image* and */video_topic/raw_image* respectively at a frame rate of 30fps. The full code can be viewed in **img_publisher2.py**. The node graph is drawn by rqt below:

![81a2980607193598451e07262ad1064](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/11525074-b3e6-4605-ab44-3990aebb2bc2)

If the *video_topic/processed_image* topic is selected, the following is shown in the rqt_image_viewer

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/58b5ffc3-dbbf-4e08-ac97-4d6a2b70b5de)