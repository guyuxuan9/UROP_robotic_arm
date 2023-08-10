# Video Publisher
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


## Some references:
- [Publish images to a topic in ROS2](https://automaticaddison.comgetting-started-with-opencv-in-ros-2-foxy-fitzroy-python/) 
- [rqt_image_view in ROS2](https://husarion.com/tutorials/ros2-tutorials1-ros2-introduction/)
- [Error: "can't open camera by index" in docker container](https://gitlab.com/voxl-publicvoxl-docker-images/roskinetic-docker/-/issues/4)