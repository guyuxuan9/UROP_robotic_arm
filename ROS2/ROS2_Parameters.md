# ROS2 Parameters
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

In order to make the code more readable and tidy, I have managed ot move some functions into another module and import it. Currently, the directory is like this:
```
testing/
├── __init__.py
├── package.xml
├── setup.py
└── src/
    ├── __init__.py
    ├── image_publisher.py
    └── lib.py
    ...

```
**lib.py** is the module that has some useful customised functions. It is imported in other files.

In the **image_publisher.py**, the import statement should be like this:

```
from testing.lib import getAreaMaxContour
```