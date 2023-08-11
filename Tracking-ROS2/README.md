# Color Tracking in ROS2
This is a package called **"tracking"** in my ROS2 workspace. The source code in in the *tracking/* folder. The main function in the **tracking.py** is the entry point. Type the following to run the package tracking function:
```
source install/setup.bash
```
```
ros2 run tracking tracking
```

As shown below, in this tracking function, there is one node called **video_publisher**. It publishes the raw image frame to the topic **video_topic/raw_image** and the processed image frame to the topic **video_topic/processed_topic**. Additionally, it also publishes the pulse width of servo motor 3,4,5,6 to the topic **motion_topic/multi_id_pos_dur**. 

<img width="277" alt="25f8f3c94334b9506425b63cb77b72f" src="https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/4313ff82-2099-4d94-8143-e0b116905624">

The message format of **motion_topic/multi_id_pos_dur** is specified in the **MultiRawIdPosDur.msg** file, which is like:

```
MultiRawIdPosDur([RawIdPosDur(id=id1,position=pos1,duration=dur1),
                  RawIdPosDur(id=id2,position=pos2,duration=dur2),
                  RawIdPosDur(id=id3,position=pos3,duration=dur3),
                  ...  ])
```
When the rqt_image_viewer is run, this is what is shown:

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/684bc08d-9052-4a49-85af-29c70c64d13d)

The red cross at the centre indicates the central position.


The following shows the topic message when the **arm_move.py** is run. The entry point of this main function is **move** (see **setup.py**).

```
ros2 run tracking move
```

In another terminal, type:
```
ros2 topic echo /motion_topic
```
This is what is shown:

<img width="960" alt="3a8fe95f484f4c1a5f26ff07ae44ad8" src="https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/0759c79c-1fa8-42eb-9c77-3df4770b610e">
