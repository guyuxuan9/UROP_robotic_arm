# Color Tracking in ROS2
This is a package called **"tracking"** in my ROS2 workspace. The source code in in the *tracking/* folder. The main function in the **tracking.py** is the entry point. Type the following to run the package tracking function:
```
source install/setup.bash
```
```
ros2 run tracking tracking
```
## V1.0 -- First trial in ROS2 environment
<img width="277" alt="25f8f3c94334b9506425b63cb77b72f" src="https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/4313ff82-2099-4d94-8143-e0b116905624">

As shown above, in this tracking function, there is one node called **video_publisher**. It publishes the raw image frame to the topic **video_topic/raw_image** and the processed image frame to the topic **video_topic/processed_topic**. Additionally, it also publishes the pulse width of servo motor 3,4,5,6 to the topic **motion_topic/multi_id_pos_dur**. 


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

## V2.0 -- Separate Controller from the main tracker (but still in the same machine)
![061b9dfa035dbdbb3c78f4e70756e7d](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/5667ed4f-fc41-4fd9-982e-f62137a15f77)

As shown in the figure (nqt graph) above, this version separates the **PID_controller** from the tracker main function. */tracker* is the main function to publish **image** and **servo motor pulse width** data. Additionally, it publishes [center_x, max_area, center_y] as a list of input to the controller. Then, PID controller subscribes to that data and apply the control algorithm in a separate function called **controller.py**. The topic message type is custom designed and has the format (defined in the **Controller.msg** in the *robot_interfaces* package)
```
float32[] controller_out
```
After executing the control algorithm, the output is published to the */controller/output* topic and the message has the same format as the input. Finally, the main tracker function subscribes to this topic and receives the controller output and do the following jobs as normal.

## V3.0 Remote Controller
In this final version of the colour tracker, the controller is moved from local raspberry pi to remote (virtual) machine. The main tracker sends the controller inputs to the topic */controller/input*. The controller subcribes to this topic in my virtual machine. Then, the PID algorithm is applied and after which the output is published to the topic *controller/output*. In the meantime, the main tracker subcribes to this output topic and do the following job.

When the local controller is moved to remote machine without any tuning, it results in **unstable** behaviour due to time delay. The following video shows the unstable behaviour.

[![unstable behaviour](https://user-images.githubusercontent.com/58468284/261861421-0aa01160-fb21-4bb4-9179-810d0824028f.jpg)](https://www.youtube.com/watch?v=f9KBYwtS_7E)

Therefore, tuning needs to be done to stablise the system. As shown in the video, the robotic arm responds too quickly to the change in position. Therefore, the **P** term of the controller is decreased. Additionally, **D** term is decreased accordingly to reduce unnecessary oscillations.
The following video shows the stable behaviour after tuning.

[![remote control](https://user-images.githubusercontent.com/58468284/261861693-7eafc284-9ef3-435d-8484-70260e5420ea.png)](https://www.youtube.com/watch?v=CL-HJ2HWKag)