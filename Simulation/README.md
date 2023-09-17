# Build visual robot model with URDF
- Install the **urdf_tutorial** package:
    ```
    sudo apt install ros-humble-urdf-tutorial
    ```
- Navigate to the package path:
    ```
    cd /opt/ros/humble/share/urdf_tutorial
    ```

    and run the commands to see the urdf model in rviz. For example:
    ```
    ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
    ``` 

## Reference
- [Official website of ROS2 URDF](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)