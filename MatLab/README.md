# Play back data using ROS2 bag and MatLab

**ROS2 bag** is a command line tool for recording data published on topics in the system. **ROS Toolbox** is a interface between MatLab and ROS2. The objective is to record the data in ROS2 and analyse it in MatLab.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/601c1ecb-228e-452c-9677-684efd5466d8)

## ROS2

To record the data published to a topic, simply type the commands (replace <TOPIC_NAME> with actual topic name):

```
ros2 bag record <TOPIC_NAME>
```

type Ctrl+C to stop recording, and after which the ros bag files will be stored in the current working directory. Inside the stored ros bag folder, there are two files: **.db3** and **metadata.yaml**. **.db3** is the data base file that stores the data while **metadata.yaml** is a config file that stores some information of the database, e.g. topic recorded, message type etc.

The bag folder name can be customised by adding **-o** right after the word "record". Meanwile, multiple topics messages can be recorded. E.g.

```
ros2 bag record -o <BAG_FOLDER_NAME> <TOPIC_1> <TOPIC_2>
```

### TroubleShooting
The following error message occurs after typing ros2 bag record command:

```
root@raspberrypi:/ros2/ros2_ws/bag_files# ros2 bag record /motion_topic/multi_id_pos_dur
Failed to load entry point 'record': /opt/ros/humble/lib/librosbag2_transport.so: undefined symbol: _ZN6rclcpp5Clock7startedEv
Traceback (most recent call last):
  File "/opt/ros/humble/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.18.6', 'console_scripts', 'ros2')())
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/cli.py", line 50, in main
    add_subparsers_on_demand(
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/command/__init__.py", line 250, in add_subparsers_on_demand
    extension.add_arguments(
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2bag/command/bag.py", line 26, in add_arguments
    add_subparsers_on_demand(
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/command/__init__.py", line 237, in add_subparsers_on_demand
    extension = command_extensions[name]
KeyError: 'record'
```
**Reason**: the system does not recognise the 'record' keyword

**Solution**: update the package list in linux system and install the most recent packages.
```
sudo apt update && sudo apt upgrade
```
After updating, this error message no longer appears.

## Visual Studio C++
Download the [Visual Studio Community 2022](https://visualstudio.microsoft.com/zh-hant/vs/community/). Follow the default setting in the installer. After installation, the details should be as below:

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7810a771-1899-4efc-b943-85108a974024)

Don't forget to click **Windows 11 SDK** and **Develop with C++ desktop**!!

## MabLab
According to [ROS Toolbox System Requirements](https://au.mathworks.com/help/ros/gs/ros-system-requirements.html), I choose to install the following versions of the softwares.

|          Software            | Version |
|:----------------------------:|:-------:|
| Matlab                       | R2023a  |
| Visual Studio (C++ Compiler) | 2022    |
| Python                       | 3.8     |
|                              |         |

In the first time of using the ROS Toolbox, remember to create the python environment. In the top menu, go to *Home/Environment/Preference*, find *ROS Toolbox*, click *Open ROS Toolbox Preferences*. Then, fill in the python executable path and click *Recreate Python Environment*. 

Since **MinGW64 Compiler (C++)** cannot be used to build ROS packages, the **Microsoft Visual C++ 2022** is chosen. To change the C++ compiler, in the Matlab command line, type 
```
mex -setup cpp
```
Then, follow the instruction to change the compiler.



