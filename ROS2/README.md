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
First of all, in order to run ros2 commands, don't forget to source the setup bash file.
```
source /opt/ros/humble/setup.bash
```
To avoid doing this everytime a new terminal is opened, this command can be added to the **.bashrc** file which will run automatically when a new terminal is opened. Type the following to do so:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
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

*PS: How to enable bidirectional copy & paste in Virtual Box?*

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/fa2f5a5e-309a-4189-b338-993eb7ffff6e)

Then a CD disk image should be mounted on Linux machine. Navigate to the path where the CD image locates. In my case, the path is */media/yuxuan/VBox_GAs_7.0.10*.  Then type:
```
sudo sh ./VBoxLinuxAdditions.run
```
This will install the necessary additions and after which restart the machine
```
sudo reboot
```
After restarting the machine, select the bidirectional option under shared clipboard.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/63229ec1-bad1-4166-a246-3c82a1aee7f1)

The settings are done!

## Test for command sent to the robot
- Details at [ROS2_Messages.md](https://github.com/guyuxuan9/UROP_robotic_arm/blob/main/ROS2/ROS2_Messages.md)

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/6b4a6be9-c2bf-472d-9751-97d7449554de)


## Test for publishing video streams using rqt_image_view
- Details at [Video_Publisher.md](https://github.com/guyuxuan9/UROP_robotic_arm/blob/main/ROS2/Video_Publisher.md)

    ![ea0dfc06171fc15bed3490caf1880f3](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/8853035a-1724-42f3-bde7-8ced0abdf552)

## Test for ROS2 parameters
- Details at [ROS2_Parameters.md](https://github.com/guyuxuan9/UROP_robotic_arm/blob/main/ROS2/ROS2_Parameters.md)

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/58b5ffc3-dbbf-4e08-ac97-4d6a2b70b5de)

