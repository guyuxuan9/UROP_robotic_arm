## ROS2 installation
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
