# Setup Raspberry Pi 4 Model B
- [Tutorial website](https://www.kevsrobots.com/learn/learn_ros/02_pi_setup.html), [tutorial video](https://www.youtube.com/watch?v=03wKo-riJlA)
- Connect to Raspberry Pi:
    - **SSH**: open a terminal, type 
    ```
    ssh USERNAME@ip_address
    ```
    and replace **USERNAME** by the actual username and **ip_address** by the acutal raspberry pi ip address. (I use hotspot from the PC and ip address in shown there)
    - **Remote desktop**: install **xrdp** on raspberry pi. **XRDP** is an open-source remote desktop protocol server, which allows you to connect to the Linux desktop from any operating system
    ```
    sudo apt-get install xrdp
    ```
- >Raspberry Pi OS is based on Debian which receives Tier 3 support, but it can run Ubuntu docker containers for Tier 1 support.

    In this setup, instead of using Ubuntu OS, Raspberry Pi OS (64 bits) is used and ROS is running within a **Docker container**.
- **How to build a docker container?**
    - Change this section of the **docker-compose.yml** file. This maps the folder in raspberry pi to the docker container.
    ```
        volumes:
      - /home/yuxuan/ros:/home/ros
      - /home/yuxuan/Documents/UROP_robotic_arm/Raspberry_Pi_setup:/ros2
    ```
    - Change directory to docker/ and run the following to build the container:
    ```
    docker build -t ros2 .
    ```
    - Start the container:
    ```
    docker-compose up -d
    ```
- After building the ros2 container, the **talker.py** can be executed and the following is printed in the terminal in VS code:

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/405c50c3-bdef-4757-9072-c8736fe6f8b3)
- After running **talker2.py**, the following is printed:

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/06793fc0-707e-471e-8772-0a1ba85f1c07)

# Create packages, publisher and subscriber
- **pub.py** publishes "Hello World" to the topic "/Hello"
- **sub.py** subscribes to the topic "/Hello"
- In the **setup.py**, the following entry points are specified:
```
    entry_points={
        'console_scripts': [
        "pub = my_py_pkg.pub:main",
        "talker = my_py_pkg.talker:main",
        "listener = my_py_pkg.sub:main"
        ],
    },
```
- Therefore, after running **pub** and **listener**, the following is printed:

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7610c173-5ed2-405e-868f-4fc0ca5e0f38)

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/1e33b673-0ba2-40de-bacf-b977963c3daf)

- The information of topic "/Hello" can be viewed as well in another terminal:

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/14c37f8f-54dc-47d8-b2b1-47676a23cc8b)



# Troubleshooting
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/da83bf66-ca1a-4a61-a5d8-e1992ab6631f)

-  See this [link](https://blog.csdn.net/qq_44504968/article/details/105799093) for solution
