## Setup Raspberry Pi 4 Model B
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
- After building the ros2 container, the **talker.py** can be executed and the following is printed in the terminal in VS code:
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/405c50c3-bdef-4757-9072-c8736fe6f8b3)



### Troubleshooting
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/da83bf66-ca1a-4a61-a5d8-e1992ab6631f)

-  See this [link](https://blog.csdn.net/qq_44504968/article/details/105799093) for solution
