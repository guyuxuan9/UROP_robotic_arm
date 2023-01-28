## Setup Raspberry Pi 4 Model B
- Headless setup (without a monitor)
- Remotely connect to Raspberry Pi via **SSH**.
- Choose **Ubuntu 22.04 Server** OS to the microSD card 
 <img width="453" alt="image" src="https://user-images.githubusercontent.com/58468284/215291836-2443f427-e3a0-4fc3-abf2-dee3a857235c.png">
    If the prompt above is shown in the Powershell, use the following command to remove all keys belonging to the IP 


        # ssh-keygen -R <host>
        ssh-keygen -R 192.168.1.247

- [Reference1 -- Raspberry Pi headless setup](https://www.makeuseof.com/how-to-ssh-into-raspberry-pi-remote/#find-raspberry-pi-rsquo-s-ip-address)
- [Reference2 -- SSH connection problem](https://elbruno.com/2020/01/27/raspberrypi-how-to-solve-the-ssh-warning-warning-remote-host-identification-has-changed/)

## ROS2 Communication between Raspberry Pi and PC nodes via LAN
- Get IP address of PC and Raspberry pi in the LAN
- Configuring the .bashrc file to set ROS_IP
- Communication established!
![image](https://user-images.githubusercontent.com/58468284/215292400-a3644fa7-0140-462a-8dc8-143fb009c373.png)
- In the figure above, the powershell on the **right** is the **Raspberry Pi node** sending "hello world", while the terminal on the **left** is the **PC(virtual machine) node** receiving "hello world".
- [Reference](https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/)