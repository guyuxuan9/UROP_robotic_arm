# Setup Raspberry Pi 4 Model B

## Ubuntu 22.04 Desktop

### Raspberry Pi Imager 
- Use this [official tool](https://www.raspberrypi.com/software/) to write ubuntu desktop image to the SD card

### Download ubuntu image manually
- Download the **.xz** file and decompress it to get the **.img** file
- Use [Win32DiskImager](https://sourceforge.net/projects/win32diskimager/files/latest/download) to write the **.img** file to the SD card

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/68dd39c2-9b09-4da9-a674-f3310f89a124)

After installing the system, it's the time to configure WIFI so that the it can connect to the specified WIFI automatically when it is powered on. Follow the procedure mentioned in [this website](https://arstech.net/raspberry-pi-4-ubuntu-wifi/).

In order to remotely connect to the raspberry pi, the remote desktop setting needs to be turned on. Go to **Settings** in ubuntu and find **Sharing** on the left menu. Turn on **Remote Desktop**.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/b58b1edb-75db-4039-93f1-d7ced1d072c5)

When I first tried to connect to the desktop using VNC viewer, the following error is showed.

```
reconnecting to VNC Server...
Protocol Error: bad rectangle: 5376x0 at 0,5418 exceeds 1920x1080 at 0,0
```

This can be solved by changing the picture quality to **Medium**. Here is the [reference website](https://askubuntu.com/questions/1448924/how-do-i-get-headless-vnc-working).

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/3ef70a8e-2f37-4c30-a580-6e67fc16d352)

Finally, the raspberry pi can be accessed through VNC viewer.

## Raspian OS + Docker
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
- Setup (default) Wifi:
    ```
    sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
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

# Setup WiFi
The objective is to setup wifi id and passwords in the config file so that the raspberry pi can connect to wifi automatically when it is powered on. Additionally, when there are more than one wifi available, connect to the wifi with the highest specified priority.

First of all, the wifi setting file is */etc/wpa_supplicant/wpa_supplicant.conf*. Therefore, edit the file using the nano editor from the terminal:

```
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Now, add the following to the file:

```
network={
    ssid="wifi_A"
    psk="passwordOfA"
    priority=1 # lower priority
}
network={
   ssid="wifi_B"
   psk="passwordOfB"
   priority=2 # higher priority
}
```

The higher the value, the higher the priority is. In this case, when "wifi_A" and "wifi_B" are both available, the raspberry pi will connect to "wifi_B". 

Furthermore, if the iPhone hotspot is used, don't forget to turn on the option called **Maximise Compatibility**. The 2.4GHz and 5GHz wifi-bands are the most popular options, and 5GHz is the default one since it offers higher speed. However, raspberry pi's Wifi can only connect to 2.4GHz one, and therefore, **Maximise Compatibility** switches the Wifi to 2.4GHz band so that the raspberry pi can find it.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/6389f4e9-9a33-4f77-ab7e-aaa4c0c4b575)

After saving the file, reboot it.

```
sudo reboot
```

In addition, when the PC (windows) mobile network is turned on, there are two IP addresses associated with it. The first one is the IP address from the router. In this case, the IP address of PC from the router is 192.168.1.102. In the meantime, the PC is giving other devices 'hotspot', which means there is a subnet where the PC acts as a router. The IP address of PC inside this subnet, in this case, is 192.168.137.1. Note that the gatway device in a subnet always has the IP address of the format xxx.xxx.xxx.1.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/e5a8ba8a-cb91-41d3-be20-5cdcb6aeae37)

## Reference
- [Set up wifi ssid and psk](https://raspberrytips.com/raspberry-pi-wifi-setup/)
- [Set wifi priority](https://raspberrypi.stackexchange.com/questions/58304/how-to-set-wifi-network-priority)
- [Maximise Compatibility in iPhone hotspot](https://timesofindia.indiatimes.com/gadgets-news/explained-maximum-compatibility-option-within-personal-hotspot-on-iphone/articleshow/95916069.cms)

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
- When I rewrite the SD card, the following error occurs.  See this [link](https://blog.csdn.net/qq_44504968/article/details/105799093) for solution.
    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/da83bf66-ca1a-4a61-a5d8-e1992ab6631f)


- When I try to connect to the raspberry pi desktop to see the GUI using VNC server, the following error occurs. See this [link](https://www.youtube.com/watch?v=hA9r13ZUS08) for solution.
    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/2c1db8e1-aa6d-4808-8974-642030fb0331)

