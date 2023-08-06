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

### VM VirtualBox
- Go to the **Network** section. Change the Adapter mode from **NAT** to **Bridged Adapter**

 ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7ca4d123-f177-42a9-8a43-231e83719c20)

- Here are some differences between **NAT** and **Bridged Adapter** modes.

**NAT**: the hosts translate the VM's IP address to the router for the VM so that it can connect to the internet.

![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/444a5830-0225-4c54-918c-eb21bdf5a985)


**Bridged network**: It allows a VM to network with other VMs and all physical machines on the physical network
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/bd5b94ad-f96f-4c87-8d9c-7cad6df74f8e)

In this case, the PC and the raspberry pi should be in the same LAN so that they can communicate. Therefore, bridged network is the best choice. 

After setting the network mode, it's the time to tell the server and client who they are. In the master PC (VM), type the following:

```
gedit .bashrc
```
and paste the following to this file (Change PC_IP_ADDRESS to the actual ip address):

```
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=PC_IP_ADDRESS
export ROS_IP=PC_IP_ADDRESS
```

```
source .bashrc
```

### Docker in Raspberry Pi
- Configure the network for the raspberry pi to tell it who it is.
    ```
    nano .bashrc
    ```
and paste the following to this file (Change PC_IP_ADDRESS and RASPBERRY_PI_IP_ADDRESS to the actual value):
```
export ROS_MASTER_URI=http://PC_IP_ADDRESS:11311/
export ROS_HOSTNAME=RASPBERRY_PI_IP_ADDRESS
export ROS_IP=RASPBERRY_PI_IP_ADDRESS
```

```
source .bashrc
```

- Communication established!
![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/7be2cc2a-09b6-4a52-b62e-f3af2c77a6c1)
- In the figure above, the terminal on the **right** is the **Raspberry Pi node** sending "hello world", while the terminal on the **right** is the **PC(virtual machine) node** receiving "hello world".
- Latency tests have been conducted using **Ping** statement in command line. 
    >A ping (Packet Internet or Inter-Network Groper) is a basic Internet program that allows a user to test and verify if a particular destination IP address exists and can accept requests in computer network administration.

    >Ping works by sending an Internet Control Message Protocol (ICMP) Echo Request to a specified interface on the network and waiting for a reply. When a ping command is issued, a ping signal is sent to a specified address. When the target host receives the echo request, it responds by sending an echo reply packet.

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/decaa904-8cc7-469e-90b8-a3a64d3e102d)

    ### Raspberry Pi send 64-byte data to VM

    ![0ca5f840073987506e02e28755106bb](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/eac8e2a9-fe1c-4909-adcb-db92f35f00f4)

    As shown in the figure above, the latency is measured in rtt(round trip time). It is the time taken to get a response after sending a message.

    ![image](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/6474a593-180b-4dd8-899f-b16bca3037aa)

    The average RTT is 10.850 ms

    ### VM send 64-byte data to Raspberry Pi

    ![09ee5cd3a4307ac764399e38a48dd95](https://github.com/guyuxuan9/UROP_robotic_arm/assets/58468284/cf6008e0-eab7-45f4-a1f6-e286300ee175)

    Although the average RTT in this case is greater than the previous one, it is due to some outliers (108, 110, 108). Besides these values, the mean RTT is around 10 ms which is quite close to the previous scenario.


## References
- [ROS on multiple PCs](https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/)
- [NAT vs Bridged mode](https://www.makeuseof.com/whats-the-difference-nat-bridge-host-only-network-modes/)
- [Ping statement](https://www.techtarget.com/searchnetworking/definition/ping)