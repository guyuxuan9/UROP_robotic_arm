## Setup Raspberry Pi for UART communication
- Since Ubuntu is used as the OS on my raspberry pi, **raspi-config** is mannually installed to the system.


        sudo apt-get update
        sudo apt-get install raspi-config

> "By default the Raspberry Pi’s serial port is configured to be used for console input/output. Whilst this is useful if you want to login using the serial port, it means you can't use the Serial Port in your programs. To be able to use the serial port to connect and talk to other devices (e.g. Arduino), the serial port console login needs to be disabled."

Disable the login shell of serial port using **raspi-config** --> **Interface Options** --> **Serial Port Enable/Disable shell...**

- There are two types of UART available on the Raspberry Pi - PL011 (**ttyAMA0**) and mini UART (**ttyS0**). **ttyAMA0** uses an independent clock and is assigned to Bluetooth by default. **ttyS0** shares the clock with the CPU.
- The CPU clock frequency of the Raspberry Pi is not fixed like a single-chip microcomputer, but is dynamically adjusted according to the load, which will cause some problems: when the CPU frequency is changed, it may cause garbled characters on the serial port.
- Therefore, we should use **ttyAMA0** as the serial port, since it is more stable.
- By default, **serial0 (GPIO14,15)** --> **ttyS0** and **serial1 (connected to bluetooth)** --> **ttyAMA0**, so the mapping needs to be exchanged.

Before:
<img width="436" alt="58f26920386e9ef353e63b75bbaeb59" src="https://user-images.githubusercontent.com/58468284/221023833-7d03d1b2-20c1-43ac-9c26-6fd2eda5dca2.png">

After: <img width="455" alt="887d93a763d0bc5c8a3c5b074fd6525" src="https://user-images.githubusercontent.com/58468284/221023978-7964ecb1-c68f-4111-8f90-a035230d0080.png">
- This can be done by adding one line


        dtoverlay=pi3-miniuart-bt
    to /boot/firmware/config.txt (**NOTE**: in ubuntu, **config.txt** file is under the folder **/boot/firmware/**, whereas in Raspian OS, it is under the folder **/boot/**)
- Finally, connect the wires between robot control board and Raspberry Pi UART (GPIO) ports. Tx -- Rx, Rx -- Tx, and GND -- GND.
![image](https://user-images.githubusercontent.com/58468284/221024419-9c2ba4fd-3334-4b2b-ab52-db5e0f3dfbdc.png)

## Send command from Raspberry Pi to the robotic arm

- Use [WinSCP](https://winscp.net/eng/index.php) to transfer files between local PC and remote Raspberry Pi.
- **Servo_control.py** is used to send commands
- **Serial_communication_protocols.pdf** is used as a reference
- Move single servo (click to view the video)
[![](https://user-images.githubusercontent.com/58468284/221028038-96ce8b61-0999-4e8c-9f32-e5c28283bf38.png)](https://youtu.be/ESTDfcogDbU)

- Action group 1 -- clip forward (click to view the video)
[![](https://user-images.githubusercontent.com/58468284/221028351-938d961a-5ff9-4279-bb8e-eb5f72be73e8.png)](https://youtu.be/JC_90WN6Ins)



## Reference
- [Reference1 -- raspi-config installation](https://elbruno.com/2022/09/02/raspberrypi-install-raspi-config-on-ubuntu-22-04-1-lts/)
- [Reference2 -- raspberry pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)