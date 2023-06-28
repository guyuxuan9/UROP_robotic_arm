#!/usr/bin/python3
# encoding: utf-8
# Before running the program, pay attention to whether the serial port mapping relationship has been modified, 
# so that ttyAMA0 is mapped to the derived GPIO Tx Rx
import serial
import pigpio
import time
import os

FRAME_HEADER = 0x55             #frame header
CMD_SERVO_MOVE = 0x03           #servo move command
CMD_ACTION_GROUP_RUN = 0x06     #action group run command
CMD_ACTION_GROUP_STOP = 0x07    #action group stop command
CMD_ACTION_GROUP_SPEED = 0x0B   #set action group speed command
CMD_GET_BATTERY_VOLTAGE = 0x0F  #get battery voltage command

open_io="sudo pigpiod"
os.system(open_io)
time.sleep(1)
pi = pigpio.pi()  # initialise pigpio module
serialHandle = serial.Serial("/dev/ttyAMA0", 9600)  # initialise serial port， baud rate = 9600
 


def moveServo(id=None,position=None,time=None):
    buf = bytearray(b'\x55\x55')
    buf.append(8)
    buf.append(3)
    buf.append(1)
    buf.extend([(0xff & time), (0xff & (time >> 8))])   # store into buffer as the Most significant 8 bits and the Least significant 8 bits 
    buf.append(id)
    buf.extend([(0xff & position), (0xff & (position >> 8))])  
    serialHandle.write(buf)  # send


 # Function:     moveServos
 # Description： move multiple servos
 # Parameters:   Num: Number of servos,time:moving time,*List:servo ID,moving angle,servo ID,moving angle,..... so on
 # Return:       void
def moveServos(Num = None,time = None,*List):
    buf = bytearray(b'\x55\x55')
    buf.append(Num*3+5)
    buf.append(3)
    buf.append(Num)
    buf.extend([(0xff & time), (0xff & (time >> 8))])   
    for i in range(0,len(List)-1,2):
        id = List[i]
        position = List[i+1]
        buf.append(0xff & id)
        buf.extend([(0xff & position), (0xff & (position >> 8))])   
    #for i in buf:
    #    print('%x' %i)
    serialHandle.write(buf)  # send


# Function:     runActionGroup
# Description： run action group
# Parameters:   NumOfAction:action group ID, Times:number of times
# Return:       void
# Others:       Times = 0 loop forever
def runActionGroup(ID = None,time=None):
    buf = bytearray(b'\x55\x55')
    buf.append(5)
    buf.append(6)
    buf.append(ID)
    buf.extend([(0xff & time), (0xff & (time >> 8))])   
    serialHandle.write(buf)  # send

    
if __name__ == '__main__':
    while True: 
        
        moveServo(1,300,500)
        time.sleep(1)
        moveServo(1,500,500)
        time.sleep(1)
        # runActionGroup(1,1000)