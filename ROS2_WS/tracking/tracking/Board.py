import serial
import time

serialHandle = serial.Serial("/dev/ttyAMA0", 9600) # Baud rate must be 9600 to communicate

def moveServo(id=None,position=None,time=None):
    buf = bytearray(b'\x55\x55')
    buf.append(8)
    buf.append(3)
    buf.append(1)
    buf.extend([(0xff & time), (0xff & (time >> 8))])   # store into buffer as the Most significant 8 bits and the Least significant 8 bits 
    buf.append(id)
    buf.extend([(0xff & position), (0xff & (position >> 8))])  
    serialHandle.write(buf)  # send

def setBusServoPulse(id, pulse, use_time):
    """
    Drive the bus servo motor to the specified position
    
    Input:
    
    id: servo id
    pulse: position
    use_time: duration
    
    Return: Null
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    moveServo(id,pulse,use_time)

if __name__ == "__main__":
    while True: 
        moveServo(1,300,500)
        time.sleep(1)
        moveServo(1,500,500)
        time.sleep(1)
