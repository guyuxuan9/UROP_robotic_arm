import time
from ArmMoveIK import *

if __name__ == "__main__":
    AK = ArmIK()
    
    AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    time.sleep(1)
    AK.setPitchRangeMoving((0, 15, 15), -45, -90, 0, 1000)
    time.sleep(1)
    AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    time.sleep(1)    
    
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((-15, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((15, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)

    # AK.setPitchRangeMoving((0, 25, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 10, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)

    # AK.setPitchRangeMoving((0, 15, 25), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 5), 0, -90, 0, 1000)
    # time.sleep(1)
    # AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 1000)

