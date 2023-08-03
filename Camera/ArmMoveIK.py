import time
import numpy as np
#import matplotlib.pyplot as plt
from InverseKinematics import *
from Board import setBusServoPulse

# Initialse the InverseKinematics class
ik = IK()

# Set the length of the link
l1 = ik.l1 + 0.75
l4 = ik.l4 - 0.15
ik.setLinkLength(L1=l1, L4=l4)

class ArmIK:
    # According to the datasheet, each bus servo motor can only rotate between 0 and 240 degrees and the coresponding pulse widths are 0 and 1000
    
    servo3Range = (0, 1000.0, 0, 240.0) # (pulse width min， pulse width max, angle min, angle max)
    servo4Range = (0, 1000.0, 0, 240.0)
    servo5Range = (0, 1000.0, 0, 240.0)
    servo6Range = (0, 1000.0, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        
        # Param unit: pulse_width / degree
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # transform the angle calcualted from inverse kinematics to pulse widths
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            logger.info('servo3(%s) out of range (%s, %s)', servo3, self.servo3Range[0] + 60, self.servo3Range[1])
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s) out of range (%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 - (90.0 - theta5) * self.servo5Param))
        # When theta5=0 or 180, the link l2 in purely horizontal. It cannot go beyong this value. Otherwise, the robot will be damaged.
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            logger.info('servo5(%s) out of range (%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False
        
        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param))
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s) out of range (%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime):
        """ Drive servo motor 3,4,5,6"""
        time.sleep(0.02)
        setBusServoPulse(3, servos[0], movetime)
        setBusServoPulse(4, servos[1], movetime)
        setBusServoPulse(5, servos[2], movetime)
        setBusServoPulse(6, servos[3], movetime)


    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        """Given the coordinate_data and the range of pitch angle (alpha1,alpha2), find the solution of inverse kinematics.
           If there are no solutions, return False. Otherwise, return the rotation angle of each servo motor and the pitch angle
           
           Input:
                coordinate_data (tuple): coordinates of the end-effector in cm. E.g.  (0,15,15)
                alpha1: upper/lower limit of pitch angle
                alpha2: lower/upper limit of pitch angle
                da: increment in the for loop"""
                
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime):
        """Given the coodinates, pitch angle alpha, and the range of the pitch angle (alpha1, alpha2), it will find the solution closest to the given alpha within the range and move the servo motor to that position
        
        Input:
            coordinate_data (tuple): coordinates of the end-effector in cm. E.g.  (0,15,15)
            alpha (degrees): expected pitch angle
            alpha1 (degrees): upper/lower limit of pitch angle
            alpha2 (degrees): lower/upper limit of pitch angle
            movetime (ms): duration"""
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)

        return servos, alpha, movetime


if __name__ == "__main__":
    AK = ArmIK()
    setBusServoPulse(1, 200, 500)
    setBusServoPulse(2, 500, 500)
    #AK.setPitchRangeMoving((0, 10, 10), -30, -90, 0, 2000)
    #time.sleep(2)
    print(AK.setPitchRangeMoving((-4.8, 15, 1.5), 0, -90, 0, 2000))