# 4DOF robotic arm inverse kinematics 
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # servo motor ID is counted from the bottom
    # length of each link
    l1 = 6.10    # distance from the centre of the base to the second motor
    l2 = 10.16   # distance from the second motor to the third one
    l3 = 9.64    # distance from the third motor to the fourth one
    l4 = 16.65   # distance from the fourth motor to the end effector


    def __init__(self): 
        print("Initialise an InverseKinematics class.")

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # set the length mannually
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):  
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):        
        """This function takes in the expected coordinates and pitch angle and calculates the rotation angle of each joint based on the inverse kinematics
        
        Inputs:
            coodinates_data (tuple): coordinates of the end-effector in cm. E.g. (0,15,15)
            Alpha: pitch angle of the end-effector in degrees

        Returns:
            __dictionary__: rotation angle of each joint: {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} 
            __bool__: False (if there is no solution)
        """
        
        # Let the coodinates of the end-effector be P(x_p, y_p, z_p) and the origin be O. P' is the projection of P on the x-y plane
        # Let the intersection point of l1 and l2 be A, 12 and l3 be B, l3 and l4 be C
        # CD is perpendicular to PD. CD is perpendicular to z-axis. Alpha is the angle between DC and PC
        # AE is perpendicular to DP' and E is on DP'
        # CF is perpendicular to AE and F is on AE

        X, Y, Z = coordinate_data

        # Find the rotation angle of the base
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) # distance from P' to the origin O
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) # When the pitch angle is positive, PD is positive and vice versa
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        
        
        # print("P_O: ",P_O, "\nCD: ",CD,"\nPD: ",PD, "\nAF: ",AF,"\nCF: ",CF)
        # print("AC: ",AC)
        if round(CF, 4) < -self.l1: # point C will be below ground level
            logger.debug('Height is less than 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # In triangle ABC, the sum of two sides is less than the third side
            logger.debug('Cannot form a triangle, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        # Find theat4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) # cosine rule
        if abs(cos_ABC) > 1:
            logger.debug('Angle ABC does not exist, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC)
        theta4 = 180.0 - degrees(ABC)

        # Find theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # cosine rule
        if abs(cos_BAC) > 1:
            logger.debug('Angle BAC does not exist, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        # Find theta3
        theta3 = Alpha - theta5 + theta4

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} 
            
if __name__ == '__main__':
    ik = IK()
    ik.setLinkLength(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    print('Link lengths: ', ik.getLinkLength())
    # print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
    print(ik.getRotationAngle((0, 15, 15), 18))