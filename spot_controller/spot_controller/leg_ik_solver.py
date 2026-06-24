import math
import numpy as np

class LegIKSolver:
    def __init__(self, l1, l2, l3):
        """
        Initializes the leg physical dimensions.
        param l1: hip offset
        param l2: Length of the thigh link
        param l2: Length of the knee/calf link
        """
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.l3 = float(l3)

    def solve(self, x, y, z):
        """
        Calculates the joint angles required to position the foot tip at target (x, y, z).
        Units are in meters and output radians.
        """

        theta_hip = math.atan2(y, x)

        d = math.sqrt(x**2 + y**2 + z**2) 

        gamma = math.acos(np.clip((self.l2**2 + self.l3**2 - d**2)/(2 * self.l2 * self.l3), -1.0, 1.0))
        beta = -math.acos(np.clip((self.l2**2 + d**2 - self.l3**2)/(2 * self.l2 * d), -1.0, 1.0))

        alpha = math.atan2(z, x)

        theta_thigh = alpha + beta
        theta_knee = math.pi - gamma

        return theta_hip, theta_thigh, theta_knee 


        