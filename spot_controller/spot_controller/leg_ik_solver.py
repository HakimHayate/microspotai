import math
import numpy as np

# z
# ^
# |
# |
#  ----> y

class LegIKSolver:
    def __init__(self, l1, l2, l3):
        """
        l1: hip offset
        l2: thigh offset
        l2: knee offset
        """
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.l3 = float(l3)

    def solve(self, x, y, z):
        theta_hip = math.atan2(y, math.sqrt(z**2 + x**2))

        d = math.sqrt(z**2 + x**2 + y**2) 

        gamma = math.acos(np.clip((self.l2**2 + self.l3**2 - d**2)/(2 * self.l2 * self.l3), -1.0, 1.0))
        beta = -math.acos(np.clip((self.l2**2 + d**2 - self.l3**2)/(2 * self.l2 * d), -1.0, 1.0))

        alpha = math.atan2(x, -z)
        theta_thigh = beta + alpha
        theta_knee = math.pi - gamma

        return theta_hip, theta_thigh, theta_knee 


        