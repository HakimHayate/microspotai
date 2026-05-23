import math

class LegIKSolver:
    def __init__(self, l1, l2):
        """
        Initializes the leg physical dimensions.
        :param l1: Length of the thigh link
        :param l2: Length of the knee/calf link
        """
        self.l1 = float(l1)
        self.l2 = float(l2)

    def solve(self, x, y, z):
        """
        Calculates the joint angles required to position the foot tip at target (x, y, z).
        Units are in meters and output radians.
        """
        d = math.sqrt(x**2 + z**2)

        gamma = math.acos((self.l1**2 + self.l2**2 - d**2)/(2 * self.l1 * self.l2))
        beta = -math.acos((self.l1**2 + d**2 - self.l2**2)/(2 * self.l1 * d))

        alpha = math.atan2(z, x)

        theta_thigh = alpha + beta
        theta_knee = math.pi - gamma

        # Hip stays horizontal while just the thigh and the knee move
        return 0, theta_thigh, theta_knee 


        