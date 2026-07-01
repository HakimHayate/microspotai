
import numpy as np

class GaitController():
    def __init__(self, solverIk, links,  
                 B=0.06, H=0.05, hz=100, duration=1):
        self.links_ = links
        self.ik_solver_ = solverIk
        self.current_positions = [0.0] * 12
        self.hz_ = hz
        self.B_ = B
        self.H_ = H

        self.coeff_height_ = np.linalg.inv(np.array([ # GOOD 
            [1, 1, 1, 1],
            [0.5**6, 0.5**5, 0.5**4, 0.5**3],
            [6, 5, 4, 3],
            [30, 20, 12, 6]
        ])) @ np.array([0, self.H_, 0, 0])

        self.t_ = 0
        self.duration_ = duration # seconds
        T = 1 / hz
        self.dt_ = 2 * T / self.duration_

    def get_foot_coordinate(self, t, thigh_foot, defaultZ=-0.15):
        if thigh_foot is None:
            thigh_foot = [0, 0, defaultZ]
        t = t % 2
        if t <= 1:
            z = self.coeff_height_[0] * t**6 + self.coeff_height_[1] * t**5 + self.coeff_height_[2] * t**4 + self.coeff_height_[3] * t**3 + thigh_foot[2]
            x =  -self.B_ / 2 + self.B_ * t + thigh_foot[0]
            
        else:
            z = thigh_foot[2]
            x = self.B_ / 2 - self.B_ * (t - 1) + thigh_foot[0]
            
        return x, thigh_foot[1], z # Keep y at 0 for stability
    
 
    def trot_gait(self, thigh_foot):
        if thigh_foot is None:
            print('Body pose not initialized')
            return None
        
        if self.t_ > 2:
            self.t_ = 0
        self.t_ += self.dt_

        command = {}

        command['front_right'] = self.get_foot_coordinate(self.t_, thigh_foot['front_right'])
        command['back_left'] = self.get_foot_coordinate(self.t_, thigh_foot['back_left'])
        
        command['front_left'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot['front_left'])
        command['back_right'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot['back_right'])
        
        return command