import numpy as np

class RotateController():
    def __init__(self, solverIk, links, T_base_thigh, T_world_base,  
                 B=0.02, H=0.03, hz=100, duration=1):
        self.links_ = links
        self.ik_solver_ = solverIk
        self.current_positions = [0.0] * 12
        self.hz_ = hz
        self.B_ = B
        self.H_ = H
        self.T_base_thigh_ = T_base_thigh
        self.T_world_base_ = T_world_base

        self.coeff_height_ = np.linalg.inv(np.array([ 
            [1, 1, 1, 1],
            [0.5**6, 0.5**5, 0.5**4, 0.5**3],
            [6, 5, 4, 3],
            [30, 20, 12, 6]
        ])) @ np.array([0, self.H_, 0, 0])

        self.t_ = 0
        self.duration_ = duration # seconds
        T = 1 / hz
        self.dt_ = 2 * T / self.duration_
        

    def get_foot_coordinate(self, t, thigh_foot, link):
        t = t % 2
        
        T_world_thigh = self.T_world_base_ @ self.T_base_thigh_[link]
        world_foot = T_world_thigh[:3, -1] + thigh_foot[link]
        Tang = np.cross(np.array([0, 0, 1]), world_foot)
        norm = np.sqrt(Tang[0]**2 + Tang[1]**2 + Tang[2]**2)

        Tx = Tang[0] / norm
        Ty = Tang[1] / norm

        if t <= 1:
            x =  Tx * (-self.B_ / 2 + self.B_ * t) + thigh_foot[link][0]
            y =  Ty * (-self.B_ / 2 + self.B_ * t) + thigh_foot[link][1]
            z = self.coeff_height_[0] * t**6 + self.coeff_height_[1] * t**5 + self.coeff_height_[2] * t**4 + self.coeff_height_[3] * t**3 + thigh_foot[link][2]

            
        else:
            x = Tx * (self.B_ / 2 - self.B_ * (t - 1)) + thigh_foot[link][0]
            y = Ty * (self.B_ / 2 - self.B_ * (t - 1)) + thigh_foot[link][1]
            z = thigh_foot[link][2]

        return x, y, z 
    
    def update_base(self, T_world_base):
        print(T_world_base)
        self.T_world_base_ = T_world_base


    def rotate(self, thigh_foot, T_world_base):
        if thigh_foot is None:
            print('Body pose not initialized')
            return None
        self.update_base(T_world_base)
        self.t_ = (self.t_ + self.dt_) % 2.0

        command = {}

        command['front_right'] = self.get_foot_coordinate(self.t_, thigh_foot, 'front_right')
        command['back_left'] = self.get_foot_coordinate(self.t_, thigh_foot, 'back_left')
        
        command['front_left'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot, 'front_left')
        command['back_right'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot, 'back_right')
        
        return command