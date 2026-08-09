import numpy as np

class GaitController():
    def __init__(self, solverIk, links,  
                 B=0.08, H=0.06, hz=50, duration=2.0):
        self.links_ = links
        self.ik_solver_ = solverIk
        self.current_positions = [0.0] * 12
        self.hz_ = hz
        self.B_ = B
        self.H_ = H
        self.swing_time_ = 1.0 # Default 1.0 means a 50/50 trot cycle

        self.coeff_height_ = np.linalg.inv(np.array([ 
            [1, 1, 1, 1],
            [0.5**6, 0.5**5, 0.5**4, 0.5**3],
            [6, 5, 4, 3],
            [30, 20, 12, 6]
        ])) @ np.array([0, self.H_, 0, 0])

        self.t_ = 0
        self.duration_ = duration # seconds
        T = 1 / hz
        # Trot uses a 2.0 cycle multiplier
        self.dt_ = 2.0 * T / self.duration_

    def get_variable_trot_coordinate(self, t, thigh_foot, swing_time=1.0):
        # The cycle goes from 0.0 to 2.0
        t = t % 2.0
        
        if t <= swing_time: 
            # SWING PHASE (Fast)
            # Map t to a 0.0 -> 1.0 percentage based on the swing_time
            normalized_t = t / swing_time 
            
            z = self.coeff_height_[0] * normalized_t**6 + self.coeff_height_[1] * normalized_t**5 + \
                self.coeff_height_[2] * normalized_t**4 + self.coeff_height_[3] * normalized_t**3 + thigh_foot[2]
            
            # Cosine smoothing to brake right before touchdown
            swing_progress = (1 - np.cos(np.pi * normalized_t)) / 2.0
            x = -self.B_ / 2 + self.B_ * swing_progress + thigh_foot[0]
            
        else: 
            # STANCE PHASE (Slow and stable)
            z = thigh_foot[2]
            
            # Map the remaining time to a 0.0 -> 1.0 percentage
            stance_time_total = 2.0 - swing_time
            stance_progress = (t - swing_time) / stance_time_total 
            
            x = self.B_ / 2 - self.B_ * stance_progress + thigh_foot[0]
            
        y = thigh_foot[1]
        
        return x, y, z

    def trot_gait(self, thigh_foot):
        if thigh_foot is None:
            print('Body pose not initialized')
            return None
            
        # Time step calculation for a 2-part cycle
        self.dt_ = 2.0 / (self.hz_ * self.duration_) 
        self.t_ = (self.t_ + self.dt_) % 2.0

        command = {}
        
        # Diagonal pairs move at the exact same time
        # Pair 1: Front-Right and Back-Left
        command['front_right'] = self.get_variable_trot_coordinate(self.t_, thigh_foot['front_right'], self.swing_time_)
        command['back_left'] = self.get_variable_trot_coordinate(self.t_, thigh_foot['back_left'], self.swing_time_)
        
        # Pair 2: Front-Left and Back-Right (offset by exactly half the cycle: 1.0)
        command['front_left'] = self.get_variable_trot_coordinate((self.t_ + 1.0) % 2.0, thigh_foot['front_left'], self.swing_time_)
        command['back_right'] = self.get_variable_trot_coordinate((self.t_ + 1.0) % 2.0, thigh_foot['back_right'], self.swing_time_)
        
        return command

    def update_parameters(self, B, H, duration, swing_time):
        """Dynamically updates gait parameters from the GUI."""
        self.B_ = B
        self.duration_ = duration
        self.swing_time_ = swing_time

        # Only recalculate the expensive matrix math if the height actually changed
        if self.H_ != H:
            self.H_ = H
            # Recalculate polynomial coefficients based on the new H value
            self.coeff_height_ = np.linalg.inv(np.array([ 
                [1, 1, 1, 1],
                [0.5**6, 0.5**5, 0.5**4, 0.5**3],
                [6, 5, 4, 3],
                [30, 20, 12, 6]
            ])) @ np.array([0, self.H_, 0, 0])
            
        # Recalculate time step based on the new duration
        T = 1 / self.hz_
        
        # 2.0 cycle multiplier for Trot
        self.dt_ = 2.0 * T / self.duration_