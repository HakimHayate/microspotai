import numpy as np

class GaitController():
    def __init__(self, solverIk, links,
                 B=0.1, H=0.04, hz=50, duration=1):
        self.links_ = links
        self.ik_solver_ = solverIk
        self.current_positions = [0.0] * 12
        self.hz_ = hz
        self.B_ = B
        self.H_ = H
        self.swing_time_ = 1.0

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
    def get_foot_coordinate(self, t, thigh_foot, X_stride, Y_stride, sign_y = 1, defaultZ=-0.15):
        if thigh_foot is None:
            thigh_foot = [0, 0, defaultZ]

        t = t % 2.0
        swing_time = getattr(self, 'swing_time_', 1.0)
        Y_stride *= sign_y
        if t <= swing_time: # SWING PHASE
            norm_t = t / swing_time

            z = self.coeff_height_[0] * norm_t**6 + self.coeff_height_[1] * norm_t**5 + \
                self.coeff_height_[2] * norm_t**4 + self.coeff_height_[3] * norm_t**3 + thigh_foot[2]

            swing_progress = (1 - np.cos(np.pi * norm_t)) / 2.0

            x = -X_stride / 2 + X_stride * swing_progress + thigh_foot[0]
            y = -Y_stride / 2 + Y_stride * swing_progress + thigh_foot[1]

        else: # STANCE PHASE
            z = thigh_foot[2]

            stance_duration = 2.0 - swing_time
            stance_progress = (t - swing_time) / stance_duration

            x = X_stride / 2 - X_stride * stance_progress + thigh_foot[0]
            y = Y_stride / 2 - Y_stride * stance_progress + thigh_foot[1]

        return x, y, z

    def update_parameters(self, B, H, duration, swing_time, turn_rate=0.0):
        self.B_ = B
        self.H_ = H
        self.duration_ = duration
        self.swing_time_ = swing_time
        self.turn_rate_ = turn_rate 

        if self.H_ != H:
                    self.H_ = H
                    self.coeff_height_ = np.linalg.inv(np.array([ 
                        [1, 1, 1, 1],
                        [0.5**6, 0.5**5, 0.5**4, 0.5**3],
                        [6, 5, 4, 3],
                        [30, 20, 12, 6]
                    ])) @ np.array([0, self.H_, 0, 0])
                    
        T = 1 / self.hz_
        
        self.dt_ = 2.0 * T / self.duration_


    def trot_gait(self, thigh_foot):
        if thigh_foot is None:
            print('Body pose not initialized')
            return None

        self.t_ = (self.t_ + self.dt_) % 2.0
        command = {}

        front_Y = self.turn_rate_
        back_Y = -self.turn_rate_

        command['front_right'] = self.get_foot_coordinate(self.t_, thigh_foot['front_right'], self.B_, front_Y)
        command['back_left'] = self.get_foot_coordinate(self.t_, thigh_foot['back_left'], self.B_, back_Y, -1)

        command['front_left'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot['front_left'], self.B_, front_Y, -1)
        command['back_right'] = self.get_foot_coordinate(self.t_ + 1, thigh_foot['back_right'], self.B_, back_Y)

        return command