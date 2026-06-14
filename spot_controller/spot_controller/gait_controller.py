from std_msgs.msg import Float64MultiArray
import math
from spot_controller.leg_ik_solver import LegIKSolver
from sensor_msgs.msg import JointState
import numpy as np

class GaitController():
    def __init__(self, controller_node, stride_length=0.06, 
                 stride_height=0.05, 
                 len_hip = 0.05, len_thigh= 0.13, 
                 len_knee = 0.13):

        self.controller_node_ = controller_node
        self.ik_solver_ = LegIKSolver(len_hip, len_thigh, len_knee)


        self.current_positions = [0.0] * 12
        self.time_elapsed_ = 0.0
        self.gait_speed = 4.0

        self.L = stride_length
        self.H = stride_height

    
    def get_foot_coordinate(self, thigh_foot, phase):
        """
        Takes a phase parameter from 0 to 2*pi and returns target (X, Z) coordinates in thigh's frame.
        """
        phi = phase % (2 * math.pi)
        
        if phi < math.pi:
            z = (self.L / 2.0) * math.cos(phi + math.pi) + thigh_foot[2]
            x =  - (self.H * math.sin(phi)) + self.thigh_foot_[0]
            
        else:
            stance_progress = (phi - math.pi) / math.pi 
            z = (self.L / 2.0) - (self.L * stance_progress) + thigh_foot[2]
            x = self.thigh_foot_[0]
            
        return x, 0, z
    

    def restart(self):
        self.time_elapsed_ = 0


    def trot_gait(self, thigh_foot, links, reset=False):
        if reset:
            self.time_elapsed_ = 0

        self.time_elapsed_ += 0.02
        base_phase = self.time_elapsed_ * self.gait_speed

        fr = self.get_foot_coordinate(base_phase, thigh_foot)
        bl = self.get_foot_coordinate(base_phase, thigh_foot)
        
        fl = self.get_foot_coordinate(base_phase + math.pi, thigh_foot)
        br = self.get_foot_coordinate(base_phase + math.pi, thigh_foot)
        
        tmp = [fr, br, bl, fl] # Same order as in links
        command = {}

        for i, link in enumerate(links):
            command[link] = tmp[i]
        
        return command