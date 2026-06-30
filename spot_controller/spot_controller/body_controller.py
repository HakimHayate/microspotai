import numpy as np
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState

from utils import get_twist, update_pos, get_rotation
import time
import math
from std_msgs.msg import Float64MultiArray
import rclpy
import pinocchio as pin

class BodyController():
    "Idea : Virtually move the base link and keep the foot planted"
    def __init__(self, model, data, links, 
                 duration = .020, dt= 0.02):
        
        self.model_ = model
        self.data_ = data
        self.links_ = links
        self.duration_ = duration
        self.time_elapsed_ = 0.0
        self.dt_ = dt

        self.reached_target_ = True

        self.T_target_ = np.eye(4)
        self.S_ = None

        self.q = np.array([0]*12)

        pin.forwardKinematics(model, data, self.q)
        pin.updateFramePlacements(model, data)

        base_id = model.getFrameId('world_link')
        T_world_base = data.oMf[base_id].homogeneous
        self.T_world_base_initially_ = T_world_base # Initial pose of robot
        self.T_world_base_ = T_world_base
        
        self.T_base_thigh_ = {}
        self.T_world_foot_ = {}
        for link in links:
            thigh_id = model.getFrameId(f'{link}_end_foot_dummy')
            T_world_thigh = data.oMf[thigh_id].homogeneous

            self.T_base_thigh_[link]= np.linalg.inv(T_world_base) @ T_world_thigh

            foot_id = model.getFrameId(f'{link}_end_foot')
            self.T_world_foot_[link] = data.oMf[foot_id].homogeneous
            print(f'T_world_foot_{link} {self.T_world_foot_[link]}')

        self.T_world_base_ = T_world_base
            

    
    def restart(self):
        self.time_elapsed_ = 0

    def update_target(self, T_world_base, desired_pose: dict) -> None:
        """
        Calculates the new target transformation matrix and updates the twist trajectory.

        Params:
            desired_pose : dict containing these keys: x, y, z, roll, pitch, yaw in radians
        """

        x = desired_pose.get('x', 0)
        y = desired_pose.get('y', 0)
        z = desired_pose.get('z', 0)

        roll = desired_pose.get('roll', 0)
        pitch = desired_pose.get('pitch', 0)
        yaw = desired_pose.get('yaw', 0)

        Rxyz = get_rotation(roll, pitch, yaw) # Convention: Roll-Pitch-Yaw

        self.T_target_ = self.T_world_base_initially_ @ np.array([
            [Rxyz[0,0], Rxyz[0,1], Rxyz[0,2], x],
            [Rxyz[1,0], Rxyz[1,1], Rxyz[1,2], y],
            [Rxyz[2,0], Rxyz[2,1], Rxyz[2,2], z],
            [0, 0, 0, 1]
        ])

        # Generate a new twist path from current position to new target
        self.S_ = get_twist(T_world_base, self.T_target_, t=self.duration_)
        
        self.time_elapsed_ = 0.0

        self.reached_target_ = False   

    

    def body_pose(self):
        if self.time_elapsed_>= self.duration_: # Robot reached target pose
            self.reached_target_ = True

        elif not self.reached_target_:
            self.time_elapsed_ += self.dt_
            # Robot moves slighly each frame to make a smooth movement
            self.T_world_base_ = update_pos(self.T_world_base_, self.S_, dt=self.dt_)
            
        thigh_foot = {}

        for link in self.links_: 
            T_world_thigh = self.T_world_base_ @ self.T_base_thigh_[link] # Move thighs coordinates to new coordinates
            R = self.T_base_thigh_[link][:3,:3]
            thigh_foot[link] = R.T @ (self.T_world_foot_[link][:-1,-1] - T_world_thigh[:-1,-1])
            print(f'thigh_foot_{link} {thigh_foot[link]}')
        
        print('\n\n')
        return thigh_foot, self.T_world_base_
