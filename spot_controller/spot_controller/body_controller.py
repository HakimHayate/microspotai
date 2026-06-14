import numpy as np
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState

from spot_controller.utils import get_twist, update_pos, get_rotation

import math
from std_msgs.msg import Float64MultiArray
import rclpy

class BodyController():
    def __init__(self, controller_node, joints_names, links, defaultZ, duration = 1.0, 
                 dt= 0.02):
        
        self.controller_node_ = controller_node
        self.joints_names_ = joints_names

        self.links_ = links

        self.duration_ = duration
        self.time_elapsed_ = 0.0
        self.dt_ = dt

        self.reached_target_ = True
        self.initialized_ = False # Initiate feet coordinates
        
        self.T_target_ = np.eye(4)

        self.S_ = None

    def getT(self):
        return self.T_world_base_, self.thigh_foot_
    
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

        self.T_target_ = np.array([
            [Rxyz[0,0], Rxyz[0,1], Rxyz[0,2], x],
            [Rxyz[1,0], Rxyz[1,1], Rxyz[1,2], y],
            [Rxyz[2,0], Rxyz[2,1], Rxyz[2,2], z],
            [0, 0, 0, 1]
        ])

        # Generate a new twist path from current position to new target
        self.S_ = get_twist(T_world_base, self.T_target_, t=self.duration_)
        
        self.time_elapsed_ = 0.0

        self.reached_target_ = False   

        
    def body_pose(self, T_world_base, T_base_thigh, T_world_foot, links):
        if self.time_elapsed_>= self.duration_: # Robot reached target pose
            self.reached_target_ = True

        elif not self.reached_target_:
            self.time_elapsed_ += self.dt_
            # Robot moves slighly each frame to make a smooth movement
            T_world_base = update_pos(T_world_base, self.S_, dt=self.dt_)

        thigh_foot = {}

        for i, link in enumerate(links):
            try:
                T_world_thigh_current = T_world_base @ T_base_thigh[link] # Move thighs coordinates to new coordinates
                T_thigh_world = np.linalg.inv(T_world_thigh_current)

                T_thigh_foot = T_thigh_world @ T_world_foot[link] # Foot coordinates in thigh frame
                
                thigh_foot[link] = T_thigh_foot[:-1, -1]

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.controller_node_.get_logger().warn(f'TF Error: {e}')

        return thigh_foot, T_world_base
