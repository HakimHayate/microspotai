import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
from leg_ik_solver import LegIKSolver
from utils import get_twist, update_pos
from scipy.spatial.transform import Rotation as R
import math
from std_msgs.msg import Float64MultiArray
import rclpy

class BodyController():
    def __init__(self, controller_node, joints_names, defaultZ, duration = 1.0, 
                 dt= 0.02, len_hip= 0.06, len_thigh= 0.13, len_knee= 0.13):
        
        self.controller_node_ = controller_node
        self.joints_names_ = joints_names

        self.links_ = ['front_right', 'back_right', 'back_left', 'front_left']

        self.solver_ = LegIKSolver(len_hip, len_thigh, len_knee)

        self.duration_ = duration
        self.time_elapsed_ = 0.0
        self.dt_ = dt

        self.reached_target_ = False
        self.initialized_ = False # Initiate feet coordinates

        self.T_world_base_ = np.eye(4)
        self.T_world_base_[2, -1] = defaultZ

        self.T_world_foot_ = {}
        self.T_world_thigh_initial_= {}
        self.T_target_ = np.eye(4)

        self.S_ = np.zeros((3,3))

    def getT(self):
        return self.T_world_base_
    
    def restart(self):
        self.time_elapsed_ = 0

    def reset_pose(self, T_target, T_world_base=None):
        '''
        Bring the robot to the default pose
        '''
        self.T_world_base_ = T_world_base if T_world_base is not None else self.T_world_base_
        self.T_target_ = T_target
        
        self.S_ = get_twist(self.T_world_base_, self.T_target_, t=self.duration_)
        
        self.time_elapsed_ = 0.0

        self.reached_target_ = False 

    def update_target(self, desired_pose: dict) -> None:
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
        
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ])

        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        Rxyz = Rz @ Ry @ Rx # Convention: Roll-Pitch-Yaw

        self.T_target_ = np.array([
            [Rxyz[0,0], Rxyz[0,1], Rxyz[0,2], x],
            [Rxyz[1,0], Rxyz[1,1], Rxyz[1,2], y],
            [Rxyz[2,0], Rxyz[2,1], Rxyz[2,2], z],
            [0, 0, 0, 1]
        ])

        # Generate a new twist path from current position to new target
        self.S_ = get_twist(self.T_world_base_, self.T_target_, t=self.duration_)
        
        self.time_elapsed_ = 0.0

        self.reached_target_ = False   

        
    def transform(self, frame_source, frame_dst):
        tf = self.controller_node_.tf_buffer_.lookup_transform(
                        target_frame=frame_source, 
                        source_frame=frame_dst,
                        time=rclpy.time.Time()
                    )

        q = tf.transform.rotation

        Rot = R.from_quat([
            q.x,
            q.y,
            q.z,
            q.w
        ]).as_matrix()

        t = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ])

        T = np.eye(4)
        T[:3,:3] = Rot
        T[:3,3] = t

        return T
        
    def body_pose(self):
        command_msg = JointState()       

        if not self.initialized_:
            try:
                for link in self.links_:
                    self.T_world_foot_[link] = self.transform(frame_source='base_link', frame_dst=f'{link}_feet')
                    self.T_world_thigh_initial_[link] = self.transform(frame_source='base_link', frame_dst=f'{link}_thigh')

                self.initialized_ = True
                self.controller_node_.get_logger().info("Standing: Initialization OK!")

            except Exception as e:
                self.controller_node_.get_logger().warn(f"Waiting for TF: {e}")

                command_msg.header.stamp = self.controller_node_.get_clock().now().to_msg()
                command_msg.name = self.joints_names_
                command_msg.position = [0] * 12

                return command_msg
            
        if self.time_elapsed_>= self.duration_: # Robot reached target pose
            self.reached_target_ = True
        else:
            self.time_elapsed_ += self.dt_

            # Robot moves slighly each frame to make a smooth movement
            self.T_world_base_ = update_pos(self.T_world_base_, self.S_, dt=self.dt_)

        command = [0] * 12

        for i, link in enumerate(self.links_):
            try:
                T_world_thigh_current = self.T_world_base_ @ self.T_world_thigh_initial_[link] # Move thighs coordiantes to new coordinates
                
                T_thigh_world = np.linalg.inv(T_world_thigh_current)

                T_thigh_foot = T_thigh_world @ self.T_world_foot_[link] # Foot coordinates in thigh frame
                
                thigh_foot = (T_thigh_foot[:-1, -1]).flatten()
                
                idx = i * 3
                command[idx], command[idx+1], command[idx+2] = self.solver_.solve(
                                                                    thigh_foot[0], 
                                                                    thigh_foot[1], 
                                                                    thigh_foot[2]
                                                                )

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.controller_node_.get_logger().warn(f'TF Error: {e}')


        command_msg.header.stamp = self.controller_node_.get_clock().now().to_msg()
        command_msg.name = self.joints_names_
        command_msg.position = command

        return command_msg
