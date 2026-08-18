import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

from gui_arm import GUI
import tkinter as tk
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import os
from utils import *
from scipy.linalg import logm
import xml.etree.ElementTree as ET
from launch.substitutions import Command
import xacro

class ArmController():
    def __init__(self):
        pkg_share = get_package_share_directory("microspot_description")
        xacro_file = os.path.join(pkg_share, 'urdf', 'spot_arm.urdf.xacro')
        
        self.arm_joint_names_ = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6"
        ]

        
        self.body_joint_names_pin_ = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]
        
        self.current_angles_ = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.body_angles_ = None
        self.x_desired_ = None

        self.home_pose = np.zeros(6)
        self.path_home = None
        self.go_home = False
        self.idx_home = 0

        robot_xml = xacro.process_file(xacro_file).toxml()
        full_model = pin.buildModelFromXML(robot_xml)
        full_collision_model = pin.buildGeomFromUrdfString(full_model, robot_xml, pin.GeometryType.COLLISION)

        joints_to_lock = []
        for joint_name in self.body_joint_names_pin_:
            joints_to_lock.append(full_model.getJointId(joint_name))
        
        q_reference = pin.neutral(full_model)
        self.model, [self.collision_model] = pin.buildReducedModel(
                                                full_model, 
                                                [full_collision_model], 
                                                joints_to_lock, 
                                                q_reference
                                            )
        self.qmin = np.array(self.model.lowerPositionLimit[0:7])
        self.qmax = np.array(self.model.upperPositionLimit[0:7])

        self.data = self.model.createData()
        self.collision_data = pin.GeometryData(self.collision_model)

        self.data = self.model.createData()

        self.collision_model.addAllCollisionPairs()

        remove_manual_collision_pair(self.collision_model, "base", "link_1")
        for i in range(1, 15):
            remove_manual_collision_pair(self.collision_model, f"link_{i}", f"link_{i+1}")

        self.collision_data = pin.GeometryData(self.collision_model)

        self.alpha = 0.05
        self.tol = 0.01

        self.gripper_id_ = self.model.getFrameId('rotor_6')

        self.end_effector = np.array([0.0, 0.0, 0.0]) # End effector in front of last joint

        self.new_path_ = False
        self.path_idx_ = 0
        self.path_ = [self.current_angles_]
        self.max_iter_ = 10000

        self.data_plan = self.model.createData()
        self.collision_data_plan = pin.GeometryData(self.collision_model)
        self.rng = np.random.default_rng()
    

    def control_loop(self):
        if self.x_desired_ is None: # Nothing to do
            print('not started')
            return None

        # Reset pose 
        if self.go_home and self.path_home is not None:
            if self.idx_home < len(self.path_home):
                self.current_angles_ = self.path_home[self.idx_home]
                
                self.idx_home += 1
                return {'command' : self.current_angles_, 'joint_names' : self.arm_joint_names_}
            else:
                self.go_home = False
                self.idx_home = 0

        # STATE 1: EXECUTING A PATH
        if self.new_path_:
            if self.path_idx_ < len(self.path_):
                self.current_angles_ = self.path_[self.path_idx_]
                self.path_idx_ += 1
                return {'command' : self.current_angles_, 'joint_names' : self.arm_joint_names_}
            
            else:
                print('Path completed! Robot is resting.')
                self.new_path_ = False
                return None

        
        # STATE 2: RESTING 
        pin.forwardKinematics(self.model, self.data, self.current_angles_)
        pin.updateFramePlacements(self.model, self.data)

        current_position = np.zeros(3)
        T_world_gripper = (self.data.oMf[self.gripper_id_]).homogeneous

        end_effector_h = np.ones(4)
        end_effector_h[:3] = self.end_effector 
        current_position = (T_world_gripper)[:3,-1]
        

        target_position = np.zeros(3)
        target_position = self.x_desired_[:3]

        position_error = target_position - current_position

        R_current = T_world_gripper[:3, :3]
        R_desired_= rpy(self.x_desired_[3], self.x_desired_[4], self.x_desired_[5])

        orientation_error = pin.log3(R_desired_ @ R_current.T)

        error = np.hstack((position_error, orientation_error))
        if np.linalg.norm(error) < self.tol: # Check for new target position
            return None
        
        # STATE 3: PLANNING
        q_inter = self.current_angles_.copy()

        q_inter[1] = 2.1 + self.rng.uniform(-1, 1) # Keep the elbow up
        q_inter[2] = 3.14 + self.rng.uniform(-1, 1)

        q_inter[3] = self.rng.uniform(self.qmin[3], self.qmax[3])
        q_inter[4] = self.rng.uniform(self.qmin[4], self.qmax[4])
        q_inter[5] = self.rng.uniform(self.qmin[5], self.qmax[5])
        q_inter[0] = self.rng.uniform(self.qmin[0], self.qmax[0])
        
        inter_path = get_safe_path(
                    self.current_angles_, 
                    q_inter,
                    self.model, 
                    self.collision_model,
                    self.data_plan, 
                    self.collision_data_plan, 
                    self.qmin,
                    self.qmax
                    )
        if inter_path is None:
            return 
        
        q_target = computeJacobian(self, q_inter, self.qmin, self.qmax, self.tol)

        if q_target is None:
            print('IK failed')
            return
        
        end_path =  get_safe_path( 
                    q_inter,
                    q_target, 
                    self.model, 
                    self.collision_model,
                    self.data_plan, 
                    self.collision_data_plan, 
                    self.qmin,
                    self.qmax
                    )

        if end_path is not None:
            self.path_ = inter_path + end_path
            print(f"Found safe path with {len(self.path_)} waypoints.")
            self.new_path_ = True
            self.path_idx_ = 0
            return
        else:
            print('path blocked')
                    
                
        

        

