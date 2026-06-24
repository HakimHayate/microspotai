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
from utils import get_safe_path, remove_manual_collision_pair, rpy
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
        
        self.data = self.model.createData()
        self.collision_data = pin.GeometryData(self.collision_model)

        self.data = self.model.createData()

        self.collision_model.addAllCollisionPairs()

        remove_manual_collision_pair(self.collision_model, "base", "link_1")
        for i in range(1, 15):
            remove_manual_collision_pair(self.collision_model, f"link_{i}", f"link_{i+1}")

        self.collision_data = pin.GeometryData(self.collision_model)

        self.alpha = 0.05
        self.tol = 0.05

        self.gripper_id_ = self.model.getFrameId('rotor_6')

        self.end_effector = np.array([0.0, 0.0, 0.0]) # End effector in front of last joint

        self.new_path_ = False
        self.path_idx_ = 0
        self.path_ = [self.current_angles_]
        self.max_iter_ = 10000

        self.data_plan = self.model.createData()
        self.collision_data_plan = pin.GeometryData(self.collision_model)
    

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
        if np.linalg.norm(position_error) < self.tol: # Check for new target position
            return None

        
        # STATE 3: PLANNING         
        q_target = self.current_angles_.copy()
        ik_success = False

        for _ in range(self.max_iter_):
            pin.forwardKinematics(self.model, self.data, q_target)
            pin.updateFramePlacements(self.model, self.data)

            T_world_gripper = (self.data.oMf[self.gripper_id_]).homogeneous
            end_effector_h = np.ones(4)
            end_effector_h[:3] = self.end_effector
            current_ik_pos = (T_world_gripper)[:3,-1]
            
            J = pin.computeFrameJacobian(
                self.model, 
                self.data, 
                q_target, 
                self.gripper_id_, 
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )

            J = J[:3]
            I = np.eye(J.shape[0])
            J_T = J.T
            lambda_sq = 0.01
            J_damped_inv = J_T @ np.linalg.inv(J @ J_T + lambda_sq * I)
            e = np.zeros(3)
            e = self.x_desired_[:3] - current_ik_pos
            R_world_gripper = T_world_gripper[:3, :3]
            #e[3:] = pin.log3(rpy(self.x_desired_[3], self.x_desired_[4], self.x_desired_[5]) @ R_world_gripper.T)
            error = np.linalg.norm(e)
            print(error)
            if error < self.tol:
                ik_success = True
                break

            delta_q = self.alpha * J_damped_inv @ e
            q_target += delta_q

        if ik_success:
            print("IK Solved")
            self.path_ = get_safe_path(
                self.current_angles_, 
                q_target, 
                self.model, 
                self.collision_model, 
                self.data_plan, 
                self.collision_data_plan 
            )
            
            if self.path_ is not None:
                print(f"Found safe path with {len(self.path_)} waypoints.")
                self.new_path_ = True
                self.path_idx_ = 0
            else:
                print("Path blocked")
                self.go_home = True
                self.path_home = get_safe_path(self.current_angles_, self.home_pose,
                                                self.model, 
                                                self.collision_model, 
                                                self.data_plan, 
                                                self.collision_data_plan)
        else:
            print("IK Failed")
            self.go_home = True
            self.path_home = get_safe_path(self.current_angles_, self.home_pose,
                                                self.model, 
                                                self.collision_model, 
                                                self.data_plan, 
                                                self.collision_data_plan)

