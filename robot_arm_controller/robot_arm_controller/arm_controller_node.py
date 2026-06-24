import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

from robot_arm_controller.robot_arm_controller.gui_arm import GUI
import tkinter as tk
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import os
from utils import get_safe_path, remove_manual_collision_pair, rpy
from scipy.linalg import logm
import xml.etree.ElementTree as ET


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        package_name = 'robot_arm_description'
        urdf_file_name = 'robot_arm.urdf'

        urdf_path = os.path.join(
            get_package_share_directory(package_name),
            'urdf',
            urdf_file_name
        )
        

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.joint_names_ = ['joint_4', 'joint_5', 'joint_3', 'joint_2', 'joint_1', 'joint_end_effector']
        self.current_angles_ = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.x_desired_ = None

        self.home_pose = np.zeros(6)
        self.path_home = None
        self.go_home = False
        self.idx_home = 0

        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)


        self.timer_ = self.create_timer(0.01, self.control_loop)

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.collision_model = pin.buildGeomFromUrdf(self.model, urdf_path, pin.GeometryType.COLLISION)

        self.collision_model.addAllCollisionPairs()

        remove_manual_collision_pair(self.collision_model, "base", "link_1")
        for i in range(1, 15):
            remove_manual_collision_pair(self.collision_model, f"link_{i}", f"link_{i+1}")

        self.collision_data = pin.GeometryData(self.collision_model)

        self.alpha = 0.01
        self.tol = 0.005

        self.link_1_id = self.model.getFrameId('link_1') # Joint_4
        self.link_3_id = self.model.getFrameId('link_3') # Joint_5
        self.link_7_id = self.model.getFrameId('link_7') # Joint_3
        self.link_11_id = self.model.getFrameId('link_11') # Joint_2
        self.link_13_id = self.model.getFrameId('link_13') # Joint_1
        self.link_15_id = self.model.getFrameId('link_15') # joint_end_effector

        self.end_effector = np.array([0.0, 0.0, 0.0]) # End effector in front of last joint

        self.new_path_ = False
        self.path_idx_ = 0
        self.path_ = [self.current_angles_]
        self.max_iter_ = 10000

        self.data_plan = self.model.createData()
        self.collision_data_plan = pin.GeometryData(self.collision_model)
    
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'target_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.x_desired_ = list(msg.data[:6])
        self.end_effector = np.array(msg.data[6:])
    
    def control_loop(self):
        if self.x_desired_ is None: # Nothing to do
            print('not started')
            return

        # Reset pose 
        if self.go_home and self.path_home is not None:
            if self.idx_home < len(self.path_home)-1:
                self.current_angles_ = self.path_home[self.idx_home]
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names_
                msg.position = self.current_angles_.tolist()
                self.joint_state_pub_.publish(msg)
                
                self.idx_home += 1
                return 
            else:
                self.go_home = False
                self.idx_home = 0

        # STATE 1: EXECUTING A PATH
        if self.new_path_:
            if self.path_idx_ < len(self.path_):
                self.current_angles_ = self.path_[self.path_idx_]
                
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names_
                msg.position = self.current_angles_.tolist()
                self.joint_state_pub_.publish(msg)
                
                self.path_idx_ += 1
                return 
            else:
                print('Path completed! Robot is resting.')
                self.new_path_ = False
                return 

        
        # STATE 2: RESTING 
        pin.forwardKinematics(self.model, self.data, self.current_angles_)
        pin.updateFramePlacements(self.model, self.data)

        current_position = np.zeros(3)
        T_world_link_15 = (self.data.oMf[self.link_15_id]).homogeneous

        end_effector_h = np.ones(4)
        end_effector_h[:3] = self.end_effector # In link 15 frame
        current_position = (T_world_link_15 @ end_effector_h)[:-1]
        

        target_position = np.zeros(3)
        target_position = self.x_desired_[:3]

        position_error = target_position - current_position

        R_current = T_world_link_15[:3, :3]
        R_desired_= rpy(self.x_desired_[3], self.x_desired_[4], self.x_desired_[5])

        orientation_error = pin.log3(R_desired_ @ R_current.T)

        error = np.hstack((position_error, orientation_error))
        if np.linalg.norm(error) < self.tol: # Check for new target position
            return 

        
        # STATE 3: PLANNING         
        q_target = self.current_angles_.copy()
        ik_success = False

        for _ in range(self.max_iter_):
            pin.forwardKinematics(self.model, self.data, q_target)
            pin.updateFramePlacements(self.model, self.data)

            T_world_link_15 = (self.data.oMf[self.link_15_id]).homogeneous
            end_effector_h = np.ones(4)
            end_effector_h[:3] = self.end_effector
            current_ik_pos = (T_world_link_15 @ end_effector_h)[:-1]

            J = pin.computeFrameJacobian(
                self.model, 
                self.data, 
                q_target, 
                self.link_15_id, 
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )

            I = np.eye(J.shape[0])
            J_T = J.T
            lambda_sq = 0.01
            J_damped_inv = J_T @ np.linalg.inv(J @ J_T + lambda_sq * I)
            e = np.zeros(6)
            e[:3] = self.x_desired_[:3] - current_ik_pos
            R_link15_world = T_world_link_15[:3, :3].T
            e[3:] = pin.log3(rpy(self.x_desired_[3], self.x_desired_[4], self.x_desired_[5]) @ R_link15_world)
            error = np.linalg.norm(e)
            
            if error < self.tol:
                ik_success = True
                break

            delta_q = self.alpha * J_damped_inv @ e
            q_target += delta_q
            print(f'error {error}')

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


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode() 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()