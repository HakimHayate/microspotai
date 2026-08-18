import os
import json
import threading
import tkinter as tk
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

from spotmicro_controller.body_controller import BodyController
from spotmicro_controller.gait_controller import GaitController
from spotmicro_controller.leg_ik_solver import LegIKSolver
from spotmicro_controller.stabilizer import Stabilizer
from spotmicro_controller.utils import quaternion_to_rpy
from spotmicro_controller.gui_controller import AppGUI
from spotmicro_controller.rotate_controller import RotateController

from spotmicro_controller.arm_controller import ArmController

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from ament_index_python.packages import get_package_share_directory

class ControllerNode(Node):
    """The ROS 2 Node logic"""
    def __init__(self, len_hip= 0.06, len_thigh= 0.13, len_knee= 0.13):
        super().__init__('controller_node')

        self.solver_ = LegIKSolver(len_hip, len_thigh, len_knee)

        self.joint_names_ = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]

        self.links_ = ['front_right', 'back_right', 'back_left', 'front_left']
        self.initialized_ = False
        self.defaultZ_ = -0.1

        self.defaultPose_ = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.defaultZ_],
            [0, 0, 0, 1]
        ])

        self.B_ = 0.06
        self.H_ = 0.03
        self.swing_time_ = 1.0

        self.isSitting = True
        self.isWalking = False
        self.isStanding = False
        self.isRotating = False
        self.isArmMoving = False
        self.auto_mode_ = False
        self.mode_swap_ = False
        self.swap_initiated_ = False

        self.sit_pose_1 = {'x':-0.01, 'y':0.0, 'z':-0.213}
        self.sit_pose_2 = {'x':-0.01, 'y':-0.025, 'z':-0.213}

        self.standup_pose_1 = {'x':-0.01, 'y':0.0, 'z':-0.213}
        self.standup_pose_2 = {'x':0.03, 'y':0.0, 'z':-0.10}

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        pkg_share = get_package_share_directory('spotmicro_description')
        urdf_file = os.path.join(pkg_share, 'urdf', 'micro_v2.urdf')

        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()

        self.model_ = pin.buildModelFromXML(robot_desc)
        self.data_ = self.model_.createData()

        self.stand_pose_saved = False

        self.T_base_thigh_ = None
        self.thigh_foot_ = None
        

        self.body_controller_ = BodyController(self, self.model_, self.data_, self.links_)
        self.T_world_base_ = self.body_controller_.set_initial_pose(self.sit_pose_2)

        self.rotate_controller_ = RotateController(self.solver_, self.links_, self.T_base_thigh_, self.T_world_base_)

        self.stabilizer_ = Stabilizer(self.T_base_thigh_)
        self.timer_ = self.create_timer(0.02, self.control_loop)

        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 1)
        self.gazebo_pub_ = self.create_publisher(Float64MultiArray, '/raw_position_bridge/commands', 10)
        self.imu_sub_ = self.create_subscription(Imu,
                                              '/imu/data',
                                              self.imu_callback,
                                              10)
        

        self.imu_last_data_ = None

        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'target_pose',
            self.arm_pose_callback,
            10
        )

        

        self.sit_pose_1_in_progress  = False
        self.sit_pose_2_in_progress  = False
        self.isSitting_mode = True

        self.standup_pose_1_in_progress  = False
        self.standup_pose_2_in_progress  = False
        self.isStandingUp_mode = False
        
        self.current_angles_ = [0]*12
        #self.arm_controller_ = ArmController()

        self.dir_motor_ = {'front_right' : 1, 'back_right' : 1,
                           'front_left' : 1, 'back_left' : 1}
        
        self.cmd_pub_ = self.create_publisher(String, '/robot_commands', 1)
        self.pc_thigh_foot_pub_ = self.create_publisher(Float64MultiArray, '/pc_thigh_foot', 1)
        self.mode_publisher_ = self.create_publisher(String, '/robot_mode', 10)

    def arm_pose_callback(self, msg):
        self.arm_controller_.x_desired_ = list(msg.data[:6])
        self.arm_controller_.end_effector = np.array(msg.data[6:])


    def imu_callback(self, msg):
        self.imu_last_data_ = msg


    def get_command(self, thigh_foot):
        command = [0] * 12
        for i, link in enumerate(self.links_):
            d = self.dir_motor_[link]
            idx = i * 3
            q = self.solver_.solve(thigh_foot[link][0], 
                                    thigh_foot[link][1], 
                                    thigh_foot[link][2]
                                )
            q = [qi * d for qi in q]
            command[idx:idx+3] = q
        return command
    
    def stabilize(self, thigh_foot=None, alpha=0.4):
        thigh_foot = thigh_foot if thigh_foot is not None else self.thigh_foot_

        thigh_foot_corrected = {}
        if self.imu_last_data_ is None:
            return  None
        
        roll_sensor, pitch_sensor, _ = quaternion_to_rpy(self.imu_last_data_.orientation.x,
                                                         self.imu_last_data_.orientation.y,
                                                         self.imu_last_data_.orientation.z,
                                                         self.imu_last_data_.orientation.w)
        
        error = self.stabilizer_.compute_error(roll_sensor, pitch_sensor, self.T_world_base_, self.thigh_foot_, self.links_)

        for link in self.links_:
            thigh_foot_corrected[link] = thigh_foot[link] - alpha * error[link]
        
        return thigh_foot_corrected
    
    def sit(self):
        if self.isSitting:
            return None

        if not self.stand_pose_saved:
            self.stand_pose_saved = True
            self.last_T_stand = self.body_controller_.T_target_

        if not self.sit_pose_1_in_progress and not self.sit_pose_2_in_progress:
            self.sit_pose_1_in_progress  = True
            self.body_controller_.update_target(self.T_world_base_, self.sit_pose_1)

        if self.sit_pose_1_in_progress and self.body_controller_.reached_target_:
            self.sit_pose_1_in_progress  = False
            self.sit_pose_2_in_progress  = True
            self.body_controller_.update_target(self.T_world_base_, self.sit_pose_2)

        if self.sit_pose_2_in_progress and self.body_controller_.reached_target_:
            self.sit_pose_2_in_progress  = False
            self.isSitting  = True
            self.get_logger().info('Robot sat down')

        return self.body_controller_.body_pose()
    
    def standup(self):
        if not self.isSitting:
            return None
        
        if not self.standup_pose_1_in_progress and not self.standup_pose_2_in_progress:
            self.standup_pose_1_in_progress  = True
            self.body_controller_.update_target(self.T_world_base_, self.standup_pose_1)

        if self.standup_pose_1_in_progress and self.body_controller_.reached_target_:
            self.standup_pose_1_in_progress  = False
            self.standup_pose_2_in_progress  = True
            self.body_controller_.update_target(self.T_world_base_, self.standup_pose_2)

        if self.standup_pose_2_in_progress and self.body_controller_.reached_target_:
            self.standup_pose_2_in_progress  = False
            self.isStandingUp_mode  = False
            self.isSitting = False
            self.isStanding = True
            self.get_logger().info('Robot is standing!')

        return self.body_controller_.body_pose()

    def control_loop(self):
        thigh_foot_correct = None
        
        if self.isWalking:
            return

        if self.auto_mode_:
            return
        
        elif self.isStanding:
            self.thigh_foot_, self.T_world_base_= self.body_controller_.body_pose()
            thigh_foot_correct = self.stabilize()

        elif self.isRotating:
            thigh_foot = self.rotate_controller_.rotate(self.thigh_foot_, self.T_world_base_)
            if thigh_foot is None:
                self.isRotating = False
            thigh_foot_correct = thigh_foot

        elif self.isSitting_mode:
            res = self.sit()
            if res is None:
                return
            self.thigh_foot_, self.T_world_base_= res
            thigh_foot_correct = self.thigh_foot_ #self.stabilize()

        elif self.isStandingUp_mode:
            res = self.standup()
            if res is None:
                return
            self.thigh_foot_, self.T_world_base_= res
            thigh_foot_correct = self.thigh_foot_

        """
        elif self.isArmMoving:
            res = self.arm_controller_.control_loop()
            if res is None:
                return
            command, arm_joint_names = res['command'], res['joint_names']
            cmd_arm = JointState()
            cmd_arm.header.stamp = self.get_clock().now().to_msg()
            cmd_arm.name = arm_joint_names
            cmd_arm.position = command
            self.joint_state_pub_.publish(cmd_arm)
            return

        """
            
        if thigh_foot_correct is not None:
            command = self.get_command(thigh_foot=thigh_foot_correct)
            self.current_angles_ = command
            # Publish to Gazebo
            cmd_gazebo = Float64MultiArray()
            cmd_gazebo.data = command
            self.gazebo_pub_.publish(cmd_gazebo)

            # Publish to Joint State topic 
            cmd_robot = JointState()
            cmd_robot.header.stamp = self.get_clock().now().to_msg()
            cmd_robot.name = self.joint_names_
            cmd_robot.position = command
            self.joint_state_pub_.publish(cmd_robot)

            flat_coords = []
            for link in self.links_: # ['front_right', 'back_right', 'back_left', 'front_left']
                flat_coords.extend(thigh_foot_correct[link])
            
            # Publish coordinates to the Pi
            msg = Float64MultiArray()
            msg.data = [float(val) for val in flat_coords] 
            self.pc_thigh_foot_pub_.publish(msg)
            

    def send_command(self, mode_name):
        command_data = {
            "mode": mode_name,
            "B": getattr(self, 'B_', 0.06),
            "H": getattr(self, 'H_', 0.03),
            "duration": getattr(self, 'duration_', 2.0),
            "swing_time": getattr(self, 'swing_time_', 1.0),
            "turn_rate": getattr(self, 'turn_rate_', 0.0)
        }
        msg = String()
        msg.data = json.dumps(command_data)
        self.cmd_pub_.publish(msg)

    def off_mode(self):
        self.isWalking = False
        self.isStanding = False
        self.isRotating = False
        self.isArmMoving = False
        self.auto_mode_ = False
        self.isSitting_mode = False
        self.isStandingUp_mode = False

        msg = String()
        msg.data = "manuel"  
        self.mode_publisher_.publish(msg)

    def trot_gait_mode(self):
        if not self.isStanding:
            self.get_logger().info('Robot has to stand up!...')
            return
        self.get_logger().info('Executing walking...')
        self.off_mode()
        self.isWalking = True
        self.send_command("walk")  
    
    def standing_mode(self):
        if self.isSitting_mode:
            self.get_logger().info('Robot has to stand up...')
            return
        self.get_logger().info('Executing standing...')
        self.off_mode()
        self.isStanding = True
        self.send_command("pc_control")  

    def rotating_mode(self):
        self.get_logger().info('Executing rotating...')
        self.off_mode()
        self.isRotating = True
        self.send_command("pc_control")
    
    def arm_mode(self):
        self.get_logger().info('Arm controlling...')
        self.off_mode()
        self.isArmMoving = True
        self.send_command("pc_control")

    def sit_mode(self):
        if not self.isStanding:
            self.get_logger().info('Robot already Sitting down...')
            return
        self.get_logger().info('Sitting down...')
        self.off_mode()
        self.isSitting_mode = True

    def standUp_mode(self):
        if not self.isSitting:
            self.get_logger().info('Already standing up...')
            return 
        self.get_logger().info('Standing up...')
        self.off_mode()
        self.isStandingUp_mode = True
        
    def auto_mode(self):
        self.get_logger().info('Auto mode...')
        self.off_mode()
        self.auto_mode_ = True
        msg = String()
        msg.data = "auto"  
        self.mode_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.daemon = True
    spin_thread.start()
    
    root = tk.Tk()
    app = AppGUI(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)  
        
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()