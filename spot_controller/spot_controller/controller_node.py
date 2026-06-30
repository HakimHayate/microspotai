import rclpy
from rclpy.node import Node
import threading
import tkinter as tk
from tf2_ros import Buffer, TransformListener
from body_controller import BodyController
from gait_controller import GaitController
from sensor_msgs.msg import JointState, Imu
import numpy as np
from std_msgs.msg import Float64MultiArray
from leg_ik_solver import LegIKSolver
from stabilizer import Stabilizer
from scipy.spatial.transform import Rotation as R
from utils import quaternion_to_rpy
from gui_controller import AppGUI
from arm_controller import ArmController
from ament_index_python.packages import get_package_share_directory
import os
import pinocchio as pin



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

        self.isWalking = False
        self.isStanding = False
        self.isRotating = False
        self.isArmMoving = False

        self.mode_swap_ = False
        self.swap_initiated_ = False

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        pkg_share = get_package_share_directory('microspot_description')
        urdf_file = os.path.join(pkg_share, 'urdf', 'micro_v2.urdf')

        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()

        self.model_ = pin.buildModelFromXML(robot_desc)
        self.data_ = self.model_.createData()

        self.body_controller_ = BodyController(self.model_, self.data_, self.links_)
        self.gait_controller_ = GaitController()
        self.stabilizer_ = None
        self.timer_ = self.create_timer(0.01, self.control_loop)

        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.gazebo_pub_ = self.create_publisher(Float64MultiArray, '/raw_position_bridge/commands', 10)
        self.imu_sub_ = self.create_subscription(Imu,
                                              '/imu/data',
                                              self.imu_callback,
                                              10)
        
        self.T_world_base_ = np.eye(4)
        self.thigh_foot_ = None # In world frame

        self.imu_last_data_ = None

        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'target_pose',
            self.arm_pose_callback,
            10
        )

        self.current_angles_ = [0]*12
        #self.arm_controller_ = ArmController()

        self.dir_motor_ = {'front_right' : 1, 'back_right' : 1,
                           'front_left' : -1, 'back_left' : -1}
    
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
                                    thigh_foot[link][2], d
                                )
            q = [qi * d for qi in q]
            command[idx:idx+3] = q
        return command
    
    def pid(self, thigh_foot=None):
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
            thigh_foot_corrected[link] = thigh_foot[link] - error[link]
        
        return thigh_foot_corrected
    

    def control_loop(self):
        thigh_foot_correct = None
        
        if self.isWalking:
            thigh_foot = self.gait_controller_.trot_gait(self.thigh_foot_, self.links_)
            thigh_foot_correct = thigh_foot #self.pid(thigh_foot)

        elif self.isStanding:
            self.thigh_foot_, self.T_world_base_= self.body_controller_.body_pose()
            thigh_foot_correct = self.thigh_foot_ # self.pid()

        elif self.isRotating:
            pass # To do

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

        else:
            pass # To do
            
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
            

    def off_mode(self):
        self.isWalking = False
        self.isStanding = False
        self.isRotating = False

        self.gait_controller_.restart()
        self.body_controller_.restart()

    def trot_gait_mode(self):
        self.get_logger().info('Executing walking...')
        self.off_mode()
        self.isWalking = True
    
    def standing_mode(self):
        self.get_logger().info('Executing standing...')
        self.off_mode()
        self.isStanding = True

    def rotating_mode(self):
        self.get_logger().info('Executing rotating...')
        self.off_mode()
        self.isRotating = True
    
    def arm_mode(self):
        self.get_logger().info('Arm controlling...')
        self.off_mode()
        self.isArmMoving = True


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