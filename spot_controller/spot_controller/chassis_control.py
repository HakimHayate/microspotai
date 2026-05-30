import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
from leg_ik_solver import LegIKSolver
from utils import get_twist, update_pos
from scipy.spatial.transform import Rotation as R
import math
import threading
from std_msgs.msg import Float64MultiArray
import tkinter as tk

class ChassisControl(Node):
    def __init__(self, target_x=0.1, target_y=0, target_z=0-0.1, roll_x=0, pitch_y=-0.2, yaw_z=0):
        super().__init__("ChassisControl")

        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/raw_position_bridge/commands', 
            10
        )

        self.joint_names = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        self.T_world_base_ = np.eye(4) # the global origin overlap with chassis initially

        l2 = 0.13  # Thigh link length
        l3 = 0.13  # Knee link 
        
        self.initialized_ = False # Inital feet coordinates

        self.T_world_foot_ = {}
        self.T_world_thigh_initial_= {}

        self.links_ = ['front_right', 'back_right', 'back_left', 'front_left']

        self.solver_ = LegIKSolver(0.06, l2, l3)

        self.duration_ = 1.0
        self.time_elapsed_ = 0.0
        self.dt_ = 0.01

        self.update_target(target_x, target_y, target_z, roll_x, pitch_y, yaw_z)

        self.timer_ = self.create_timer(self.dt_, self.control_loop)
        self.S_ = get_twist(self.T_world_base_, self.T_target_, t = self.duration_)  # Twist

    def update_target(self, x, y, z, roll, pitch, yaw):
        """
        Calculates the new target transformation matrix and updates the twist trajectory.
        Called by the GUI whenever a slider moves.
        """
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
        
        # Reset the timer so the robot smoothly interpolates to the new slider position
        self.time_elapsed_ = 0.0

    def control_loop(self):
        command = [0] * 12        

        if not self.initialized_:
            try:
                for link in self.links_:
                    tf_foot = self.tf_buffer_.lookup_transform(
                        target_frame=f'base_link', # Initially world frame = base frame
                        source_frame=f'{link}_feet',
                        time=rclpy.time.Time()
                    )

                    q = tf_foot.transform.rotation

                    R_bt = R.from_quat([
                        q.x,
                        q.y,
                        q.z,
                        q.w
                    ]).as_matrix()

                    t_bt = np.array([
                        tf_foot.transform.translation.x,
                        tf_foot.transform.translation.y,
                        tf_foot.transform.translation.z
                    ])

                    T_world_foot = np.eye(4)
                    T_world_foot[:3,:3] = R_bt
                    T_world_foot[:3,3] = t_bt

                    self.T_world_foot_[link] = T_world_foot

                    tf_thigh = self.tf_buffer_.lookup_transform(
                        target_frame='base_link',
                        source_frame=f'{link}_thigh',
                        time=rclpy.time.Time()
                    )

                    q = tf_thigh.transform.rotation

                    R_bt = R.from_quat([
                        q.x,
                        q.y,
                        q.z,
                        q.w
                    ]).as_matrix()

                    t_bt = np.array([
                        tf_thigh.transform.translation.x,
                        tf_thigh.transform.translation.y,
                        tf_thigh.transform.translation.z
                    ])

                    T_world_thigh = np.eye(4)
                    T_world_thigh[:3,:3] = R_bt
                    T_world_thigh[:3,3] = t_bt

                    self.T_world_thigh_initial_[link] = T_world_thigh


                self.initialized_ = True
                self.get_logger().info("Initialization OK!")

            except Exception as e:
                self.get_logger().warn(f"Waiting for TF: {e}")
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = [0] * 12

                self.joint_pub.publish(msg)
                return 
            
        if self.time_elapsed_>= self.duration_:
            return
        else:
            self.time_elapsed_ += self.dt_
        # Robot moves slighly each frame to make a smooth movement
        self.T_world_base_ = update_pos(self.T_world_base_, self.S_, dt=self.dt_)

        for i, link in enumerate(self.links_):
            try:
                T_world_thigh_current = self.T_world_base_ @ self.T_world_thigh_initial_[link]
                T_thigh_world = np.linalg.inv(T_world_thigh_current)
                T_thigh_foot = T_thigh_world @ self.T_world_foot_[link]
                thigh_foot = (T_thigh_foot[:-1, -1]).flatten()
                idx = i * 3

                command[idx], command[idx+1], command[idx+2] = self.solver_.solve(
                    thigh_foot[0], 
                    thigh_foot[1], 
                    thigh_foot[2]
                )

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'TF Error: {e}')

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = command

        self.joint_pub.publish(msg)

        command_msg = Float64MultiArray()
        command_msg.data = command
        self.get_logger().info(f"Publishing: {command_msg.data}")
        self.cmd_pub.publish(command_msg)

class RobotGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("Quadruped Chassis Controller")
        self.root.geometry("400x400")
        
        
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=-0.1)
        self.roll_var = tk.DoubleVar(value=0.0)
        self.pitch_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)

        self.create_sliders()
        
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_sliders(self):
        def add_slider(label, variable, from_, to, resolution):
            frame = tk.Frame(self.root)
            frame.pack(fill=tk.X, padx=10, pady=5)
            tk.Label(frame, text=label, width=10).pack(side=tk.LEFT)
            slider = tk.Scale(frame, variable=variable, from_=from_, to=to, 
                              resolution=resolution, orient=tk.HORIZONTAL, 
                              command=self.on_slider_change)
            slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        add_slider("X Target", self.x_var, -0.2, 0.2, 0.01)
        add_slider("Y Target", self.y_var, -0.2, 0.2, 0.01)
        add_slider("Z Target", self.z_var, -0.3, 0.1, 0.01)
        add_slider("Roll", self.roll_var, -0.5, 0.5, 0.01)
        add_slider("Pitch", self.pitch_var, -0.5, 0.5, 0.01)
        add_slider("Yaw", self.yaw_var, -1.0, 1.0, 0.01)

        # Reset button
        tk.Button(self.root, text="Reset to Zero", command=self.reset).pack(pady=20)

    def on_slider_change(self, event=None):
        # Push the latest slider values to the ROS node
        self.node.update_target(
            self.x_var.get(),
            self.y_var.get(),
            self.z_var.get(),
            self.roll_var.get(),
            self.pitch_var.get(),
            self.yaw_var.get()
        )

    def reset(self):
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        self.z_var.set(-0.1)
        self.roll_var.set(0.0)
        self.pitch_var.set(0.0)
        self.yaw_var.set(0.0)
        self.on_slider_change()

    def on_close(self):
        self.root.destroy()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisControl()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = RobotGUI(node)
    gui.root.mainloop()

if __name__ == '__main__':
    main()