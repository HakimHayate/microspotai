import rclpy
from rclpy.node import Node
import threading
import tkinter as tk
from tf2_ros import Buffer, TransformListener
from spot_controller.body_controller import BodyController
from spot_controller.gait_controller import GaitController
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float64MultiArray

class ControllerNode(Node):
    """The ROS 2 Node logic"""
    def __init__(self):
        super().__init__('controller_node')

        self.joint_names = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]

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

        self.mode_swap_ = False
        self.swap_initiated_ = False

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.body_controller_ = BodyController(self, self.joint_names, defaultZ= self.defaultZ_)
        self.gait_controller_ = GaitController(self, self.joint_names)

        self.timer_ = self.create_timer(0.01, self.control_loop)

        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 10)
        self.gazebo_pub_ = self.create_publisher(Float64MultiArray, '/raw_position_bridge/commands', 10)

        self.T_world_base_ = None
        self.thigh_foot_ = None

    def control_loop(self):
        if self.mode_swap_:
            if not self.swap_initiated_:    
                self.body_controller_.reset_pose(self.defaultPose_)
                self.swap_initiated_ = True

            if not self.body_controller_.reached_target_:
                cmd_robot, cmd_gazebo = self.body_controller_.body_pose()
                self.joint_state_pub_.publish(cmd_robot)
                if cmd_gazebo is not None:
                    self.gazebo_pub_.publish(cmd_gazebo)

            else:
                self.mode_swap_ = False
            return 

        if self.isWalking:
            cmd_robot, cmd_gazebo = self.gait_controller_.trot_gait(thigh_foot=self.thigh_foot_)
            self.joint_state_pub_.publish(cmd_robot)
            if cmd_gazebo is not None:
                self.gazebo_pub_.publish(cmd_gazebo)
            
            # self.T_world_base_ = self.gait_controller_.getT()
        
        elif self.isStanding:
            cmd_robot, cmd_gazebo = self.body_controller_.body_pose()
            print(cmd_robot.position)
            self.joint_state_pub_.publish(cmd_robot)
            if cmd_gazebo is not None:
                    self.gazebo_pub_.publish(cmd_gazebo)
            self.T_world_base_, self.thigh_foot_= self.body_controller_.getT()

        elif self.isRotating:
            pass # To do

        else:
            pass # To do

    def off_mode(self):
        self.isWalking = False
        self.isStanding = False
        self.isRotating = False

        self.gait_controller_.restart()
        self.body_controller_.restart()

    def trot_gait_mode(self):
        self.get_logger().info('Executing walking...')
        self.off_mode()
        self.mode_swap_ = False
        self.swap_initiated_ = False
        self.isWalking = True
    
    def standing_mode(self):
        self.get_logger().info('Executing standing...')
        self.off_mode()
        self.mode_swap_ = False
        self.swap_initiated_ = False
        self.isStanding = True

    def rotating_mode(self):
        self.get_logger().info('Executing rotating...')
        self.off_mode()
        self.mode_swap_ = True
        self.swap_initiated_ = False
        self.isRotating = True

class AppGUI:
    """The Tkinter Graphical Interface"""
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        
        self.root.title("Quadruped Control Panel")
        self.root.geometry("500x700")
        self.root.eval('tk::PlaceWindow . center') 


        label = tk.Label(root, text="Select an Action", font=("Helvetica", 14, "bold"))
        label.pack(pady=10)

        btn_walk = tk.Button(root, text="Walk", command=ros_node.trot_gait_mode, width=20, bg="#4CAF50", fg="white")
        btn_walk.pack(pady=5)

        btn_stand = tk.Button(root, text="Stand", command=ros_node.standing_mode, width=20, bg="#2196F3", fg="white")
        btn_stand.pack(pady=5)

        btn_quit = tk.Button(root, text="Quit Node", command=self.quit_app, width=20, bg="#f44336", fg="white")
        btn_quit.pack(pady=5)

    
        tk.Label(root, text="Body Pose Adjustments (Stand Mode Only)", font=("Helvetica", 12, "bold")).pack(pady=(20, 5))


        self.x_slider = tk.Scale(root, from_=-0.2, to=0.2, resolution=0.01, orient=tk.HORIZONTAL, label="X", length=300, command=self.on_pose_change)
        self.x_slider.set(0.0)
        self.x_slider.pack()


        self.y_slider = tk.Scale(root, from_=-0.2, to=0.2, resolution=0.01, orient=tk.HORIZONTAL, label="Y", length=300, command=self.on_pose_change)
        self.y_slider.set(0.0)
        self.y_slider.pack()

        
        self.z_slider = tk.Scale(root, from_=-0.18, to=-0.01, resolution=0.01, orient=tk.HORIZONTAL, label="Z", length=300, command=self.on_pose_change)
        self.z_slider.set(-0.1)
        self.z_slider.pack()

        
        self.roll_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Roll (Radians)", length=300, command=self.on_pose_change)
        self.roll_slider.pack()

        self.pitch_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Pitch (Radians)", length=300, command=self.on_pose_change)
        self.pitch_slider.pack()

        self.yaw_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Yaw (Radians)", length=300, command=self.on_pose_change)
        self.yaw_slider.pack()

        btn_reset = tk.Button(root, text="Reset Pose", command=self.reset_sliders, width=15)
        btn_reset.pack(pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

    def on_pose_change(self, event=None):
        # Only send updates if the robot is actually in the standing state
        if getattr(self.ros_node, 'isStanding', False):
            desired_pose = {
                'x': float(self.x_slider.get()), 
                'y': float(self.y_slider.get()),
                'z': float(self.z_slider.get()),
                'roll': float(self.roll_slider.get()),
                'pitch': float(self.pitch_slider.get()),
                'yaw': float(self.yaw_slider.get())
            }
            
            self.ros_node.body_controller_.update_target(desired_pose)

    def reset_sliders(self):
        self.x_slider.set(0.0)
        self.y_slider.set(0.0)
        self.z_slider.set(self.ros_node.defaultZ_)
        self.roll_slider.set(0.0)
        self.pitch_slider.set(0.0)
        self.yaw_slider.set(0.0)
        self.on_pose_change()

    def quit_app(self):
        self.ros_node.get_logger().info("Closing GUI and shutting down ROS...")
        self.root.quit()


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