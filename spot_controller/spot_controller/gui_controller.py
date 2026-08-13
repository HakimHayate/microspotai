import tkinter as tk

class AppGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        
        self.root.title("Microspot Control Panel")
        self.root.geometry("550x900") 
        self.root.eval('tk::PlaceWindow . left') 

        # --- MODE SELECTION ---
        label = tk.Label(root, text="Select an Action", font=("Helvetica", 14, "bold"))
        label.pack(pady=5)

        btn_walk = tk.Button(root, text="Walk", command=ros_node.trot_gait_mode, width=20, bg="#4CAF50", fg="white")
        btn_walk.pack(pady=2)

        btn_stand = tk.Button(root, text="Stand Mode", command=ros_node.standing_mode, width=20, bg="#2196F3", fg="white")
        btn_stand.pack(pady=2)

        btn_rotate = tk.Button(root, text="Rotate", command=ros_node.rotating_mode, width=20, bg="#2196F3", fg="white")
        btn_rotate.pack(pady=2)

        btn_arm = tk.Button(root, text="Move Arm", command=ros_node.arm_mode, width=20, bg="#2196F3", fg="white")
        btn_arm.pack(pady=2)

        btn_auto = tk.Button(root, text="Auto", command=ros_node.auto_mode, width=20, bg="#2196F3", fg="white")
        btn_auto.pack(pady=2)

        btn_sit = tk.Button(root, text="Sit", command=ros_node.sit_mode, width=20, bg="#FF9800", fg="white")
        btn_sit.pack(pady=2)

        btn_standup = tk.Button(root, text="Standup", command=ros_node.standUp_mode, width=20, bg="#FF9800", fg="white")
        btn_standup.pack(pady=2)
        
        btn_quit = tk.Button(root, text="Quit Node", command=self.quit_app, width=20, bg="#f44336", fg="white")
        btn_quit.pack(pady=2)


        tk.Label(root, text="Gait Tuning (Walk Mode Only)", font=("Helvetica", 12, "bold")).pack(pady=(15, 0))

        self.stride_slider = tk.Scale(root, from_=0.0, to=0.15, resolution=0.01, orient=tk.HORIZONTAL, label="Stride Length (B)", length=300, command=self.on_gait_change)
        self.stride_slider.set(0.04) 
        self.stride_slider.pack()

        self.height_slider = tk.Scale(root, from_=0.01, to=0.10, resolution=0.01, orient=tk.HORIZONTAL, label="Step Height (H)", length=300, command=self.on_gait_change)
        self.height_slider.set(0.03) 
        self.height_slider.pack()

        
        self.duration_slider = tk.Scale(root, from_=0.5, to=3.0, resolution=0.1, orient=tk.HORIZONTAL, label="Cycle Duration (Seconds)", length=300, command=self.on_gait_change)
        self.duration_slider.set(1.5) 
        self.duration_slider.pack()

        self.swing_time_slider = tk.Scale(root, from_=0.2, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, label="Swing Time (Max 4.0)", length=300, command=self.on_gait_change)
        self.swing_time_slider.set(0.8) 
        self.swing_time_slider.pack()

        self.turn_slider = tk.Scale(root, from_=-0.04, to=0.04, resolution=0.005, orient=tk.HORIZONTAL, label="Turn Rate (L <-> R)", length=300, command=self.on_gait_change)
        self.turn_slider.set(0.0) 
        self.turn_slider.pack()

        tk.Label(root, text="Body Pose Adjustments (Stand Mode Only)", font=("Helvetica", 12, "bold")).pack(pady=(15, 0))

        self.x_slider = tk.Scale(root, from_=-0.1, to=0.1, resolution=0.01, orient=tk.HORIZONTAL, label="X", length=300, command=self.on_pose_change)
        self.x_slider.set(0.05)
        self.x_slider.pack()

        self.y_slider = tk.Scale(root, from_=-0.1, to=0.1, resolution=0.01, orient=tk.HORIZONTAL, label="Y", length=300, command=self.on_pose_change)
        self.y_slider.set(0.0)
        self.y_slider.pack()
        
        self.z_slider = tk.Scale(root, from_=-0.3, to=0.02, resolution=0.01, orient=tk.HORIZONTAL, label="Z", length=300, command=self.on_pose_change)
        self.z_slider.set(-0.14)
        self.z_slider.pack()
        
        self.roll_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Roll (Radians)", length=300, command=self.on_pose_change)
        self.roll_slider.pack()

        self.pitch_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Pitch (Radians)", length=300, command=self.on_pose_change)
        self.pitch_slider.pack()

        self.yaw_slider = tk.Scale(root, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, label="Yaw (Radians)", length=300, command=self.on_pose_change)
        self.yaw_slider.pack()

        btn_reset = tk.Button(root, text="Reset Pose & Gait", command=self.reset_sliders, width=20)
        btn_reset.pack(pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

    def on_gait_change(self, event=None):
        new_B = float(self.stride_slider.get())
        new_H = float(self.height_slider.get())
        new_duration = float(self.duration_slider.get())
        new_swing_time = float(self.swing_time_slider.get())
        new_turn = float(self.turn_slider.get())
        
        if getattr(self.ros_node, 'isStanding', False):
            self.ros_node.B_ = new_B
            self.ros_node.H_ = new_H
            self.ros_node.duration_ = new_duration
            self.ros_node.swing_time_ = new_swing_time
            self.ros_node.turn_rate_ = new_turn

            if hasattr(self.ros_node, 'send_command'):
                self.ros_node.send_command("pc_control")

    def on_pose_change(self, event=None):
        if getattr(self.ros_node, 'isStanding', False):
            desired_pose = {
                'x': float(self.x_slider.get()), 
                'y': float(self.y_slider.get()),
                'z': float(self.z_slider.get()),
                'roll': float(self.roll_slider.get()),
                'pitch': float(self.pitch_slider.get()),
                'yaw': float(self.yaw_slider.get())
            }
            
            self.ros_node.body_controller_.update_target(self.ros_node.T_world_base_, desired_pose)

    def reset_sliders(self):
        # Reset Pose
        self.x_slider.set(0.04)
        self.y_slider.set(0.01)
        self.z_slider.set(-0.08)
        self.roll_slider.set(0.0)
        self.pitch_slider.set(0.0)
        self.yaw_slider.set(0.0)
        
        # Reset Gait
        self.stride_slider.set(0.04)
        self.height_slider.set(0.03)
        self.duration_slider.set(1.5)
        self.swing_time_slider.set(0.8)
        
        self.on_pose_change()
        self.on_gait_change()

    def quit_app(self):
        self.ros_node.get_logger().info("Closing GUI and shutting down ROS...")
        self.root.quit()