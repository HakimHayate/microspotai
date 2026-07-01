import tkinter as tk


class AppGUI:
    """The Tkinter Graphical Interface"""
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        
        self.root.title("Microspot Control Panel")
        self.root.geometry("500x700")
        self.root.eval('tk::PlaceWindow . left') 


        label = tk.Label(root, text="Select an Action", font=("Helvetica", 14, "bold"))
        label.pack(pady=10)

        btn_walk = tk.Button(root, text="Walk", command=ros_node.trot_gait_mode, width=20, bg="#4CAF50", fg="white")
        btn_walk.pack(pady=5)

        btn_stand = tk.Button(root, text="Stand", command=ros_node.standing_mode, width=20, bg="#2196F3", fg="white")
        btn_stand.pack(pady=5)

        btn_rotate = tk.Button(root, text="Rotate", command=ros_node.rotating_mode, width=20, bg="#2196F3", fg="white")
        btn_rotate.pack(pady=5)

        btn_arm = tk.Button(root, text="Move Arm", command=ros_node.arm_mode, width=20, bg="#2196F3", fg="white")
        btn_arm.pack(pady=5)
        
        btn_quit = tk.Button(root, text="Quit Node", command=self.quit_app, width=20, bg="#f44336", fg="white")
        btn_quit.pack(pady=5)

    
        tk.Label(root, text="Body Pose Adjustments (Stand Mode Only)", font=("Helvetica", 12, "bold")).pack(pady=(20, 5))


        self.x_slider = tk.Scale(root, from_=-0.2, to=0.2, resolution=0.01, orient=tk.HORIZONTAL, label="X", length=300, command=self.on_pose_change)
        self.x_slider.set(0.0)
        self.x_slider.pack()


        self.y_slider = tk.Scale(root, from_=-0.2, to=0.2, resolution=0.01, orient=tk.HORIZONTAL, label="Y", length=300, command=self.on_pose_change)
        self.y_slider.set(0.0)
        self.y_slider.pack()

        
        self.z_slider = tk.Scale(root, from_=-0.25, to=0.1, resolution=0.01, orient=tk.HORIZONTAL, label="Z", length=300, command=self.on_pose_change)
        self.z_slider.set(0)
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