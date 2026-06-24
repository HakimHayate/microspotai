import tkinter as tk
import math

class GUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("gui")
        self.root.geometry("350x600")
        
        self.slider1 = tk.Scale(self.root, from_=-0.3, to=0.30, orient='horizontal', resolution=0.0001, command=self.update1)
        self.slider1.set(self.node.x_desired_[0])
        self.slider1.pack(pady=5)
        self.label1 = tk.Label(self.root, text=f"x: {self.node.x_desired_[0]:.3f}")
        self.label1.pack()
        
        self.slider2 = tk.Scale(self.root, from_=-0.3, to=0.3, orient='horizontal', resolution=0.0001, command=self.update2)
        self.slider2.set(self.node.x_desired_[1])
        self.slider2.pack(pady=5)
        self.label2 = tk.Label(self.root, text=f"y: {self.node.x_desired_[1]:.3f}")
        self.label2.pack()
        
        self.slider3 = tk.Scale(self.root, from_=0, to=0.30, orient='horizontal', resolution=0.0001, command=self.update3)
        self.slider3.set(self.node.x_desired_[2])
        self.slider3.pack(pady=5)
        self.label3 = tk.Label(self.root, text=f"z: {self.node.x_desired_[2]:.3f}")
        self.label3.pack()

        self.slider4 = tk.Scale(self.root, from_=-math.pi, to=math.pi, orient='horizontal', resolution=0.1, command=self.update4)
        self.slider4.set(self.node.x_desired_[3])
        self.slider4.pack(pady=5)
        self.label4 = tk.Label(self.root, text=f"roll: {self.node.x_desired_[3]:.3f}")
        self.label4.pack()
        
        self.slider5 = tk.Scale(self.root, from_=-math.pi, to=math.pi, orient='horizontal', resolution=0.1, command=self.update5)
        self.slider5.set(self.node.x_desired_[4])
        self.slider5.pack(pady=5)
        self.label5 = tk.Label(self.root, text=f"pitch: {self.node.x_desired_[4]:.3f}")
        self.label5.pack()
        
        self.slider6 = tk.Scale(self.root, from_=-math.pi, to=math.pi, orient='horizontal', resolution=0.1, command=self.update6)
        self.slider6.set(self.node.x_desired_[5])
        self.slider6.pack(pady=5)
        self.label6 = tk.Label(self.root, text=f"yaw: {self.node.x_desired_[5]:.3f}")
        self.label6.pack()

        self.slider7 = tk.Scale(self.root, from_=-0.1, to=0.1, orient='horizontal', resolution=0.001, command=self.update7)
        self.slider7.set(self.node.end_effector[2])
        self.slider7.pack(pady=5)
        self.label7 = tk.Label(self.root, text=f"end effector: {self.node.end_effector[2]:.3f}")
        self.label7.pack()

        self.quit_button = tk.Button(self.root, text="Quit", command=self.root.quit, fg="red")
        self.quit_button.pack(pady=20)

    def update1(self, x):
        self.node.x_desired_[0] = float(x)
        self.label1.config(text=f"x: {self.node.x_desired_[0]:.3f}")

    def update2(self, y):
        self.node.x_desired_[1] = float(y)
        self.label2.config(text=f"y: {self.node.x_desired_[1]:.3f}")

    def update3(self, z):
        self.node.x_desired_[2] = float(z)
        self.label3.config(text=f"z: {self.node.x_desired_[2]:.3f}")

    def update4(self, roll):
        self.node.x_desired_[3] = float(roll)
        self.label4.config(text=f"roll: {self.node.x_desired_[3]:.3f}")  # Fixed reference

    def update5(self, pitch):
        self.node.x_desired_[4] = float(pitch)
        self.label5.config(text=f"pitch: {self.node.x_desired_[4]:.3f}")  # Fixed reference

    def update6(self, yaw):
        self.node.x_desired_[5] = float(yaw)
        self.label6.config(text=f"yaw: {self.node.x_desired_[5]:.3f}")  # Fixed reference

    def update7(self, d):
        self.node.end_effector[2] = float(d)
        self.label7.config(text=f"end effector: {self.node.end_effector[2]:.3f}")  # Fixed reference