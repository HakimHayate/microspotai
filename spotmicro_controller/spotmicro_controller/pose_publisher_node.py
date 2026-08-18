import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
import numpy as np


class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.array_publisher_ = self.create_publisher(Float64MultiArray, 'target_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.marker_pub_ = self.create_publisher(Marker, 'target_marker', 10)
        self.end_effector = np.zeros(3)
        self.x_desired_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # [x, y, z, roll, pitch, yaw]

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [
            self.x_desired_[0],  # x
            self.x_desired_[1],  # y
            self.x_desired_[2],  # z
            self.x_desired_[3],  # roll
            self.x_desired_[4],  # pitch
            self.x_desired_[5],   # yaw
            self.end_effector[0],
            self.end_effector[1],
            self.end_effector[2]
        ]
        
        self.array_publisher_.publish(msg)
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "desired_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.x_desired_[0]
        marker.pose.position.y = self.x_desired_[1]
        marker.pose.position.z = self.x_desired_[2]
        
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub_.publish(marker)

import threading
from gui import GUI
import tkinter as tk

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode() 
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()

    root = tk.Tk()
    app = GUI(root, node)
    
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()