import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class IcpSlamNode(Node):
    def __init__(self):
        super().__init__("icp_slam_node")
        self.scan_sub_ = self.create_subsciption(LaserScan, '/scan', self.icp_callback, 10)
    
        

