import numpy as np
import math
import threading
import time

from spotmicro_mapping.icp_point2line import icp_point2line as clean_icp
from spotmicro_mapping.graph import Graph
from spotmicro_mapping.se2 import v2t, t2v, relative, wrap_angle
from spotmicro_mapping.optimizer import magic_optimizer
from spotmicro_mapping.map_manager import OccupancyGridMap

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class FrontEndNode(Node):
    def __init__(self):
        super().__init__('front_end_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.local_map = OccupancyGridMap()
        self.pose = np.zeros(3)
        self.prev_pose = np.zeros(3)
        self.prev_pose_backend = np.zeros(3)
        self.prev_yaw = 0.0
        self.current_yaw = 0.0
        self.init = True
        self.guess = np.zeros(3)

        self.accumulated_dist = 0
        
        self.sub_imu_yaw = self.create_subscription(
            Float64,
            '/imu/yaw',
            self.yaw_callback,
            10)
        
        self.sub_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.keyframe_pub_ = self.create_publisher(Float32MultiArray, '/slam/keyframe', 10)

    def yaw_callback(self, msg):
        self.current_yaw = -msg.data

    def pub_keyframe(self, pose, pts):
        msg = Float32MultiArray()
        msg.data = np.hstack((pose, pts.flatten())).tolist()
        self.keyframe_pub_.publish(msg)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_mask = (ranges <= msg.range_max) & (ranges >= msg.range_min)

        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        pts = np.zeros((len(ranges), 2))
        pts[:,0] = ranges * np.cos(angles)
        pts[:,1] = ranges * np.sin(angles)

        self.current_pts = pts
        self.new_pts = True

        self.guess[:2] = self.pose[:2]
        self.guess[-1] = wrap_angle(self.pose[2]+self.current_yaw - self.prev_yaw)

        self.pose, score, pts_map, robot_pos_map = self.local_map.match(self.guess, pts)
        self.prev_yaw = self.current_yaw
        
        if not self.init and np.linalg.norm(self.pose[:2] - self.prev_pose[:2]) < 0.1 and abs(self.pose[2] - self.prev_pose[2]) < np.deg2rad(1):
            pass
        else:
            self.pub_keyframe(self.pose, pts)
            self.prev_pose_backend = self.pose.copy()
            self.accumulated_dist += np.linalg.norm(self.pose[:2] - self.prev_pose[:2])

            

            self.local_map.update(pts_map, robot_pos_map)
            self.prev_pose = self.pose.copy()
            if self.init:
                self.init = False

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        
        tf_msg.transform.translation.x = float(self.pose[0])
        tf_msg.transform.translation.y = float(self.pose[1])
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(self.pose[2] / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.pose[2] / 2.0)
        
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontEndNode()
    

    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()