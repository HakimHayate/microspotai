import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import math

# Import your custom functions
from se2 import t2v, v2t, project
from icp_point2line_teacher import icp_point2line

def euler_to_quaternion(yaw):
    """Converts yaw angle to a ROS-compatible quaternion."""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

class LaserOdometryNode(Node):
    def __init__(self):
        super().__init__('laser_odometry_node')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.yaw_sub = self.create_subscription(Float64, '/imu/yaw', self.yaw_callback, 10)
        
        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        self.prev_pts = None
        self.current_imu_yaw = None
        self.last_scan_imu_yaw = None
        
        self.global_pose = np.array([0.0, 0.0, 0.0])
        self.last_delta = np.array([0.0, 0.0, 0.0])
        
        # Publish a static transform from base_link to laser (Assuming laser is at center of robot)
        self.publish_static_tf()

    def publish_static_tf(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'base_link'
        static_transformStamped.child_frame_id = 'laser_frame' # Change this if your LaserScan frame_id is different
        
        # Zero offset (assuming laser is in middle of robot)
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(static_transformStamped)

    def yaw_callback(self, msg):
        self.current_imu_yaw = msg.data

    def scan_to_points(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid_mask = (np.isfinite(ranges)) & (ranges > msg.range_min) & (ranges < msg.range_max)
        return np.column_stack((ranges[valid_mask] * np.cos(angles[valid_mask]), 
                                ranges[valid_mask] * np.sin(angles[valid_mask])))

    def scan_callback(self, msg):
        current_pts = self.scan_to_points(msg)
        
        if self.prev_pts is None:
            self.prev_pts = current_pts
            if self.current_imu_yaw is not None:
                self.last_scan_imu_yaw = self.current_imu_yaw
            return

        try:
            dx, dy, dth = self.last_delta
            
            # Use IMU for rotation guess if available
            if self.current_imu_yaw is not None and self.last_scan_imu_yaw is not None:
                dth = self.current_imu_yaw - self.last_scan_imu_yaw
                dth = np.arctan2(np.sin(dth), np.cos(dth))
                self.last_scan_imu_yaw = self.current_imu_yaw

            # Frame-to-Frame ICP for fast odometry
            guess_pose = np.array([dx, dy, dth])
            T_opt, raw_delta = icp_point2line(self.prev_pts, current_pts, guess_pose)
            
            self.last_delta = raw_delta
            
            # Update global odometry
            self.global_pose = t2v(v2t(self.global_pose) @ T_opt)
            
            # --- PUBLISH ODOMETRY TO TF TREE ---
            t = TransformStamped()
            t.header.stamp = msg.header.stamp # Sync with laser scan time!
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = float(self.global_pose[0])
            t.transform.translation.y = float(self.global_pose[1])
            t.transform.translation.z = 0.0
            
            quat = euler_to_quaternion(self.global_pose[2])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)
            
            self.prev_pts = current_pts
            
        except Exception as e:
            self.get_logger().error(f"Odometry failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()