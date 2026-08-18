import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        self.filtered_angle = np.zeros(2)

        self.yaw_ = 0.0

        self.last_time_ = None

        self.imu_sub_ = self.create_subscription(
            Imu, 
            '/imu/data_raw', 
            self.filter_imu_callback, 
            10)
        
        self.yaw_pub_ = self.create_publisher(Float64, '/imu/yaw', 20)

    def filter_imu_callback(self, msg):
        angular_velocity = msg.angular_velocity
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time_ is not None:
            dt = current_time - self.last_time_
            if np.abs(angular_velocity.z * dt) > 0.0005:
                self.yaw_ += angular_velocity.z * dt
        self.get_logger().info(f'yaw = {self.yaw_}')
        msg_yaw = Float64()
        msg_yaw.data = self.yaw_
        self.yaw_pub_.publish(msg_yaw)

        self.last_time_ = current_time
  
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()