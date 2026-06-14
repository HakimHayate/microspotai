import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import numpy as np


def get_correction_angle(x, y, z):
    roll =   -math.atan2(-y, math.sqrt(x*x + z*z))
    pitch = -math.atan2(x, math.sqrt(y*y + z*z))

    return np.array([roll, pitch])


def get_prediction(angular_velocity_x, angular_velocity_y, prev_angle, dt=0.01): 
    # Dynamic motion of model
    return  prev_angle + np.array([angular_velocity_x, angular_velocity_y]) * dt


def complemenary_filter(prediction_angle, correction_angle, alpha=0.95):
    return alpha * prediction_angle + (1-alpha) * correction_angle

def euler_to_quaternion(euler_angle):
    
    roll = euler_angle[0]
    pitch = euler_angle[1]
    yaw = 0.0  

    
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return np.array([qx, qy, qz, qw])


class MyNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')
        self.filtered_angle = np.zeros(2)
        self.imu_sub_ = self.create_subscription(
            Imu, 
            '/imu/data_raw', 
            self.filter_imu_callback, 
            10)
        self.imu_pub_ = self.create_publisher(Imu,
                                              '/imu/data',
                                              10)
        

    def filter_imu_callback(self, msg):
        linear_acceleration = msg.linear_acceleration
        angular_velocity = msg.angular_velocity

        correction_angle = get_correction_angle(linear_acceleration.x,
                                            linear_acceleration.y,
                                            linear_acceleration.z)
        
        prediction_angle = get_prediction(angular_velocity.x,
                                    angular_velocity.y,
                                    self.filtered_angle)
        
        self.filtered_angle = complemenary_filter(prediction_angle=prediction_angle,
                                                  correction_angle=correction_angle)

        q = euler_to_quaternion(self.filtered_angle)

        filtered_msg = Imu()
        filtered_msg.header = msg.header

        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.linear_acceleration = msg.linear_acceleration
        filtered_msg.orientation.x = q[0]
        filtered_msg.orientation.y = q[1]
        filtered_msg.orientation.z = q[2]
        filtered_msg.orientation.w = q[3]

        self.imu_pub_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()