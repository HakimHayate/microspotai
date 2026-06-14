import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_driver import ImuDriver 
from calibration import load_calibration

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.get_logger().info('IMU node initiated')

        self.imu_publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        self.timer_ = self.create_timer(0.05, self.imu_callback) 
        
        self.calibration_pathfile_ = 'imu_calibration.json'

        self.imu_driver_ = ImuDriver(calibration=load_calibration(self.calibration_pathfile_))


    def imu_callback(self):
        msg = Imu()
        
        msg.header.stamp = self.get_clock().now().to_msg()

        data = self.imu_driver_.read()
        accel = data['accel']
        gyro = data['gyro']

        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']

        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        self.imu_publisher_.publish(msg)
        
        msg.orientation_covariance[0] = -1 

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
