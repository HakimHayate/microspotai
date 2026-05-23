import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from adafruit_servokit import ServoKit

class RealRobotHardwareBridge(Node):
    def __init__(self):
        super().__init__('real_robot_hardware_bridge')
        
        self.kit = ServoKit(channels=16)


        self.joint_to_channel = {
            # FRONT RIGHT (RF)
            'front_right_hip_joint': 0,
            'front_right_thigh_joint': 1,
            'front_right_knee_joint': 2,

            # FRONT LEFT (LF)
            'front_left_hip_joint': 4,
            'front_left_thigh_joint': 5,
            'front_left_knee_joint': 6,

            # BACK LEFT (BL)
            'back_left_hip_joint': 8,
            'back_left_thigh_joint': 9,
            'back_left_knee_joint': 10,

            # BACK RIGHT (BR)
            'back_right_hip_joint': 12,
            'back_right_thigh_joint': 13,
            'back_right_knee_joint': 15
        }


        self.joint_offsets = {
            # FRONT RIGHT (RF)
            'front_right_hip_joint': 90,
            'front_right_thigh_joint': 90,
            'front_right_knee_joint': 5,

            # FRONT LEFT (LF)
            'front_left_hip_joint': 85,
            'front_left_thigh_joint': 100,
            'front_left_knee_joint': 0,

            # BACK LEFT (BL)
            'back_left_hip_joint': 0,
            'back_left_thigh_joint': 80,
            'back_left_knee_joint': 10,

            # BACK RIGHT (BR)
            'back_right_hip_joint': 0,
            'back_right_thigh_joint': 95,
            'back_right_knee_joint': -15
        }

        self.direction = {
            # FRONT RIGHT (RF)
            'front_right_hip_joint': 1,
            'front_right_thigh_joint': 1,
            'front_right_knee_joint': 1,

            # FRONT LEFT (LF)
            'front_left_hip_joint': -1,
            'front_left_thigh_joint': -1,
            'front_left_knee_joint': -1,

            # BACK LEFT (BL)
            'back_left_hip_joint': -1,
            'back_left_thigh_joint': -1,
            'back_left_knee_joint': -1,

            # BACK RIGHT (BR)
            'back_right_hip_joint': 1,
            'back_right_thigh_joint': 1,
            'back_right_knee_joint': 1
        }
        
        
        for channel in self.joint_to_channel.values():
            self.kit.servo[channel].actuation_range = 180
            self.kit.servo[channel].set_pulse_width_range(500, 2500)

        # Subscribe to the joint calculations coming from Brain node
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Hardware bridge initialized')
        

    def joint_state_callback(self, msg):
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_to_channel:
                channel = self.joint_to_channel[joint_name]
                radian_angle = msg.position[i]
                
                servo_degree = self.direction[joint_name] *  math.degrees(radian_angle) + self.joint_offsets[joint_name]
                servo_degree = max(0.0, min(180, servo_degree))
                
                # Push directly down the physical I2C wire to the motor
                self.kit.servo[channel].angle = servo_degree

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down the hardware bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()