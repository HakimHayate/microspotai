import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import json
from adafruit_servokit import ServoKit

class RealRobotHardwareBridge(Node):
    def __init__(self):
        super().__init__('real_robot_hardware_bridge')
        
        calibrate_file = 'calibrate.json'
        micro_config_file = 'micro_config.json'
        
        try:
            with open(calibrate_file, 'r') as f:
                self.calibrate_data = json.load(f)
            with open(micro_config_file, 'r') as f:
                self.micro_config = json.load(f)
        except FileNotFoundError as e:
            self.get_logger().error(f"Could not find config file: {e}")
            raise

        self.joint_offsets = self.calibrate_data['joints']
        self.direction_rules = self.calibrate_data['direction']
        
        # Initialize Boards
        self.boards = {
            board_name: ServoKit(channels=16, address=addr)
            for board_name, addr in self.micro_config['boards'].items()
        }

        self.name_map = {
            "FL_Hip": "front_left_hip_joint", "FL_Thigh": "front_left_thigh_joint", "FR_Thigh": "front_right_thigh_joint",
            "FR_Hip": "front_right_hip_joint", "FL_Knee": "front_left_knee_joint", "FR_Knee": "front_right_knee_joint",
            "BR_Knee": "back_right_knee_joint", "BL_Hip": "back_left_hip_joint", "BR_Hip": "back_right_hip_joint",
            "BL_Knee": "back_left_knee_joint", "BR_Thigh": "back_right_thigh_joint", "BL_thigh": "back_left_thigh_joint"
        }

        # Format: { 'joint_name': (servo_object, offset, direction_multiplier) }
        self.fast_lookup = {}
        

        for short_name, config in self.micro_config['servos'].items():
            if short_name in self.name_map:
                std_name = self.name_map[short_name]
                board_name = config["board"]
                pin = config["pin"]
                
                servo_obj = self.boards[board_name].servo[pin]
                servo_obj.actuation_range = 180
                servo_obj.set_pulse_width_range(500, 2500)
                
                offset = self.joint_offsets.get(std_name, 90.0)
                dir_mult = self.direction_rules.get("right", 1) if "right" in std_name else self.direction_rules.get("left", 1)
                
                self.fast_lookup[std_name] = (servo_obj, offset, dir_mult)

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info('Hardware bridge initialized')

    def joint_state_callback(self, msg):
        rad_to_deg = 57.2957795 

        for i, joint_name in enumerate(msg.name):
            if joint_name in self.fast_lookup:
                servo_obj, offset, dir_mult = self.fast_lookup[joint_name]
                
                servo_degree = (dir_mult * (msg.position[i] * rad_to_deg)) + offset
                servo_degree = max(0.0, min(180.0, servo_degree))
                
                servo_obj.angle = servo_degree
                    

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()