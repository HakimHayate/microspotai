import rclpy
from rclpy.node import Node
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray

from spotmicro_controller.gait_controller import GaitController
from spotmicro_controller.leg_ik_solver import LegIKSolver

class PiWalkerNode(Node):
    def __init__(self, len_hip=0.06, len_thigh=0.13, len_knee=0.13):
        super().__init__('pi_walker_node')

        self.solver_ = LegIKSolver(len_hip, len_thigh, len_knee)
        self.links_ = ['front_right', 'back_right', 'back_left', 'front_left']
        self.joint_names_ = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]
        self.dir_motor_ = {'front_right': 1, 'back_right': 1, 'front_left': 1, 'back_left': 1}

        self.gait_controller_ = GaitController(self.solver_, self.links_)

        self.current_mode = "pc_control"

        self.thigh_foot_ = None

        self.cmd_sub = self.create_subscription(String, '/robot_commands', self.command_callback, 1)
        self.pc_coord_sub = self.create_subscription(Float64MultiArray, '/pc_thigh_foot', self.pc_coord_callback, 1)

        self.joint_state_pub_ = self.create_publisher(JointState, '/joint_states', 1)

        self.timer_ = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Pi Walker Node initialized. Central IK Solver ready.")

    def command_callback(self, msg):
        data = json.loads(msg.data)
        self.current_mode = data.get("mode", "pc_control")

        if self.current_mode == "walk":
            self.gait_controller_.update_parameters(
                B=data['B'], H=data['H'],
                duration=data['duration'], swing_time=data['swing_time'],
                turn_rate=data.get('turn_rate', 0.0)
            )

    def pc_coord_callback(self, msg):
        if self.current_mode == "pc_control":
            # Rebuild the dictionary from the 12 incoming floats
            if self.thigh_foot_ is None:
                self.thigh_foot_ = {}
            for i, link in enumerate(self.links_):
                idx = i * 3
                self.thigh_foot_[link] = list(msg.data[idx:idx+3])

    def get_command(self, thigh_foot):
        command = [0] * 12
        for i, link in enumerate(self.links_):
            d = self.dir_motor_[link]
            idx = i * 3
            q = self.solver_.solve(thigh_foot[link][0], thigh_foot[link][1], thigh_foot[link][2])
            q = [qi * d for qi in q]
            command[idx:idx+3] = q
        return command

    def control_loop(self):
        target_thigh_foot = None

        if self.current_mode == "walk":
            if self.thigh_foot_ is None:
                return
            target_thigh_foot = self.gait_controller_.trot_gait(self.thigh_foot_)
        elif self.current_mode == "pc_control":
            target_thigh_foot = self.thigh_foot_

        if target_thigh_foot is not None:
            command = self.get_command(thigh_foot=target_thigh_foot)

            cmd_robot = JointState()
            cmd_robot.header.stamp = self.get_clock().now().to_msg()
            cmd_robot.name = self.joint_names_
            cmd_robot.position = command
            self.joint_state_pub_.publish(cmd_robot)

def main(args=None):
    rclpy.init(args=args)
    node = PiWalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()