import rclpy
from rclpy.node import Node
import math
import json
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from se2 import wrap_angle

class ExecutePath(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.pose_sub = self.create_subscription(
            Float32MultiArray,
            '/current_pose', 
            self.pose_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path', 
            self.path_callback,
            10
        )

        self.mode_ = "manuel"
        self.cmd_pub = self.create_publisher(String, '/robot_commands', 1)

        self.subscription = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )


        self.current_pose = None
        self.current_path = []

        
        self.lookahead_distance = 0.3  
        self.Kp_turn = 0.03 
        self.walking_speed_B = 0.06 
        self.thresh_arrived = 0.15

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Path executer ready.")

    def mode_callback(self, msg):
        self.mode_ = msg.data
        self.get_logger().info(f'Received Mode: "{self.mode_}"')
        
    def pose_callback(self, msg):
        print('pose received')
        self.current_pose = np.array(msg.data, dtype=np.float32)

    def path_callback(self, msg):
        self.current_path = msg.poses
        self.get_logger().info("New path received...")

    def control_loop(self):
        if self.mode_ != 'auto':
            return
        
        if self.current_pose is None:
            self.get_logger().info("I don't know where am I. Move around so I can localize myself...")
            return 
        
        if not self.current_path:
            self.get_logger().info("No path has been planned yet...")
            return 
        
        final_goal = self.current_path[-1].pose.position
        final_goal_arr = np.array([final_goal.x, final_goal.y])
        dist_to_goal = np.linalg.norm(final_goal_arr - self.current_pose[:2])
        
        if dist_to_goal < self.thresh_arrived: 
            self.get_logger().info("Goal reached!")
            self.current_path = None
            self.send_robot_command("stand", 0.0, 0.0)
            return
        
        inter_x = final_goal.x
        inter_y = final_goal.y
        
        closest_dist = float('inf')
        closest_index = 0
        
        for i, pose_stamped in enumerate(self.current_path):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - self.current_pose[0], py - self.current_pose[1])
            
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
                
        for i in range(closest_index, len(self.current_path)):
            px = self.current_path[i].pose.position.x
            py = self.current_path[i].pose.position.y
            dist = math.hypot(px - self.current_pose[0], py - self.current_pose[1])
            
            if dist >= self.lookahead_distance:
                inter_x = px
                inter_y = py
                break 
        
        theta_heading = np.atan2(-inter_y + self.current_pose[1], -inter_x+self.current_pose[0])
        print(f'theta heading {theta_heading}')
        heading_error = wrap_angle(theta_heading - self.current_pose[2])
        print(f'heading error {heading_error}')
        turn_rate = self.Kp_turn * heading_error
        
        turn_rate = max(-0.03, min(0.03, turn_rate)) 

        self.send_robot_command("walk", self.walking_speed_B, turn_rate)

    def send_robot_command(self, mode, stride, turn):
        print('sending command to pi')
        command_data = {
            "mode": mode,
            "B": 0.05,
            "H": 0.06, 
            "duration": 1.5, 
            "swing_time": 0.60,
            "turn_rate": float(turn)
        }
        msg = String()
        msg.data = json.dumps(command_data)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExecutePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()