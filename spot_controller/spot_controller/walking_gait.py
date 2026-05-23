#!/usr/bin/env copper
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
from .leg_ik_solver import LegIKSolver
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# Import geometric point types
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from rclpy.time import Time

class MicroSpotManualController(Node):
    def __init__(self, stride_length=0.06, stride_height=0.05, stance_depth=-0.25):
        super().__init__('walking_gait')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        l1 = 0.05  # Hip offset
        l2 = 0.13  # Thigh link length
        l3 = 0.13  # Knee link length

        self.ik_fr = LegIKSolver(l2, l3)
        self.ik_br = LegIKSolver(l2, l3)
        self.ik_bl = LegIKSolver(l2, l3)
        self.ik_fl = LegIKSolver(l2, l3)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/raw_position_bridge/commands', 
            10
        )

        self.joint_names = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]

        # Check if walking works properly in Rviz
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Internal placeholders
        self.current_positions = [0.0] * 12
        self.time_elapsed = 0.0
        self.gait_speed = 4.0

        self.L = stride_length
        self.H = stride_height
        self.X0 = 0.17

    def transform(self, frame_start, frame_end, x, y, z):
        foot_point = PointStamped()
        foot_point.header.stamp = Time().to_msg()
        foot_point.header.frame_id = frame_start 
        foot_point.point.x = x
        foot_point.point.y = y
        foot_point.point.z = z

        try:
            return self.tf_buffer.transform(foot_point, frame_end)

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform frame: {ex}')
            return None
        
    
    def get_foot_coordinate(self, phase):
        """
        Takes a phase parameter from 0 to 2*pi and returns target (X, Z) coordinates.
        """
        phi = phase % (2 * math.pi)
        
        if phi < math.pi:
            z = (self.L / 2.0) * math.cos(phi + math.pi)
            x = self.X0 - (self.H * math.sin(phi))
            
        else:
            stance_progress = (phi - math.pi) / math.pi
            z = (self.L / 2.0) - (self.L * stance_progress)
            x = self.X0
            
        return x, z

    def joint_state_callback(self, msg):
        """ This gathers the live 'state_interface' data from Gazebo """
        if len(msg.position) >= 12:
            self.current_positions = msg.position

    def control_loop(self):
        self.time_elapsed += 0.02
        base_phase = self.time_elapsed * self.gait_speed

        fr_x, fr_z = self.get_foot_coordinate(base_phase)
        bl_x, bl_z = self.get_foot_coordinate(base_phase)
        
        fl_x, fl_z = self.get_foot_coordinate(base_phase + math.pi)
        br_x, br_z = self.get_foot_coordinate(base_phase + math.pi)

        '''
        fr_point = self.transform('front_right_feet', 'front_right_hip', fr_x, 0.0, fr_z)
        bl_point = self.transform('back_left_feet', 'back_left_hip', bl_x, 0.0, bl_z)
        fl_point = self.transform('front_left_feet', 'front_left_hip', fl_x, 0.0, fl_z)
        br_point = self.transform('back_right_feet', 'back_right_hip', br_x, 0.0, br_z)
        

        if fr_point is None or bl_point is None or fl_point is None or br_point is None:
            self.get_logger().info("Waiting for TF buffer to populate transforms...")
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [0.0, 0.4, -0.8] * 4 
            self.joint_pub.publish(msg)
            return  
        

        # Extract the calculated coordinates relative to the hip
        fr_x_hip, fr_z_hip = fr_point.point.x, fr_point.point.z
        br_x_hip, br_z_hip = br_point.point.x, br_point.point.z
        bl_x_hip, bl_z_hip = bl_point.point.x, bl_point.point.z
        fl_x_hip, fl_z_hip = fl_point.point.x, fl_point.point.z
        '''

        try:
            fr_h, fr_t, fr_k = self.ik_fr.solve(x=fr_x, y=0, z=fr_z)
            br_h, br_t, br_k = self.ik_br.solve(x=br_x,   y=0, z=br_z)
            bl_h, bl_t, bl_k = self.ik_bl.solve(x=bl_x,   y=0,  z=bl_z)
            fl_h, fl_t, fl_k = self.ik_fl.solve(x=fl_x, y=0,  z=fl_z)
            
            command_msg = Float64MultiArray()
            command_msg.data = [
                fr_h, fr_t, fr_k,   # Front Right Leg
                br_h, br_t, br_k,   # Back Right Leg
                bl_h, bl_t, bl_k,   # Back Left Leg
                fl_h, fl_t, fl_k    # Front Left Leg
            ]
            
            self.cmd_pub.publish(command_msg)

            msg = JointState()
        
            # synchronize the message with the current ROS time clock
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names

            msg.position = [
                fr_h, fr_t, fr_k,   
                br_h, br_t, br_k,   
                bl_h, bl_t, bl_k,   
                fl_h, fl_t, fl_k    
            ]

            self.joint_pub.publish(msg)
            
        except ValueError as err:
            self.get_logger().warn(f"Gait target error dropped: {err}")


def main(args=None):
    rclpy.init(args=args)
    node = MicroSpotManualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()