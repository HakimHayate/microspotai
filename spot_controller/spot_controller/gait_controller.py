from std_msgs.msg import Float64MultiArray
import math
from leg_ik_solver import LegIKSolver
from sensor_msgs.msg import JointState


class GaitController():
    def __init__(self, controller_node, joints_names, stride_length=0.06, 
                 stride_height=0.05, stance_depth=-0.25, 
                 len_hip = 0.05, len_thigh= 0.13, 
                 len_knee = 0.13):

        self.controller_node_ = controller_node
        self.ik_solver_ = LegIKSolver(len_hip, len_thigh, len_knee)
        
        self.joints_names_ = joints_names

        self.current_positions = [0.0] * 12
        self.time_elapsed_ = 0.0
        self.gait_speed = 4.0

        self.L = stride_length
        self.H = stride_height
        self.X0 = 0.17

    
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

    def restart(self):
        self.time_elapsed_ = 0

    def trot_gait(self, reset=False):
        if reset:
            self.time_elapsed_ = 0
            
        self.time_elapsed_ += 0.02
        base_phase = self.time_elapsed_ * self.gait_speed

        fr_x, fr_z = self.get_foot_coordinate(base_phase)
        bl_x, bl_z = self.get_foot_coordinate(base_phase)
        
        fl_x, fl_z = self.get_foot_coordinate(base_phase + math.pi)
        br_x, br_z = self.get_foot_coordinate(base_phase + math.pi)
        
        
        try:
            fr_h, fr_t, fr_k = self.ik_solver_.solve(x=fr_x, y=0, z=fr_z)
            br_h, br_t, br_k = self.ik_solver_.solve(x=br_x,   y=0, z=br_z)
            bl_h, bl_t, bl_k = self.ik_solver_.solve(x=bl_x,   y=0,  z=bl_z)
            fl_h, fl_t, fl_k = self.ik_solver_.solve(x=fl_x, y=0,  z=fl_z)
            
            command_msg = JointState()
            command_msg.header.stamp = self.controller_node_.get_clock().now().to_msg()
            command_msg.name = self.joints_names_
            command_msg.position = [
                fr_h, fr_t, fr_k,   # Front Right Leg
                br_h, br_t, br_k,   # Back Right Leg
                bl_h, bl_t, bl_k,   # Back Left Leg
                fl_h, fl_t, fl_k    # Front Left Leg
            ]
            
        except ValueError as err:
            self.controller_node.get_logger().warn(f"Gait target error dropped: {err}")
        
        return command_msg