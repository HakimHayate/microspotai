import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
from leg_ik_solver import LegIKSolver
from utils import get_twist, update_pos
from scipy.spatial.transform import Rotation as R

class ChassisControl(Node):
    def __init__(self, target_x=0, target_y=0, target_z=-0.2):
        super().__init__("ChassisControl")
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )

        self.joint_names = [
            'front_right_hip_joint', 'front_right_thigh_joint', 'front_right_knee_joint',
            'back_right_hip_joint',  'back_right_thigh_joint',  'back_right_knee_joint',
            'back_left_hip_joint',   'back_left_thigh_joint',   'back_left_knee_joint',
            'front_left_hip_joint',  'front_left_thigh_joint',  'front_left_knee_joint'
        ]

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        self.T_current_ = np.eye(4) # I consider the frame attached to the chassis as the global origin

        l2 = 0.13  # Thigh link length
        l3 = 0.13  # Knee link length

        self.solver_ = LegIKSolver(l2, l3)

        self.duration_ = 10.0
        self.time_elapsed_ = 0.0
        self.dt_ = 0.01

        self.timer_ = self.create_timer(self.dt_, self.control_loop)
        self.vx_, self.vy_, self.vz_ = get_twist(target_x, target_y, target_z, 0, 0, 0, t = self.duration_) # I assume chassis is the global origin


    def control_loop(self):
        if self.time_elapsed_>= self.duration_:
            self.vx_, self.vy_, self.vz_ = 0.0, 0.0, 0.0
        else:
            self.time_elapsed_ += self.dt_

        command = [0] * 12 # I'll put the angles for the 12 joints here
        current_x, current_y, current_z = self.T_current_[0, -1], self.T_current_[1, -1], self.T_current_[2, -1]
        
        # Robot moves slighly each frame to make a smooth movement
        delta_x, delta_y, delta_z = update_pos(self.vx_, self.vy_, self.vz_, current_x, current_y, current_z, dt=self.dt_)

        # Matrix that we will use to actually move the robot
        T_delta = np.array([
            [1, 0, 0, delta_x],
            [0, 1, 0, delta_y],
            [0, 0, 1, delta_z],
            [0, 0, 0, 1]
        ])

        links = ['front_right', 'back_right', 'back_left', 'front_left']
        safe_to_publish = True      
        for i, link in enumerate(links):
            try:
                transform_stamped = self.tf_buffer_.lookup_transform(
                    target_frame='base_link',
                    source_frame= f'{link}_thigh',
                    time=rclpy.time.Time() # Use the newest available transform
                )

                thigh_x = transform_stamped.transform.translation.x
                thigh_y = transform_stamped.transform.translation.y
                thigh_z = transform_stamped.transform.translation.z

                
                thigh = T_delta @ np.array([thigh_x, thigh_y, thigh_z, 1.0]).reshape(4, -1)
                
                qx = transform_stamped.transform.rotation.x
                qy = transform_stamped.transform.rotation.y
                qz = transform_stamped.transform.rotation.z
                qw = transform_stamped.transform.rotation.w

                rot_thigh = R.from_quat([qx, qy, qz, qw])

                transform_stamped = self.tf_buffer_.lookup_transform(
                    target_frame='base_link',
                    source_frame= f'{link}_feet',
                    time=rclpy.time.Time() 
                )

                feet_x = transform_stamped.transform.translation.x
                feet_y = transform_stamped.transform.translation.y
                feet_z = transform_stamped.transform.translation.z


                thigh = thigh[:-1]
                feet = np.array([feet_x, feet_y, feet_z]).reshape(3, -1)
                
                hip_feet = feet - thigh
                hip_feet = rot_thigh.inv().apply(hip_feet.flatten())
                idx = i * 3
                command[idx], command[idx+1], command[idx+2] = self.solver_.solve(
                                                                    hip_feet[0], 
                                                                    hip_feet[1], 
                                                                    hip_feet[2])
                self.get_logger().info(f'thigh_feet: {hip_feet[0]}, {hip_feet[1]}, {hip_feet[2]}')

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                safe_to_publish = True
                self.get_logger().warn(f'TF Error: {e}')

        if safe_to_publish:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = command

            self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ChassisControl()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()