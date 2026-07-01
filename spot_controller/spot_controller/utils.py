import numpy as np
from scipy.linalg import logm, expm
import math

def get_twist(T_current, T_desired, t=1):
    
    S = logm(np.linalg.inv(T_current) @ T_desired) / t
    
    return S

def update_pos(T_current, S, dt=1):
    T_current_updated = T_current @ expm(S*dt)
 
    return T_current_updated

def get_rotation(roll, pitch, yaw):
    Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)],
    ])

    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx # Convention: Roll-Pitch-Yaw

def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # clamp at ±90°
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class Node:
    def __init__(self, parent, configuration):
        self.parent = parent
        self.configuration = configuration

def isCollision(q, model, collision_model, data, collision_data):
    
    is_colliding = pin.computeCollisions(model, data, collision_model, collision_data, q, True)
    
    return is_colliding

def remove_manual_collision_pair(collision_model, name1, name2):
    ids_1 = [i for i, geom in enumerate(collision_model.geometryObjects) if name1 in geom.name]
    ids_2 = [i for i, geom in enumerate(collision_model.geometryObjects) if name2 in geom.name]
    
    for i1 in ids_1:
        for i2 in ids_2:
            pair = pin.CollisionPair(i1, i2)
            if collision_model.existCollisionPair(pair):
                collision_model.removeCollisionPair(pair)


def rpy(roll, pitch, yaw):
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def interpolate(q_start, q_end, k):
    s = np.linspace(0, 1, k)
    return [q_start*(1-ss) + q_end*ss for ss in s]

def get_safe_path(q_start, q_end, model,
                  collision_model, 
                  data, collision_data, min, max, 
                  nb_random_q = 10, steps=100, max_iter=100):
    
    path = interpolate(q_start, q_end, steps)
    for q in path:
        if isCollision(q, model, collision_model, data, collision_data):
            continue
    
    return path
    

def computeJacobian(arm_controller, seed, min, max, tol, max_iter=10000):
    q_target = seed.copy()

    for _ in range(max_iter):
        J_anal = np.zeros((6, arm_controller.model.nv))
        pin.forwardKinematics(arm_controller.model, arm_controller.data, q_target)
        pin.updateFramePlacements(arm_controller.model, arm_controller.data)
        T_world_gripper = arm_controller.data.oMf[arm_controller.gripper_id_].homogeneous
        end_effector_h = np.ones(4)
        end_effector_h[:3] = arm_controller.end_effector
        gripper_pos = (T_world_gripper @ end_effector_h)[:-1]
        for i in range(1, arm_controller.model.njoints):
            joint = arm_controller.model.joints[i]
            if joint.nv == 0:
                continue

            idx = joint.idx_v
            joint_pos = arm_controller.data.oMi[i].translation
            v_q = np.zeros(arm_controller.model.nv)
            v_q[idx] = 1.0
            
            pin.forwardKinematics(arm_controller.model, arm_controller.data, q_target, v_q)
            
            local_axis = arm_controller.data.v[i].angular
            world_axis = arm_controller.data.oMi[i].rotation @ local_axis
            
            # Compute the Jacobian columns
            J_anal[:3, idx] = np.cross(world_axis, gripper_pos - joint_pos)
            J_anal[3:, idx] = world_axis


        I = np.eye(J_anal.shape[0])
        J_T = J_anal.T
        lambda_sq = 0.01
        J_damped_inv = J_T @ np.linalg.inv(J_anal @ J_T + lambda_sq * I)
        e = np.zeros(6)
        e[:3] = arm_controller.x_desired_[:3] - gripper_pos

        R_current = T_world_gripper[:3, :3]
        R_desired_= rpy(arm_controller.x_desired_[3], arm_controller.x_desired_[4], arm_controller.x_desired_[5])

        orientation_error = pin.log3(R_desired_ @ R_current.T)
        e[3:] = orientation_error
        
        delta_q = arm_controller.alpha * J_damped_inv @ e
        q_target += delta_q
        #q_target = np.clip(q_target, min, max)
        error = np.linalg.norm(e)

        if error < tol:
            return q_target
        print(error)
    
    return None # IK failed

def get_random_pose(q, min, max):
    return np.random.rand() * max + min