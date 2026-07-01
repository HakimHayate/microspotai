import numpy as np
import math
from utils import get_rotation


class Stabilizer:
    def __init__(self, T_base_thigh):
        self.T_base_thigh_ = T_base_thigh

    
    def compute_error(self, sensor_roll, sensor_pitch, T_world_base_ref, thigh_foot, links):
        R = T_world_base_ref[:3, :3]
        yaw_ref = math.atan2(R[1,0], R[0,0])
        error = {}
        T_world_base_sensor = np.zeros((4,4))
        T_world_base_sensor[:3, :3] = get_rotation(sensor_roll, sensor_pitch, yaw_ref)
        T_world_base_sensor[:,-1] = T_world_base_ref[:, -1]

        for link in links:
            thigh_foot_h = np.ones(4)
            thigh_foot_h[:3] = thigh_foot[link]

            foot_world_sensor = T_world_base_sensor @ self.T_base_thigh_[link] @ thigh_foot_h
            foot_world_ref = T_world_base_ref @ self.T_base_thigh_[link] @ thigh_foot_h
            error_foot_world = foot_world_ref[:3] - foot_world_sensor[:3]
            R_world_thigh = (T_world_base_sensor @ self.T_base_thigh_[link])[:3, :3]
            error[link] = R_world_thigh.T @ error_foot_world

        return error