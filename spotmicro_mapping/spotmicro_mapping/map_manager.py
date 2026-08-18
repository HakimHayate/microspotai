import numpy as np
from spotmicro_mapping.utils import raycaster
import math
import matplotlib.pyplot as plt
from spotmicro_mapping.se2 import *
from scipy.ndimage import map_coordinates


class OccupancyGridMap:
    def __init__(self, width=50, height=50, resolution=0.05): # width & height in m
        
        self.width_cells_ = int(width/resolution)
        self.height_cells_ = int(height/resolution)
        self.resolution_ = resolution
    
        self.map_ = np.zeros((self.width_cells_, self.height_cells_), dtype=np.float16)
        self.max_ = 10 

        radius = 3

        x = np.arange(-radius, radius + 1)
        y = np.arange(-radius, radius + 1)
        xx, yy = np.meshgrid(x, y)

        self.offsets = np.c_[xx.ravel(), yy.ravel()]

        distances = np.linalg.norm(self.offsets, axis=1)

        self.weights = np.maximum(0.0, 1.0 - (distances / (radius + 1.0)))

        self.origin_x_ = self.width_cells_ // 2
        self.origin_y_ = self.height_cells_ // 2
        self.origin_ = np.array([self.origin_x_, self.origin_y_])

        self.p_occ_hit_ = 0.85 # Probability a cell is occupied given the laser hit it
        self.p_occ_free_ = 0.15 # Probability a cell is occupied given the laser didnt hit it
        self.p_prior_ = 0.5 # Prior belief 

        self.l_occ_hit_ = math.log(self.p_occ_hit_/(1 - self.p_occ_hit_))
        self.l_occ_free_ = math.log(self.p_occ_free_/(1 - self.p_occ_free_)) 
        self.l_prior = math.log(self.p_prior_/(1 - self.p_prior_))
        self.sign_plot_ = -1


    def world_to_grid(self, x, y):
        return math.floor(x / self.resolution_ + self.origin_x_), math.floor(y / self.resolution_) + self.origin_y_

    def world_to_grid_vectorized(self, pts):
        return  np.floor(pts / self.resolution_ + self.origin_).astype(np.int32)

    def grid_to_world(self, pts_map):
        return (pts_map- self.origin_)*self.resolution_

    def update(self, pts_map, robot_pos_map):
        grid_x, grid_y = robot_pos_map

        free_cels = set()
        hit_cels = set()
        for p in pts_map:
            grid_x_hit, grid_y_hit = p
            path = raycaster([grid_x, grid_y], [grid_x_hit, grid_y_hit]) 
            
            for cel in path[:-1]:
                free_cels.add(tuple(cel))
                

            if len(path) > 0:
                hit_cels.add(tuple(path[-1]))

        free_cels -= hit_cels

        if len(free_cels) > 0 :
            free_cels = np.array(list(free_cels), dtype=int)
            valid_mask_free = (0 <= free_cels[:,0]) & (free_cels[:,0] < self.width_cells_) & (0 <= free_cels[:,1]) & (free_cels[:,1] < self.height_cells_)
            self.map_[free_cels[valid_mask_free][:,0], free_cels[valid_mask_free][:,1]] += self.l_occ_free_
            
        
        if len(hit_cels) > 0:
            hit_cels = np.array(list(hit_cels),dtype=int)
            valid_mask_hit = (0 <= hit_cels[:,0]) & (hit_cels[:,0] < self.width_cells_) & (0 <= hit_cels[:,1]) & (hit_cels[:,1] < self.height_cells_)
            self.map_[hit_cels[valid_mask_hit][:,0], hit_cels[valid_mask_hit][:,1]] += self.l_occ_hit_
        
        self.map_ = np.clip(self.map_, -self.max_, self.max_)

    def get_score(self, global_pts_map):
        shifted_pts = global_pts_map[:, None, :] + self.offsets
        
        x = shifted_pts[..., 0]
        y = shifted_pts[..., 1]
        
        valid_mask = (x >= 0) & (x < self.width_cells_) & \
                     (y >= 0) & (y < self.height_cells_)
        
        valid_x = x[valid_mask]
        valid_y = y[valid_mask]
        
        _, offset_indices = np.nonzero(valid_mask)
        valid_weights = self.weights[offset_indices]
        
        map_values = self.map_[valid_x, valid_y]
        
        occupied = map_values > 0
        
        return np.dot(map_values[occupied], valid_weights[occupied])

    def match(self, initial_pose, pts, x_range_pixels=2, y_range_pixels=2, theta_range=np.deg2rad(15), angular_resolution= np.deg2rad(1)):
        x_search = range(-x_range_pixels, x_range_pixels+1)
        y_search = range(-y_range_pixels, y_range_pixels+1)
        theta_search = np.arange(-theta_range, theta_range, angular_resolution)
        #pts = filter_lidar_ransac(pts)
        R = get_R(initial_pose[2])

        global_pts = pts @ R.T + initial_pose[:2]
        global_pts_map = self.world_to_grid_vectorized(global_pts)
        best_score = self.get_score(global_pts_map)
        best_pose = initial_pose.copy()
        best_pts_map = global_pts_map
        robot_pos_map = self.world_to_grid(initial_pose[0],initial_pose[1])

        for dtheta in theta_search: 
            theta = wrap_angle(initial_pose[2] + dtheta)
            R = get_R(theta)
            global_pts = pts @ R.T + initial_pose[:2]
            global_pts_map = self.world_to_grid_vectorized(global_pts)

            for dy in y_search: 
                for dx in x_search:
                    translated_pts_map = global_pts_map + np.array([dx, dy])
                    score = self.get_score(translated_pts_map)
                    if score > best_score:
                        best_score = score
                        best_pose = initial_pose + np.array([dx*self.resolution_, dy*self.resolution_, 0])
                        best_pose[2] = theta
                        best_pts_map = translated_pts_map
                        x,y = self.world_to_grid(initial_pose[0],initial_pose[1])
                        robot_pos_map = x+dx, y+dy

        return best_pose, best_score, best_pts_map, robot_pos_map

    def rebuild(self, graph):
        self.map_ = np.zeros((self.width_cells_, self.height_cells_), dtype=np.float16)
        for node in graph.nodes_dict_.values():
            robot_pos_map = self.world_to_grid(node.pose_[0], node.pose_[1])
            world_pts = project(v2t(node.pose_), node.pts_)
            pts_map = self.world_to_grid_vectorized(world_pts)
            self.update(pts_map, robot_pos_map)


    def get_path(self, current_pose_map, dst_pose_map, max_iterations):
        path = []
        dir = [1, -1]
        diag = [[1, 1], [-1, -1], [1, -1], [-1, 1]]
        pose = current_pose_map.copy()
        for _ in range(max_iterations):
            best_next_pos = None
            best_next_score = np.inf
            for i in dir:
                tmp_pose = pose.copy()
                tmp_pose[0] =+ i
                score = self.map_[tmp_pose] + np.linalg.norm(tmp_pose - dst_pose_map)
                if score < best_next_score:
                    best_next_pos = tmp_pose.copy()
                    best_next_score = score
            for i in dir:
                tmp_pose = pose.copy()
                tmp_pose[1] =+ i
                score = self.map_[tmp_pose] + np.linalg.norm(tmp_pose - dst_pose_map)
                if score < best_next_score:
                    best_next_pos = tmp_pose.copy()
                    best_next_score = score
            for i in diag:
                tmp_pose = pose.copy()
                tmp_pose =+ i
                score = self.map_[tmp_pose] + np.linalg.norm(tmp_pose - dst_pose_map)
                if score < best_next_score:
                    best_next_pos = tmp_pose.copy()
                    best_next_score = score
            path.append(best_next_pos)
            pose = best_next_pos.copy()

            if pose == dst_pose_map:
                return path
        print("No path found")
        return None

    def plot_map(self):
        prob_map = 1 / (1 + np.exp(-self.map_))

        plt.figure()
        plt.imshow(prob_map.T, origin='lower', cmap='gray', vmin=0, vmax=1)
        plt.colorbar(label='Occupancy probability')
        plt.title("Occupancy Grid Map")
        plt.show()