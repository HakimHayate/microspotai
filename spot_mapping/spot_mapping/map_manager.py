import numpy as np
from utils import raycaster
import math
import matplotlib.pyplot as plt

class OccupancyGridMap:
    def __init__(self, width=10000, height=10000, resolution=100): # width & height in mm
        self.width_cells_ = int(width/resolution)
        self.height_cells_ = int(height/resolution)
        self.resolution_ = resolution

        self.map_ = np.zeros((self.width_cells_, self.height_cells_), dtype=np.float16)
        self.max_ = 10 

        self.p_occ_hit_ = 0.85 # Probability a cell is occupied given the laser hit it
        self.p_occ_free_ = 0.30 # Probability a cell is occupied given the laser didnt hit it
        self.p_prior_ = 0.5 # Prior belief 

        # Pre compute logs to make program runs faster
        self.l_occ_hit_ = math.log(self.p_occ_hit_/(1 - self.p_occ_hit_))
        self.l_occ_free_ = math.log(self.p_occ_free_/(1 - self.p_occ_free_)) 
        self.l_prior = math.log(self.p_prior_/(1 - self.p_prior_))
        self.sign_plot_ = -1


    def world_to_grid(self, x, y):
        return int(x / self.resolution_), int(y / self.resolution_)


    def update(self, pose_x, pose_y, pose_theta, scan):
        for s in scan:
            hit_angle, hit_dist = s[1], s[2]
            theta = math.radians(hit_angle) + pose_theta
            grid_x_hit = int((math.cos(theta) * hit_dist + pose_x)/self.resolution_)
            grid_y_hit =  int((self.sign_plot_ * math.sin(theta) * hit_dist + pose_y)/self.resolution_)
            grid_x, grid_y = int(pose_x/self.resolution_), int(pose_y/self.resolution_)

            path = raycaster([grid_x, grid_y], [grid_x_hit, grid_y_hit]) 
            
            for cel in path[:-1]:
                x, y = cel
                if 0 <= x < self.width_cells_ and 0 <= y < self.height_cells_:
                    self.map_[cel[0]][cel[1]] += self.l_occ_free_ - self.l_prior

            cel = path[-1]
            x, y = cel
            if 0 <= x < self.width_cells_ and 0 <= y < self.height_cells_:
                self.map_[cel[0]][cel[1]] += self.l_occ_hit_ - self.l_prior
        
        # Limit the values in map
        self.map_ = np.clip(self.map_, -self.max_, self.max_)
        print("map min/max:", self.map_.min(), self.map_.max())
    

    def plot_map(self):
        prob_map = 1 / (1 + np.exp(-self.map_))

        plt.figure()
        plt.imshow(prob_map.T, origin='lower', cmap='gray', vmin=0, vmax=1)
        plt.colorbar(label='Occupancy probability')
        plt.title("Occupancy Grid Map")
        plt.show()