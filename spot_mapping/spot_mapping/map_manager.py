import numpy as np
from utils import raycaster
import math
import matplotlib.pyplot as plt
from se2 import *

class OccupancyGridMap:
    def __init__(self, width=50, height=50, resolution=0.01): # width & height in mm
        self.width_cells_ = int(width/resolution)
        self.height_cells_ = int(height/resolution)
        self.resolution_ = resolution

        self.map_ = np.zeros((self.width_cells_, self.height_cells_), dtype=np.float16)
        self.max_ = 10 

        self.origin_x_ = self.width_cells_ // 2
        self.origin_y_ = self.height_cells_ // 2

        self.p_occ_hit_ = 0.85 # Probability a cell is occupied given the laser hit it
        self.p_occ_free_ = 0.30 # Probability a cell is occupied given the laser didnt hit it
        self.p_prior_ = 0.5 # Prior belief 

        self.l_occ_hit_ = math.log(self.p_occ_hit_/(1 - self.p_occ_hit_))
        self.l_occ_free_ = math.log(self.p_occ_free_/(1 - self.p_occ_free_)) 
        self.l_prior = math.log(self.p_prior_/(1 - self.p_prior_))
        self.sign_plot_ = -1


    def world_to_grid(self, x, y):
        return int(x / self.resolution_) + self.origin_x_, int(y / self.resolution_) + self.origin_y_


    def update(self, pose, pts):
        T = v2t(pose)
        pts_h = toHomogeneous(pts)
        global_pts = compose(T, pts_h.T)[:, :-1]
        grid_x, grid_y = self.world_to_grid(pose[0], pose[1])

        free_cels = set()
        hit_cells = set()

        for p in global_pts:
            grid_x_hit, grid_y_hit = self.world_to_grid(p[0], p[1])

            if not (0 <= grid_x_hit < self.width_cells_ and 0 <= grid_y_hit < self.height_cells_):
                continue

            path = raycaster([grid_x, grid_y], [grid_x_hit, grid_y_hit]) 
            
            for cel in path[:-1]:
                free_cels.add(tuple(cel))
                

            if len(path) > 0:
                hit_cells.add(tuple(path[-1]))
            
        for cel in free_cels:
            if 0 <= cel[0] < self.width_cells_ and 0 <= cel[1] < self.height_cells_:
                    self.map_[cel[0], cel[1]] += self.l_occ_free_ 
        for cel in hit_cells:
            if 0 <= cel[0] < self.width_cells_ and 0 <= cel[1] < self.height_cells_:
                self.map_[cel[0], cel[1]] += self.l_occ_hit_
        
        self.map_ = np.clip(self.map_, -self.max_, self.max_)
        
    
    def rebuild(self, graph):
        self.map_ = np.zeros((self.width_cells_, self.height_cells_), dtype=np.float16)
        for node in graph.nodes_dict_.values():
            self.update(node.pose_, node.pts_)

    def plot_map(self):
        prob_map = 1 / (1 + np.exp(-self.map_))

        plt.figure()
        plt.imshow(prob_map.T, origin='lower', cmap='gray', vmin=0, vmax=1)
        plt.colorbar(label='Occupancy probability')
        plt.title("Occupancy Grid Map")
        plt.show()