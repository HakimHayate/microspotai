import math
import numpy as np
import time

# x axis forward, y axis down
def icp_version1(scan1, scan2, size = 200):
    size = min(size, min(len(scan1), len(scan2)))

    pts_matching = np.zeros((size, 2))

    # Find matching points 
    for i, pt1 in enumerate(scan1):
        if i >= size:
            break
        dist_min = np.inf
        for pt2 in scan2:
            dist = (pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2
            if dist < dist_min:
                pts_matching[i] = pt2
                dist_min = dist

    # Compute center mean
    scan_1_center_x = scan1[:size,0].mean()
    scan_1_center_y = scan1[:size,1].mean() 

    scan_2_center_x = pts_matching[:,0].mean() 
    scan_2_center_y = pts_matching[:,1].mean() 

    # Center scans
    scan1_centered = np.zeros((size, 2))
    scan2_centered = np.zeros((size, 2))

    scan1_centered[:size, 0] = scan1[:size, 0] - scan_1_center_x
    scan1_centered[:size, 1] = scan1[:size, 1] - scan_1_center_y

    scan2_centered[:,0] = pts_matching[:, 0] - scan_2_center_x
    scan2_centered[:,1] = pts_matching[:, 1] - scan_2_center_y

    # Compute orientation (theta)
    Sxx = (scan1_centered[:size, 0] * scan2_centered[:, 0]).sum()
    Sxy = (scan1_centered[:size, 0] * scan2_centered[:, 1]).sum()
    Syx = (scan1_centered[:size, 1] * scan2_centered[:, 0]).sum()
    Syy = (scan1_centered[:size, 1] * scan2_centered[:, 1]).sum()

    theta = math.atan2(Syx - Sxy, Sxx + Syy)

    # Compute translation (tx, ty)
    tx = scan_2_center_x - (scan_1_center_x * math.cos(theta) + scan_1_center_y * math.sin(theta))
    ty = scan_2_center_y - (-scan_1_center_x * math.sin(theta) + scan_1_center_y * math.cos(theta))


    return tx, ty, theta

def polar_to_cartezian(scan):
    scan = np.array(scan)
    angles = np.array(scan[:,1])
    distances = np.array(scan[:,2])

    x = distances * np.cos(np.radians(angles))
    y = distances * np.sin(np.radians(angles))
    
    return np.stack((x, y), axis=1) # returns array N x 2


def match_version1(pts_source, pts_destiny, size= 300, thresh = 10):

    size = min(size, min(len(pts_destiny), len(pts_source)))
    pts_source_matching = np.zeros((size, 2))
    pts_destiny_matching = np.zeros((size, 2))

    # Find matching points 
    for i, pt1 in enumerate(pts_source):
        if i >= size:
            break
        dist_min = np.inf
        for pt2 in pts_destiny:
            dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
            if dist < dist_min and dist < thresh:
                pts_source_matching[i] = pt1
                pts_destiny_matching[i] = pt2

    
    return pts_source_matching, pts_destiny_matching # returns array N x 2

def match_version2(tree, new_points):
    matching_points_source = []
    matching_points_destiny = []
    distances = []
    
    for destiny_pt in new_points:
        source_pt, distance = tree.search_tree(destiny_pt)
        matching_points_source.append(source_pt)
        matching_points_destiny.append(destiny_pt)
        distances.append(distance)
    
    return np.array(matching_points_source).reshape(-1, 2), np.array(matching_points_destiny).reshape(-1, 2), distances # returns array N x 2
        

def icp_version2(pts_source, pts_destiny): 
    pts_source_mean = pts_source.mean(axis=0).reshape(-1, 1)
    pts_destiny_mean = pts_destiny.mean(axis=0).reshape(-1, 1)

    w = min(pts_source.shape[0], pts_destiny.shape[0])

    W = np.zeros((2, 2))
    for i in range(w):
        W += (pts_source[i, :].reshape(-1, 1) - pts_source_mean) @ (pts_destiny[i, :].reshape(-1, 1) - pts_destiny_mean).T

    U, D, Vt = np.linalg.svd(W)

    S = np.array([
        [1, 0],
        [0,  np.linalg.det(U) * np.linalg.det(Vt)] # Make sure it minimizes the error while keeping det(C) = 1
    ]) 

    C = Vt.T @ S @ U.T

    r = pts_destiny_mean - C @ pts_source_mean
    T = np.array([
            [C[0,0], C[0,1], r[0, 0]],
            [C[1,0], C[1,1], r[1, 0]],
            [0, 0, 1]
        ])
    return T, r # C.T : rotation from destiny frame to source frame, r : translation from destiny frame to source in source frame

def full_icp(pts_source, pts_destiny, tol=0.00001, max_iteration=50):
    tree = Tree(pts_source)
    tree.build_tree()

    prev_error = 0 
    T_base_current = np.eye(3)
    for _ in range(max_iteration):
        matched_pts_source, matched_pts_destiny = match_version1(pts_source, pts_destiny) # N x 2 
        C, r = icp_version2(matched_pts_source, matched_pts_destiny)
        
        T_delta = np.array([
            [C[0,0], C[0,1], r[0, 0]],
            [C[1,0], C[1,1], r[1, 0]],
            [0, 0, 1]
        ])
        T_base_current = T_base_current @ T_delta
        
        transformed = np.zeros(pts_destiny.shape) # N x 2
        
        for i in range(len(transformed)):
            transformed[i, :] = (C @ pts_destiny[i].reshape(-1, 1) + r).T

        error = np.mean(np.linalg.norm(
            matched_pts_source - matched_pts_destiny,
            axis=1
        ))
        if np.abs(error - prev_error) < tol:
            break
        
        pts_destiny = transformed
        prev_error = error
    
    return T_base_current, C
        
    

# A quick test for lidar and the icp
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
from nearestNeighbor import Tree

min_scans = 100
def main():
    global_map = []
    PORT_NAME = "COM6"
    T_world_lidar = np.eye(3)
    lidar = RPLidar(PORT_NAME, timeout=3, baudrate=256000)
    plt.ion()
    fig, ax = plt.subplots()
    initialized = False
    tree = None
    prev_pts = None
    all_pts= []
    i = 0
    for scan in lidar.iter_scans():
        all_pts.append(polar_to_cartezian(scan))


        current_pts = np.vstack(all_pts)
        all_pts = []
        i = 0
        if not initialized:
            tree = Tree(current_pts)
            tree.build_tree() 
            prev_pts = current_pts
            global_map.append(polar_to_cartezian(scan))
            initialized = True
            continue

        pts_current_h = np.hstack((current_pts, np.ones((current_pts.shape[0], 1))))
        map = np.array(np.vstack(global_map)).reshape(-1, 2)
        T_prev_current, C = full_icp(map, current_pts)
        T_world_lidar = T_world_lidar @ T_prev_current
        transformed = (T_world_lidar @ pts_current_h.T).T
        transformed = transformed[:, :-1]
        theta = math.atan2(T_world_lidar[1,0], T_world_lidar[0,0])
        print(f'theta = {math.degrees(theta)}')

        global_map.append(transformed)
        
        if transformed is None:
            continue
        ax.cla()
        
        ax.set_aspect('equal')
        ax.set_xlim(-5000, 5000)
        ax.set_ylim(-500, 5000)
        plt.scatter(map[:,0], map[:,1], c='green', s=1, label='map')

        # plt.scatter(current_pts[:,0], current_pts[:,1], c='red', s=1, label='current')
        # plt.scatter(prev_pts[:,0], prev_pts[:,1], c='black', s=1, label='prev')

        plt.scatter([1000.0], [0.0], c='r', s=1.5)
        plt.scatter([0.0], [1000.0], c='b', s=1.5)
        plt.scatter([0.0], [0.0], c='black', s=1)
        plt.legend()
        plt.draw()
        plt.pause(0.1)
        time.sleep(5)

        prev_pts = current_pts


if __name__ == '__main__':
    main()
