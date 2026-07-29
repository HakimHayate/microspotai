import numpy as np
import copy
from se2 import t2v, v2t, project

def find_line(points, k=100, thresh=0.01, min_pts=10):
    R = np.array([
        [0, -1],
        [1, 0]
    ])

    normal = None
    best_line = []

    for _ in range(k):
        line = []
        i1 = np.random.randint(0, len(points))
        i2 = np.random.randint(0, len(points))
        
        d = points[i2] - points[i1]
        norm = np.linalg.norm(d)

        if norm == 0:
            continue
        n = R @ (d / norm)

        bool_distances = np.abs((points-points[i1]) @ n) < thresh
        line = points[bool_distances]

        if len(line) > len(best_line) and len(line) >= min_pts:
            best_line = line
            normal = n

    return best_line, normal

def extract_all_lines(points, thresh= 0.01, max_iteration=100):
    normals = []
    lines = []
    for _ in range(max_iteration):
        best_line, normal = find_line(points,thresh=thresh)
        if normal is None or len(best_line) == 0:
            continue
        normals.append(normal)
        lines.append(np.mean(best_line, axis=0))

        distances = np.abs((points - np.mean(best_line, axis=0)) @ normal.T)
        
        points = points[distances > thresh]
    
    return np.array(normals), np.array(lines)

def find_matches(src, dst, tol=0.1):
    normals, lines = extract_all_lines(src)
    if len(lines) == 0:
        return None
    distances = np.abs(np.sum((dst[:, None] - lines) * normals, axis=2))

    closest_line_indices = np.argmin(distances, axis=1)
    min_distances = np.min(distances, axis=1)
    
    valid_mask = min_distances<tol

    valid_line_indices = closest_line_indices[valid_mask]
    matched_lines = lines[valid_line_indices]
    matched_normals = normals[valid_line_indices]
     
    return matched_lines, valid_mask, matched_normals


from scipy.spatial import cKDTree
def filter_and_cluster_points(points, threshold=1):
        if len(points) == 0:
            return points
        tree = cKDTree(points)
        clusters = tree.query_ball_tree(tree, threshold)
        return np.array([np.mean(points[cluster], axis=0) for cluster in clusters])


def icp_point2line(src, dst, guess=np.array([0.0,0.0,0.0]), alpha=0.01, tol=0.01, max_iterations=100): 
    '''
    src, dst, normals: Nx2
    '''
    #src = filter_and_cluster_points(src)
    res = find_matches(src, project(v2t(guess), dst))
    if res is None:
        return guess
    src, dst_idx, normals = res
    if len(src) == 0:
        return guess
    dst = dst[dst_idx]
    dst_tmp = copy.deepcopy(dst)
    error = np.inf
    n = len(src)
    nx = normals[:, 0]
    ny = normals[:, 1]
    current_pose = guess.copy()
    for _ in range(max_iterations):
        tx, ty, theta = current_pose
        J = np.zeros(3)
        tmp = np.sum(normals * (dst_tmp - src), axis=1) 

        c, s = np.cos(theta), np.sin(theta)
        x, y = dst[:,0], dst[:, 1]
        dtheta = nx *(-s*x - c*y) + ny * (c*x - s*y) 
        J = np.array([
            tmp @ nx,
            tmp @ ny,
            tmp @ dtheta
        ])/n

        current_pose -= alpha * J 
        T = v2t(current_pose)
        dst_tmp = project(T, dst)
        error = np.mean(np.abs(tmp))
        if error < tol:
            break


    return current_pose