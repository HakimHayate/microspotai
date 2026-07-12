import numpy as np
import copy
from se2 import v2t, project

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

def extract_all_lines(points, max_iteration=100):
    normals = []
    lines = []
    for _ in range(max_iteration):
        best_line, normal = find_line(points)
        if normal is None or len(best_line) == 0:
            continue
        normals.append(normal)
        lines.append(np.mean(best_line, axis=0))

        distances = np.linalg.norm(points[:, None] - best_line, axis=2)
        min_distances = np.min(distances, axis=1)
        
        points = points[min_distances > 1e-6]
    
    return normals, np.array(lines)

def find_matches(src, dst, tol=0.1):
    normals, lines = extract_all_lines(src)
    distances = np.abs(np.sum((dst[:, None] - lines) * normals, axis=2))

    closest_line_indices = np.argmin(distances, axis=1)
    min_distances = np.min(distances, axis=1)
    
    valid_mask = min_distances<tol
    matched_dst = dst[valid_mask]

    valid_line_indices = closest_line_indices[valid_mask]
    matched_lines = lines[valid_line_indices]
    matched_normals = normals[valid_line_indices]
     
    return matched_lines, matched_dst, matched_normals

def icp_line2point(src, dst, normals, current_pose, alpha=0.01, tol=0.001, max_iterations=100):
    '''
    src, dst, nornals: Nx2
    '''
    dst_tmp = copy.deepcopy(dst)
    error = np.inf

    N = src.shape[0]

    nx = normals[:, 0]
    ny = normals[:, 1]

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
        ])

        current_pose -= alpha * J 
        T = v2t(current_pose)
        dst_tmp = project(T, dst)
        error = np.mean(np.linalg.norm(dst_tmp - src),axis=1)
        if error < tol:
            break


    return current_pose