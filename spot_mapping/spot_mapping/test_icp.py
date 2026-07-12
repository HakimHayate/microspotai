import numpy as np
import copy
import time

# ==========================================
# 1. SE(2) Helper Functions
# ==========================================
def v2t(pose):
    x, y, theta = pose
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

def project(T, pts):
    ones = np.ones((pts.shape[0], 1))
    pts_h = np.hstack([pts, ones]) 
    return (T @ pts_h.T).T[:, :2]  

# ==========================================
# 2. Your Version (The Challenger)
# ==========================================
def icp_line2point_challenger(src, dst, normals, current_pose, alpha=0.01, tol=0.001, max_iterations=100):
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
        error = np.mean(np.linalg.norm(dst_tmp - src,axis=1))
        if error < tol:
            break

    print(f'student : {current_pose}')
    return current_pose

# ==========================================
# 3. Super Saiyan Version (The Professor)
# ==========================================
def icp_line2point_ssj(src, dst, normals, current_pose, alpha=0.01, tol=0.001, max_iterations=100):
    dst_tmp = dst.copy()
    error = np.inf
    
    # PRE-COMPUTE CONSTANTS: Extract these outside the loop!
    nx = normals[:, 0]
    ny = normals[:, 1]
    x = dst[:, 0]
    y = dst[:, 1]
    
    for _ in range(max_iterations):
        tx, ty, theta = current_pose
        
        # Scalar error projection
        tmp1 = np.sum(normals * (dst_tmp - src), axis=1) 
        
        # Calculate rotation gradient values as 1D arrays
        c, s = np.cos(theta), np.sin(theta)
        dtheta_term = nx * (-s * x - c * y) + ny * (c * x - s * y)
        
        # NO TMP ARRAYS! Direct dot product links straight to C-level BLAS
        J = np.array([
            np.dot(tmp1, nx),
            np.dot(tmp1, ny),
            np.dot(tmp1, dtheta_term)
        ])
        
        current_pose -= alpha * J
        T = v2t(current_pose)
        dst_tmp = project(T, dst)
        
        error = np.mean(np.linalg.norm(dst_tmp - src, axis=1))
        if error < tol:
            break
    print(f'prof : {current_pose}')
    return current_pose

# ==========================================
# 4. The World Martial Arts Tournament
# ==========================================
if __name__ == "__main__":
    # Create a massive point cloud to stress test memory allocation
    N = 500_000 
    print(f"Generating {N} points for the showdown...")
    
    x_vals = np.linspace(0, 100, N)
    y_vals = np.sin(x_vals)
    src_points = np.column_stack((x_vals, y_vals))
    
    tangent_x, tangent_y = np.ones(N), np.cos(x_vals)
    normals_raw = np.column_stack((-tangent_y, tangent_x))
    normals_cloud = normals_raw / np.linalg.norm(normals_raw, axis=1)[:, np.newaxis]
    
    # Offset target
    T_offset = v2t([0.05, -0.02, 0.01])
    dst_points = project(T_offset, src_points)
    
    pose_1 = np.array([0.0, 0.0, 0.0])
    pose_2 = np.array([0.0, 0.0, 0.0])
    
    print("\n--- FIGHT! ---")
    
    start = time.perf_counter()
    res_chal = icp_line2point_challenger(src_points, dst_points, normals_cloud, pose_1, alpha=0.0001)
    time_chal = time.perf_counter() - start
    print(f"Your Vectorized Form  : {time_chal:.5f} seconds")
    
    start = time.perf_counter()
    res_ssj = icp_line2point_ssj(src_points, dst_points, normals_cloud, pose_2, alpha=0.0001)
    time_ssj = time.perf_counter() - start
    print(f"Super Saiyan (No Tmp): {time_ssj:.5f} seconds")
    
    print(f"\nWinner Speedup: {time_chal / time_ssj:.2f}x Faster")