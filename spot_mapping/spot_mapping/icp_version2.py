import numpy as np
import copy

def icp_line2point(src, dst, normals, current_pose, alpha=0.01, tolr=0.001):
    '''
    src, dst, nornals: 2*N
    '''
    J = np.zeros(3)
    while error > tol:
        
        for i in range(len(normals)):
            ni = normals[i]
            di = dst[i] - src[i]
            x, y, theta = current_pose
            tmp = np.array([
                ni[0],
                ni[1],
                ni[0] * (-np.sin(theta)*x - np.cos(theta) * y) + ni[1] * (np.cos(theta)*x - np.sin(theta) * y)
            ])
            J += (ni.T @ di) * tmp

        current_pose -= alpha * J 
        

    return current_pose