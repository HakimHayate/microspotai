import numpy as np
from nearestNeighbor import Tree
from se2 import *
from scipy.spatial import cKDTree

def transform(src, dst):
    '''
    Args:
        src: N * m numpy array, N numbers of points, m number of dimensions
        dst: N * m numpy array, N numbers of points, m number of dimensions 
    
    Returns:
        T: Transformation matrix mapping dst to src
    '''

    assert src.shape == dst.shape

    src_centroid = src.mean(axis=0)
    dst_centroid = dst.mean(axis=0)

    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    W = src_centered.T @ dst_centered # Cross variance
    U, _, Vt = np.linalg.svd(W)

    S = np.eye(src.shape[1])
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1

    C = Vt.T @ S @ U.T
    r = src_centroid - C.T @ dst_centroid

    T_src_dest = np.eye(src.shape[1]+1)
    T_src_dest[:-1, :-1] = C.T
    T_src_dest[:-1, -1] = r   

    return T_src_dest


def toHomogeneous(points):
    pts_h = np.ones((points.shape[0], points.shape[1]+1))
    pts_h[:,:-1] = points
    return pts_h



def icp(src, dst, guess, max_interation=50, tol=1e-2):
    tree = cKDTree(src)

    error = np.inf
    
    T_src_dst = v2t(guess)
    dst = project(T_src_dst, dst)

    for _ in range(max_interation):
        distances, indices = tree.query(dst)
        valid_matches = distances <= tol

        if np.sum(valid_matches) < 3:
            return None
        
        src_match = src[indices[valid_matches]]
        dst_match = dst[valid_matches]

        if len(src_match) <= 0:
            print('ICP failed ')
            return None
        
        T = transform(src_match, dst_match)
        T_src_dst = T @ T_src_dst 

        dst = project(T, dst)

        error = np.mean(distances[valid_matches])

        if error < tol:
            break

    if error > tol:
        print('ICP failed ')
        return None
    
    return T_src_dst, error

def rotate(src, measurement):
    T = v2t(measurement)
    print(T.shape)
    src_h = np.ones((src.shape[0]+1, src.shape[1]))
    src_h[:-1, :] = src
    return (T @ src_h)[:-1]

def main():
    

    src = np.random.rand(10, 2)

    T_true = v2t(np.array([0.1, 0.2, 0.57]))

    src_h = toHomogeneous(src)
    dst = (np.linalg.inv(T_true) @ src_h.T).T[:, :2]   # because transform() claims dst -> src

    T_est = transform(src, dst)
    T_est_icp = icp(src, dst)
    

    print("True:")
    print(T_true)

    print("Estimated:")
    print(T_est)

    print("Estimated icp:")
    print(T_est_icp)

    print(t2v(T_est))

if __name__ == '__main__':
    main()