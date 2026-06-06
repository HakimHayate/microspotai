import numpy as np
from nearestNeighbor import Tree

def transform(src, dst):
    '''
    Computes the least-squares rigid transformation
    (rotation + translation) that maps src to dst.
    Reflections are corrected so that det(R)=1.

    Args:
        src: N * m numpy array, N numbers of points, m number of dimensions
        dst: N * m numpy array, N numbers of points, m number of dimensions 
    
    Returns:
        T: Transformation matrix mapping src to dst
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
    r = dst_centroid - C @ src_centroid

    T = np.eye(src.shape[1]+1)
    T[:-1, :-1] = C
    T[:-1, -1] = r

    return T


def toHomogeneous(points):
    pts_h = np.ones((points.shape[0], points.shape[1]+1))
    pts_h[:,:-1] = points
    return pts_h



def icp(src, dst, max_interation=50, tol=1e-3):
    tree = Tree(dst)
    tree.build_tree()
    error = np.inf
    
    T_dst_src = np.eye(src.shape[1]+1)

    for _ in range(max_interation):
        dst_match, src_match = tree.search(src)

        T = transform(src_match, dst_match)
        T_dst_src = T @ T_dst_src 

        src_h = toHomogeneous(src)
        src_h = (T @ src_h.T).T
        src = src_h[:, :-1]

        error = np.sqrt(((dst_match - src_match)**2).sum(axis=1)).mean(axis=0)

        if error < tol:
            break
    
    return T_dst_src
