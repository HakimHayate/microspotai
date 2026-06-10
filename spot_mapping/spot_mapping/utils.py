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


def raycaster(p_start, p_end, max_iterations=100):
    '''
    Bresenham's line algorithm, takes 2 points in a grid and give the path that connects them.

    Parameters
        p_start : sequence[int]
            Starting grid coordinate (x, y).

        p_end : sequence[int]
            Ending grid coordinate (x, y).

    Returns
        list[tuple[int, int]]
            Ordered list of grid coordinates from p_start to p_end
    '''
    x_robot, y_robot, x_hit, y_hit = p_start[0], p_start[1], p_end[0], p_end[1]
    dx = abs(x_hit - x_robot)
    dy = abs(y_hit - y_robot)

    sx = -1 if x_hit - x_robot < 0 else 1
    sy = -1 if y_hit - y_robot < 0 else 1

    current_x, current_y = x_robot, y_robot

    path = [(current_x, current_y)]
    e = dx - dy

    for i in range(max_iterations):
        if (current_x, current_y) == (x_hit, y_hit):
            break
        tmp = e
        if tmp <= dx:
            current_y += sy
            e += 2 *dx
            
        if tmp >= -dy:
            current_x += sx
            e -= 2 * dy 

        path.append((current_x, current_y))
    return path

def main():
    p_start = (0, 0)
    p_end = (3, 4)
    path = raycaster(p_start, p_end)

    print(path)

if __name__ == '__main__':
    main()