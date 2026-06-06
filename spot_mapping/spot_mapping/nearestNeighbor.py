import numpy as np
import random
import math

# Personal implementation of KDTree algorithm to find matching points

class Node:
    def __init__(self, point, axis, left=None, right=None):
        self.point = point
        self.axis = axis
        self.left = left
        self.right = right

def qsort(points, axis):
    n = len(points)
    if n <= 1:
        return points # Base case
    
    # Picking a random index to escape worst case scenario where the list is already sorted
    pivot_index = random.randint(0, n-1)
    pivot = points[pivot_index]

    left = [x for x in points if x[axis] < pivot[axis]]
    middle = [x for x in points if x[axis] == pivot[axis]]
    right = [x for x in points if x[axis] > pivot[axis]]

    return qsort(left, axis) + middle + qsort(right, axis)

def euclidian_distance(x, y):
    return math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2)

class Tree:
    def __init__(self, points):
        self.root = None
        self.points = points

    def build_tree(self, points=None, depth=0):
        if depth == 0:
            points = self.points

        if points is None or len(points) == 0:
            return None
        points = qsort(points, axis=depth%2)

        n = len(points)
        med = points[n // 2]
        
        node = Node(med, depth%2, self.build_tree(points[:n//2], depth + 1), self.build_tree(points[n//2+1:], depth + 1))

        if depth == 0:
            self.root = node
        
        return node


    def search_tree(self, target, depth=0, next_branch = None, best_point=None, best_dist=float('inf')):
        if depth == 0 and not next_branch:
            branch = self.root
        else:
            branch = next_branch

        if branch is None:
            return best_point, best_dist

        dist = euclidian_distance(branch.point, target)

        if best_dist > dist:
            best_dist = dist
            best_point = branch.point

        next_branch = None
        opposite_branch = None

        # Greedy search
        if branch.point[branch.axis] <= target[branch.axis]:
            next_branch = branch.right
            opposite_branch = branch.left
            best_point, best_dist = self.search_tree(target, depth+1, next_branch=next_branch, best_point=best_point, best_dist=best_dist)

        else:

            next_branch = branch.left
            opposite_branch = branch.right
            best_point, best_dist = self.search_tree(target, depth+1, next_branch=next_branch, best_point=best_point, best_dist=best_dist)


        # ghost check
        ghost_check_dist = abs(branch.point[branch.axis] - target[branch.axis])
        if ghost_check_dist < best_dist:
            best_point, best_dist = self.search_tree(target, depth+1, next_branch=opposite_branch, best_point=best_point, best_dist=best_dist)


        return best_point, best_dist
    
    def search(self, points):
        matches_tree = []
        matches_points = []
        for p in points:
            best_point , _ = self.search_tree(p)
            matches_tree.append(best_point)
            matches_points.append(p)
        return np.array(matches_tree).reshape(-1,2), np.array(matches_points).reshape(-1, 2)

# A test with brute force to see if I get same results (TEST OK!)
def brute_search(points, target):
    best = None
    best_dist = np.inf
    for p in points:
        d = euclidian_distance(p, target)
        if d < best_dist:
            best = p
            best_dist = d
    
    return np.array(best)

from scipy.spatial import KDTree
from sklearn.neighbors import NearestNeighbors

def main():
    points = np.random.random((100000, 2)) *5 + 50 
    neigh = NearestNeighbors(n_neighbors=1)
    
    tree = Tree(points)
    tree.build_tree()
    kd = KDTree(points)
    for _ in range(100):
        target = np.random.random(2) * 5 + 50
        
        p_tree, _ = tree.search_tree(target)
        p_brute = brute_search(points, target)
        dist, idx = kd.query(target)
        if p_tree[0] != points[idx][0] or p_tree[1] != points[idx][1]:
            print('mismatch')
            break

if __name__ == '__main__':
    main()