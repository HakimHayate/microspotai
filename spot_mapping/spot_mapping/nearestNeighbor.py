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
    
    def compute_stats(self, distances):
        if distances is None:
            return
        distances.sort()
        distances = np.array(distances)
        print(f'distance min {np.min(distances)}')
        print(f'distance max {np.max(distances)}')
        print(f'distance mean {np.mean(distances)}')
        print(f'distance median {np.median(distances)}')
        print()

    def search(self, points, dist_tol= 0.02):
        matches_tree = []
        matches_points = []
        distances = []
        for p in points:
            best_point, dist = self.search_tree(p)
            #distances.append(dist)
            if dist > dist_tol:
                continue
            matches_tree.append(best_point)
            matches_points.append(p)
        #self.compute_stats(distances)
        return np.array(matches_tree).reshape(-1,2), np.array(matches_points).reshape(-1, 2)

# A test with brute force to see if I get same results (TEST OK!)
def brute_search_batch(src_points, target_points, dist_tol=0.1):
    matches_brute = []
    matches_points = []
    
    for target in target_points:
        best = None
        best_dist = np.inf
        for p in src_points:
            d = euclidian_distance(p, target)
            if d < best_dist:
                best = p
                best_dist = d
        
        matches_brute.append(best)
        matches_points.append(target)
            
    return matches_brute, matches_points

import time

def main():
    print("Generating point clouds...")
    
    src_points = np.random.random((10000, 2)) * 10
    
    target_points = np.random.random((500, 2)) * 10 
    

    print("\nBuilding KD-Tree...")
    tree = Tree(src_points)
    start_time = time.time()
    tree.build_tree()
    print(f"Tree built in: {time.time() - start_time:.4f} seconds")


    print("\nRunning KD-Tree Search...")
    start_time = time.time()
    tree_matches_src, tree_matches_tgt = tree.search(target_points, dist_tol=tol)
    tree_time = time.time() - start_time
    print(f"KD-Tree found {len(tree_matches_src)} valid matches in {tree_time:.4f} seconds")


    print("\nRunning Brute Force Search...")
    start_time = time.time()
    brute_matches_src, brute_matches_tgt = brute_search_batch(src_points, target_points, dist_tol=tol)
    brute_time = time.time() - start_time
    print(f"Brute Force found {len(brute_matches_src)} valid matches in {brute_time:.4f} seconds")


    print("\n--- RESULTS ---")
    
    if len(tree_matches_src) != len(brute_matches_src):
        print(" MISMATCH: Algorithms found different numbers of matches!")
        return


    mismatches = 0
    for i in range(len(tree_matches_src)):
        if not np.array_equal(tree_matches_src[i], brute_matches_src[i]):
            mismatches += 1

    if mismatches == 0:
        print("SUCCESS: KD-Tree perfectly matches Brute Force!")
        print(f"Speedup Factor: KD-Tree is {brute_time / tree_time:.1f}x faster than Brute Force.")
    else:
        print(f"FAILED: Found {mismatches} mismatched points between the algorithms.")

if __name__ == '__main__':
    main()