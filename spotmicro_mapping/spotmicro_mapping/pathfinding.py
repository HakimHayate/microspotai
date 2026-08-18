import numpy as np

class Node:
    def __init__(self, pose, parent, f, accumulated_g, id):
        self.pose = np.array(pose)
        self.parent = parent
        self.accumulated_g = accumulated_g
        self.f = f
        self.id = id

class PathFinding:
    def __init__(self, map):
        self.map = map
        self.directions = [[1, 1], [1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [1, -1], [-1, 1]]
        self.free_id = 0

    def get_id(self):
        self.free_id += 1
        return self.free_id-1

    def get_path(self, current_pose_map, dst_pose_map, max_iterations=1000):
        arrived_node = None
        paths = {}
        visited_pose = set()
        node = Node(current_pose_map, None, 0, 0, self.get_id())
        paths[node.id] = node
        for _ in range(max_iterations):
            # Find the best path so far
            best_f = np.inf
            parent = None
            for n in paths.values():
                if n.f < best_f:
                    parent = n
                    best_f = n.f
            visited_pose.add(tuple(parent.pose))

            # explore all directions
            for dir in self.directions:
                tmp_pose = parent.pose + dir
                if tmp_pose[0] < 0 or tmp_pose[0]>=self.map.width_cells_ or tmp_pose[1] < 0 or tmp_pose[1]>=self.map.height_cells_ or tuple(tmp_pose) in visited_pose:
                    continue
                
                accumulated_g = parent.accumulated_g + self.map.map_[tuple(tmp_pose)]
                f = accumulated_g + np.linalg.norm(tmp_pose - dst_pose_map)
                new_node = Node(tmp_pose, parent, f, accumulated_g, self.get_id()) 
                paths[new_node.id] = new_node

                if np.array_equal(tmp_pose, dst_pose_map):
                    arrived_node = new_node
             
            paths.pop(parent.id)

            if arrived_node is not None:
                best_path = []
                node = arrived_node
                best_path.append(node.pose)
                while node.parent is not None:
                    node = node.parent
                    best_path.append(node.pose)
                return best_path[::-1]
            
        print('Failed to find a path')
        return None

