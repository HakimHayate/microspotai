import numpy as np
from scipy.spatial import KDTree
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class Node:
    def __init__(self, parent, configuration):
        self.parent = parent
        self.configuration = configuration

def isCollision(q, model, collision_model, data, collision_data):
    
    is_colliding = pin.computeCollisions(model, data, collision_model, collision_data, q, True)
    
    return is_colliding

def remove_manual_collision_pair(collision_model, name1, name2):
    """
    Finds all geometries matching the strings and removes their collision pairs.
    """
    # Find all geometry IDs that contain our target link names
    ids_1 = [i for i, geom in enumerate(collision_model.geometryObjects) if name1 in geom.name]
    ids_2 = [i for i, geom in enumerate(collision_model.geometryObjects) if name2 in geom.name]
    
    # Remove every pairing between these two links
    for i1 in ids_1:
        for i2 in ids_2:
            pair = pin.CollisionPair(i1, i2)
            if collision_model.existCollisionPair(pair):
                collision_model.removeCollisionPair(pair)


def rpy(roll, pitch, yaw):
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def get_safe_path(q_start, q_end, model, collision_model, data, collision_data, thresh=0.05, step=0.38, max_iter=10000, k=50):
    nodes = [Node(None, q_start)]
    error = np.inf
    n = len(q_start)

    s = np.linspace(0, 1, k)
    path = [q_start*(1-ss) + q_end*ss for ss in s]
    for q in path:
        if isCollision(q, model, collision_model, data, collision_data):
            print('obstacle!')
            return
    return path

    '''
    for _ in range(max_iter):
        tree = KDTree([node.configuration for node in nodes], leafsize=30)

        if (np.random.rand() < 0.40):
            q_random = q_end
        else:
            q_random = np.random.rand(n) * np.pi

        _, idx = tree.query(q_random)

        q_nearest = nodes[idx].configuration
        diff = q_random - q_nearest

        norm = np.linalg.norm(diff)

        if norm < 1e-8:
            continue
        direction = (q_random - q_nearest)/np.sqrt(np.sum((q_random - q_nearest)**2))
        q_new = q_nearest + step * direction

        if isCollision(q_new, model, collision_model, data, collision_data):
            continue

        node = Node(nodes[idx],q_new)
        nodes.append(node)
    
        error = np.linalg.norm(q_new - q_end)

        if error < thresh:
            path = [q_end]

            while node is not None:
                path.append(node.configuration)
                node = node.parent

            return path[::-1]
    '''
    

