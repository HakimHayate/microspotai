import copy
import matplotlib.pyplot as plt
import numpy as np

from spotmicro_mapping.jacobians import compute_gradient_analytical, compute_gradient_numerical
from spotmicro_mapping.se2 import wrap_angle

class PoseNode:
    def __init__(self, id, pose, pts): # Pose : x, y, theta
        self.id_ = id
        self.pose_ = np.array(pose, dtype=float)
        self.pts_ = pts
        
    def __repr__(self):
        return f"{self.id_}: ({self.pose_[0]:.2f}, {self.pose_[1]:.2f}, {self.pose_[2]:.2f})"


class Edge:
    def __init__(self, id_source, id_destination, measurement, is_loop_closure): # z : x, y, theta
        self.id_source_ = id_source
        self.id_destination_ = id_destination
        self.measurement_ = np.array(measurement, dtype=float)
        self.is_loop_closure = is_loop_closure


class Graph:
    def __init__(self):
        self.nodes_dict_ = {}
        self.edges_ = []
        self.free_id_ = 0

    def add_node(self, pose, pts):
        node = PoseNode(self.free_id_, pose, pts)
        self.nodes_dict_[node.id_] = node
        self.free_id_ += 1
        return node.id_

    def add_edge(self, id_source, id_destination, measurement, is_loop_closure=False):
        self.edges_.append(Edge(id_source, id_destination, measurement, is_loop_closure))

    def check_loop_closure(self, current_node_id, tol_translation=0.5, mid_nodes = 50):
        condidates = []
        current_node = self.nodes_dict_[current_node_id]
        for id in self.nodes_dict_:
            if abs(current_node_id - id) <= mid_nodes:
                continue
            node = self.nodes_dict_[id]
            dist = np.linalg.norm (node.pose_[:2] - current_node.pose_[:2])
            if dist < tol_translation:
                condidates.append(id)
        return condidates
    

    def print_graph(self):
        print("Poses")
        for node in self.nodes_dict_.values():
            print(node)

        print("\nEdges")
        for e in self.edges_:
            print(f"{e.id_source_} -> {e.id_destination_}   z = {e.measurement_}")

    def copy(self):
        new_graph = Graph()
        new_graph.nodes_dict_ = copy.deepcopy(self.nodes_dict_)
        new_graph.edges_ = copy.deepcopy(self.edges_)
        return new_graph
    
    def draw_graph(self):
        plt.clf()
        for node in self.nodes_dict_.values():
            plt.scatter(node.pose_[0], node.pose_[1], c='blue')
            plt.text(node.pose_[0], node.pose_[1], node.id_)
            plt.arrow(
                node.pose_[0],
                node.pose_[1],
                0.1*np.cos(node.pose_[2]),
                0.1*np.sin(node.pose_[2]),
                head_width=0.005
            )
        
        for e in self.edges_:
            p1 = self.nodes_dict_[e.id_source_]
            p2 = self.nodes_dict_[e.id_destination_]

            plt.plot([p1.pose_[0], p2.pose_[0]],
                     [p1.pose_[1], p2.pose_[1]], 
                     'k--')
            
        plt.axis('equal')
        plt.grid()
        plt.draw()
        plt.pause(0.01)


def main():
    example_pose0 = [0, 0, 0]
    example_pose1 = [0, 0.1, 0.5]
    example_pose2 = [0.7, 0.5, 1.1]
    example_pose3 = [2.0, 0.5, 0.4]

    edge_01 = Edge(0, 1, [0, 0.1, 0.5])
    edge_12 = Edge(1, 2, [0.7, 0.4, 0.6])
    edge_23 = Edge(2, 3, [1.3, 0.0, -0.7])
    edge_30 = Edge(3, 0, [-1.3, 0.0, 0.7])

    graph = Graph()

    graph.add_node(example_pose0)
    graph.add_node(example_pose1)
    graph.add_node(example_pose2)
    graph.add_node(example_pose3)

    graph.add_edge(edge_01)
    graph.add_edge(edge_12)
    graph.add_edge(edge_23)
    graph.add_edge(edge_30)

    #graph.print_graph()
    #graph.draw_graph()
    
    J_anal = compute_gradient_analytical(graph)
    J_num = compute_gradient_numerical(graph)

    print(J_anal)
    print('\n')
    print(J_num)
if __name__ == '__main__':
    main()