import numpy as np

from spotmicro_mapping.jacobians import compute_gradient_analytical, jacobians
from spotmicro_mapping.residuals import cost_function, residual
from spotmicro_mapping.se2 import wrap_angle

def magic_optimizer_v1(graph, max_iteration=50, alpha = 0.001, tol=1e-2):
    prev_cost = cost_function(graph)
    error = np.inf

    for _ in range(max_iteration):
        gradient = compute_gradient_analytical(graph)

        for id in graph.nodes_dict_:
            graph.nodes_dict_[id].pose_ -= alpha * gradient[id*3:id*3+3]
            graph.nodes_dict_[id].pose_[2] = wrap_angle(graph.nodes_dict_[id].pose_[2])

        current_cost = cost_function(graph)

        error = abs(current_cost - prev_cost)
        print(error)
        if error < tol:
            break
        
        prev_cost = current_cost
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
import numpy as np

def compute_linear_system(graph):
    n = len(graph.nodes_dict_)

    
    H = lil_matrix((3*n, 3*n)) 
    b = np.zeros(3*n)

    for edge in graph.edges_:
        Ji, Jj = jacobians(graph, edge)   # already transposed
        node_source = graph.nodes_dict_[edge.id_source_]
        node_dst = graph.nodes_dict_[edge.id_destination_]

        e = residual(node_source.pose_, node_dst.pose_, edge.measurement_)

        i = edge.id_source_ * 3
        j = edge.id_destination_ * 3

        H[i:i+3, i:i+3] += Ji @ Ji.T
        H[i:i+3, j:j+3] += Ji @ Jj.T
        H[j:j+3, i:i+3] += Jj @ Ji.T
        H[j:j+3, j:j+3] += Jj @ Jj.T

        b[i:i+3] += Ji @ e
        b[j:j+3] += Jj @ e

    return H, b

def magic_optimizer(graph, max_iteration=20, tol=1e-6):
    n = len(graph.nodes_dict_)

    for _ in range(max_iteration):
        H, b = compute_linear_system(graph)
        
        H[0:3, 0:3] += np.eye(3) * 1e8 
        b[0:3] = 0


        dx = spsolve(H.tocsr(), -b)

        if np.linalg.norm(dx) < tol:
            break

        for node_id in graph.nodes_dict_:
            idx = node_id * 3
            graph.nodes_dict_[node_id].pose_ += dx[idx:idx+3]
            graph.nodes_dict_[node_id].pose_[2] = wrap_angle(
                graph.nodes_dict_[node_id].pose_[2]
            )