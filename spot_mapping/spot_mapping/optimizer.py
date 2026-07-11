import numpy as np
from jacobians import compute_gradient_analytical
from residuals import cost_function
from se2 import wrap_angle

def magic_optimizer(graph, max_iteration=100, alpha = 0.01, tol=1e-4):
    prev_cost = cost_function(graph)
    error = np.inf

    for _ in range(max_iteration):
        gradient = compute_gradient_analytical(graph)

        for id in graph.nodes_dict_:
            graph.nodes_dict_[id].pose_ -= alpha * gradient[id*3:id*3+3]
            graph.nodes_dict_[id].pose_[2] = wrap_angle(graph.nodes_dict_[id].pose_[2])

        current_cost = cost_function(graph)

        error = abs(current_cost - prev_cost)

        if error < tol:
            break
        
        prev_cost = current_cost

