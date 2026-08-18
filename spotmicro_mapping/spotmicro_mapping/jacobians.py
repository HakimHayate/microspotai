import numpy as np
from spotmicro_mapping.residuals import residual, cost_function

def jacobians(graph, edge):
    node_source = graph.nodes_dict_[edge.id_source_]
    node_dst = graph.nodes_dict_[edge.id_destination_]

    xi, yi, thetai = node_source.pose_
    xj, yj, thetaj = node_dst.pose_

    Ji = np.array([
        [-np.cos(thetai), np.sin(thetai), 0],
        [-np.sin(thetai), -np.cos(thetai), 0],
        [-np.sin(thetai) * (xj - xi) + np.cos(thetai) * (yj - yi), -np.cos(thetai)*(xj-xi)-np.sin(thetai)*(yj-yi), -1]               
    ])

    Jj = np.array([
        [np.cos(thetai), -np.sin(thetai), 0],
        [np.sin(thetai), np.cos(thetai), 0],
        [0, 0, 1]               
    ])

    return Ji, Jj

def compute_gradient_analytical(graph):
    gradient_J = np.zeros((len(graph.nodes_dict_)*3))

    for edge in graph.edges_:
        Ji, Jj = jacobians(graph, edge)
        node_source = graph.nodes_dict_[edge.id_source_]
        node_dst = graph.nodes_dict_[edge.id_destination_]

        e = residual(node_source.pose_, node_dst.pose_, edge.measurement_)

        idx = edge.id_source_ * 3
        gradient_J[idx:idx+3] += Ji @ e

        idx = edge.id_destination_ * 3
        gradient_J[idx:idx+3] += Jj @ e

    return gradient_J



def compute_gradient_numerical(graph, eps=1e-4):
    gradient_J = np.zeros((len(graph.nodes_dict_)*3))
    cost = cost_function(graph)

    for key in graph.nodes_dict_:
        for i in range(3):
            graph_cpy = graph.copy()
            graph_cpy.nodes_dict_[key].pose_[i] += eps
            cost_eps = cost_function(graph_cpy)
            gradient_J[graph_cpy.nodes_dict_[key].id_*3+i] += (cost_eps - cost) / (eps)
            
    return gradient_J
