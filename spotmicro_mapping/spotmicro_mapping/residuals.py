import numpy as np
import spotmicro_mapping.se2 as se2

def residual(xi, xj, zij):
    hij = se2.relative(xi, xj)
    hij[2] = se2.wrap_angle(hij[2])
    return hij - zij

def cost_function(graph):
    cost = 0
    for edge in graph.edges_:
        node_source = graph.nodes_dict_[edge.id_source_]
        node_dst = graph.nodes_dict_[edge.id_destination_]

        e = residual(node_source.pose_, node_dst.pose_, edge.measurement_)
        cost += 0.5 * e.T @ e
    
    return cost