import numpy as np
import math

def v2t(v):
    theta = v[2]
    T = np.array([
        [math.cos(theta), -math.sin(theta), v[0]],
        [math.sin(theta), math.cos(theta), v[1]],
        [0, 0, 1]
    ])

    return T

def t2v(T):
    theta = math.atan2(T[1, 0], T[0, 0])
    v = np.array([T[0, -1], T[1, -1], theta])
    return v

def compose(T1, T2):
    return T1 @ T2


def inv(T):
    T_inv = np.eye(3)
    T_inv[:2, :2] = T[:2, :2].T
    T_inv[:2, -1] = -T[:2, :2].T @ T[:2, -1]

    return T_inv

def relative(xA, xB):
    return t2v(compose(inv(v2t(xA)), v2t(xB))) 

def wrap_angle(angle):
    return (angle + np.pî) % (2*np.pi) - np.pi