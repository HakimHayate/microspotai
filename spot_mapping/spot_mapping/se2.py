import numpy as np
import math

def v2t(v):
    theta = v[2]
    T = np.array([
        [np.cos(theta), -np.sin(theta), v[0]],
        [np.sin(theta), np.cos(theta), v[1]],
        [0, 0, 1]
    ])

    return T

def t2v(T):
    theta = np.atan2(T[1, 0], T[0, 0])
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
    return (angle + np.pi) % (2*np.pi) - np.pi

def toHomogeneous(pts):
    pts_h = np.ones((pts.shape[0], pts.shape[1]+1))
    pts_h[:, :-1] = pts
    return pts_h

def project(T, pts):
    R = T[:2, :2]
    t = T[:2, 2]
    return pts @ R.T + t