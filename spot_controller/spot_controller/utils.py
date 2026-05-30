import numpy as np
from scipy.linalg import logm, expm

def get_twist(T_current, T_desired, t=1):
    
    S = logm(np.linalg.inv(T_current) @ T_desired) / t
    
    return S

def update_pos(T_current, S, dt=1):
    T_current_updated = T_current @ expm(S*dt)
 
    return T_current_updated


