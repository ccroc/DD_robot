import numpy as np 

def cartesian_regulation_control_law(x, y, theta):
    #gains
    k1 = 0.5
    k2 = 0.5
    
    #control inputs
    v = -k1 * (x * np.cos(theta) + y * np.sin(theta))
    w = k2 * (np.arctan2(y, x) + np.pi - theta)

    return np.array([v, w])