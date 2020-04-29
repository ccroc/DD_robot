import numpy as np

# This implements a simple constant gain controller for $k_1(v_d,\omega_d)$
def k1(v_d, w_d):
    return 0.9

def k3(v_d, w_d):
    return 0.9

def control(e, v_d, w_d):
    k2 = 1;
    u_1 = -k1(v_d, w_d) * e[0]
    if e[2] == 0:
        u_2 = -k2*v_d*e[1] - k3(v_d,w_d)*e[2]
    else:
        u_2 = -k2*v_d*np.sin(e[2])/e[2]*e[1] - k3(v_d,w_d)*e[2]
    
    return np.array([u_1,u_2])
    
def unicycle_error_model(e, t, v_d, w_d, time):
    T = np.searchsorted(time,t)
    if (T>=len(time)):
        print('Warning! Time t',t)
        T = len(time)-1
   # T=t
        
    u_1, u_2 = control(e, v_d[T], w_d[T])
    edot_1 = u_1 + e[1]*(w_d[T] - u_2)
    edot_2 = v_d[T]*np.sin(e[2]) - e[0]*(w_d[T] - u_2)
    edot_3 = u_2
    ret =  [edot_1,edot_2, edot_3]
    return ret

    