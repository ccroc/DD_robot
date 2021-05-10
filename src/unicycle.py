import numpy as np

'''
This function implements a simple constant gain controller for $k_1(v_d,\omega_d)$
''' 
zeta = 0.8
a = 5

def k1_circ(v_d, w_d):
    global zeta, a
    return 2*zeta*a
    #return 1.2

def k1(v_d, w_d):
    global zeta, a
    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)
    
'''
This function implements a simple constant gain controller for $k_3(v_d,\omega_d)$
''' 
def k3_circ(v_d, w_d):
    global zeta, a
    return 2*zeta*a
    #return 1.2

def k3(v_d, w_d):
    global zeta, a
    return 2*zeta*np.sqrt(v_d**2 + a*w_d**2)
    

'''
This function implements the control. k1 and k3 functions
are used to (possibly) implement time varying gains, whereas
the gain k2 is set in the function.
'''
def control(e, v_d, w_d):
    #k2 = 0.5;
    global a, zeta
    #k2 = (a**2 -w_d**2)/v_d
    k2 = a
    
    u_1 = -k1(v_d, w_d) * e[0]
    
    # Be sure that if e[2] = 0 sin(e[2])/e[2] is computes to 1.0
    if e[2] == 0: #e_3
        u_2 = -k2*v_d*e[1] - k3(v_d,w_d)*e[2]
    else:
        u_2 = -k2*v_d*np.sin(e[2])/e[2]*e[1] - k3(v_d,w_d)*e[2]
    
    return np.array([u_1,u_2])
    
'''
Error dynamics. This function is used odeint 
to simulate the closed-loop system.
    @ e: array of errors [e1, e2, e3]
    @ t: current time
    @ v_d: desired velocity at time t
    @ w_d: angular velocity at time t
    @ time: array of times
'''
def unicycle_error_model(e, t, v_d, w_d, time):
    T = np.searchsorted(time,t) #index of t
    if (T>=len(time)):
        T = len(time)-1
    # compute control input    
    u_1, u_2 = control(e, v_d[T], w_d[T])
    # differential equation of error
    edot_1 = u_1 + e[1]*(w_d[T] - u_2)
    edot_2 = v_d[T]*np.sin(e[2]) - e[0]*(w_d[T] - u_2)
    edot_3 = u_2
    return [edot_1,edot_2, edot_3]

def R(theta):
    return np.array([[np.cos(theta), np.sin(theta), 0], 
                     [-np.sin(theta), np.cos(theta),1],
                     [ 0, 0 ,1]])