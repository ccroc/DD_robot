import numpy as np
import matplotlib.pyplot as plt

'''
Given the initial and final desired states (q_i, q_f) and
the parameter k (inital/final desired speed) computes 
thanks to flatness the desired trajectory and reference inputs.

The output is a dictionary with the following keys:
- x: x component of the desired trajectory
- y: y component of the desired trajectory
- theta: theta component of the desired trajectory
- v: reference linear velocity input
- w: reference angular velocity input
'''
def compute_cubic_trajectory(q_i,q_f,k,  t=None):
    #Compute derivative
    x_i = q_i[0]
    y_i = q_i[1]
    x_f = q_f[0]
    y_f = q_f[1]

    if t is not None:
        s = t/t[-1]
        tau = 1/t[-1]
    else:
        s = np.linspace(0,1,200)
        tau = 1
        
    b_x = k*np.cos(q_i[2]) + 3*q_i[0]
    b_y = k*np.sin(q_i[2]) + 3*q_i[1]

    a_x = k*np.cos(q_f[2]) - 3*q_f[0]
    a_y = k*np.sin(q_f[2]) - 3*q_f[1]

    #Cartesian cubic path 
    x = x_f*s**3 - x_i*(s-1)**3 + a_x * s**2 * (s-1) + b_x * s * (s-1)**2
    y = y_f*s**3 - y_i*(s-1)**3 + a_y * s**2 * (s-1) + b_y * s * (s-1)**2

    #Compute first derivative
    xp = 3*x_f*s**2 - 3*x_i*(s-1)**2 + a_x*(3*s**2 -2*s) + b_x*(s-1)*(3*s-1)
    yp = 3*y_f*s**2 - 3*y_i*(s-1)**2  +a_y*(3*s**2 -2*s) + b_y*(s-1)*(3*s-1)
    
    #We can compute the geometric velocity and the posture angle theta
    v = np.sqrt(xp**2 + yp**2)
    theta = np.arctan2(yp,xp)
    
    #Compute second derivative
    xpp = 6*x_f*s - 6*x_i*(s-1) + a_x*(6*s-2) + b_x*(6*s-4)
    ypp = 6*y_f*s - 6*y_i*(s-1) + a_y*(6*s-2) + b_y*(6*s-4)
    
    #Compute the angular velocity
    w = (ypp*xp - xpp*yp)/(v**2)
    
    out = {}
    out['x'] = x
    out['y'] = y
    out['theta'] = theta
    out['v'] = v*tau
    out['w'] = w*tau
   
    return out


def plot_wmr(pose,scale=1.,ax=None, color=None):
    #Triangle in (0,0)
    X = np.array([[0.,-1.], [0.,1.], [2., 0.]])*scale
        
    x0,y0,theta = pose
    R = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
    #rotate
    X = np.matmul(X,R)
    
    #Translate
    Xt = np.vstack((X.transpose(),[1,1,1]))
    #2D Translation matrix
    T = np.identity(3)
    T[0:2,2] = [x0,y0]
    
    #Translate
    Xtr = np.matmul(T,Xt)
    robot = Xtr[0:-1,:].transpose()
    
    if ax:
        if not color:
            color = 'black'
        
        robot_p = plt.Polygon(robot, color=color)
        ax.add_patch(robot_p)
        
    return robot

