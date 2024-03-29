#!/usr/bin/env python3
import numpy as np

#Trajectory generation
class Trajectory_generation():
    def cubic_trajectory(self, q_i, q_f, k, t=None):
        ''' cubic interpolation from initial to final pose
        Parameters:
            q_i: array of initial pose
            q_f: array of finale pose
            k: intial and final geometric velocity
            t: array of times  

        Returns:
            x: array of desired x
            y: array of desired y
            v: array of derired linear velocity
            w: array of desired angular velocity
            theta: array of desired angular pose                  
        '''
        # intial pose coordinates
        x_i = q_i[0]
        y_i = q_i[1]
        # finale pose coordinates
        x_f = q_f[0]
        y_f = q_f[1]
       
        if t is not None:
             # normalize time array between [0,1]
            s = t/t[-1]
            tau = 1/t[-1]
        else:
            s = np.linspace(0,1,200)
            tau = 1
            
        b_x = k*np.cos(q_i[2]) + 3*q_i[0]
        b_y = k*np.sin(q_i[2]) + 3*q_i[1]

        a_x = k*np.cos(q_f[2]) - 3*q_f[0]
        a_y = k*np.sin(q_f[2]) - 3*q_f[1]

        #Cartesian cubic path interpolating intial-final poses
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
        
        return [x, y, v, w, theta]

    def cyrcular_trajectory(self, t):
        R = 3 # circle radius
        v_d_val = 0.5 # [m/s], const linear vel
        w_d_val = v_d_val/R # [rad/s], const angular vel
        # desired trajectory, starting from origin (t=0)
        x_d = R * np.cos(w_d_val * t) - R 
        y_d = R * np.sin(w_d_val * t)
        dotx_d = -R*w_d_val*np.sin(w_d_val* t) # time derivative of x_d
        doty_d =  R*w_d_val*np.cos(w_d_val* t) # time derivative of y_d
        theta_d = np.arctan2(doty_d, dotx_d)
        # desired velocity
        v_d = np.sqrt(dotx_d**2 + doty_d**2)       
        w_d = w_d_val * np.ones(len(t)) # array of same const w/ length=array of time
        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d]

    def eight_trajectory(self, t):
        # coordinates of center
        x_c = 0
        y_c = 0
        R = 3 #radius
        w_d = 1./15  # Desired Angular speed
        v_d = w_d * R
        # desired trajectory
        x_d = x_c + R*np.sin(2*w_d*t) 
        y_d = x_c + R*np.sin(w_d*t) 
        
        dotx_d = 2*R*w_d*np.cos(2*w_d*t)
        doty_d = R*w_d*np.cos(w_d*t)
        return [x_d,y_d,dotx_d, doty_d, v_d*t, w_d*t]
