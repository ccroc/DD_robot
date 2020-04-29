#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion

import numpy as np
from cubicpath import compute_pose_inputs, plot_wmr
from unicycle import unicycle_error_model, control
from scipy.integrate import odeint




class Trajectory_control():
    #attributes
    t = []
    x_d = []
    y_d = []
    v_d = []
    w_d = []
    theta_d = []
    q=[0, 0, 0]
    q_i=[]
    q_f=[]

    

    #methods
    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_control', anonymous=True) #make node
        #self.twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.twist_pub = rospy.Publisher('/DD_controller/cmd_vel', Twist, queue_size=10) 

        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)


    #current robot pose
    def odometryCb(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.get_angle_pose(msg.pose.pose)
        self.q = np.array([x, y, theta])
        #rospy.loginfo(self.q)
        return self.q

    #compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta


    #Trajectory generation
    def trajectory_generation_cubic(self):
        self.q_i = np.array([0.,0.,0.]) #Initial posture (x_i,y_i,theta_i)
        self.q_f = np.array([4.,4., 0.])    #Final posture   (x_f,y_f,theta_f)
        k = 0.2

        self.t=np.linspace(0,30,1000)
        
        out = compute_pose_inputs(self.q_i, self.q_f, k, t=self.t)  #return dictionary of waypoint and desire linear/angular vel
        self.x_d = out['x']
        self.y_d = out['y']
        self.v_d = out['v']
        self.w_d = out['w']
        self.theta_d = out['theta']

    def trajectory_generation_cyrcle(self):
        self.t=np.linspace(0,10,1000)

        R = 3
        v_d_val = 0.5 # m/s
        w_d_val = v_d_val/R
        self.x_d = R * np.cos(w_d_val * self.t) - R
        self.y_d = R * np.sin(w_d_val * self.t)
        dotx_d = -R*w_d_val*np.sin(w_d_val*self.t)
        doty_d =  R*w_d_val*np.cos(w_d_val*self.t)
        self.v_d = np.sqrt(dotx_d**2 + doty_d**2)
        self.theta_d = np.arctan2(doty_d, dotx_d)
        self.w_d = w_d_val * np.ones(len(self.t))
 



    def get_error(self, T):
        #slide 80 LDC
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        #rospy.loginfo("x={} y={} th={}".format(x,y,theta))
        #compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = self.theta_d[T] - theta
        err = np.array([e1, e2, e3])
        return err

    
    #postprocessing 
    def unicicle_publish_control_var(self):
        #comput control inputs variable from error
        for i in np.arange(0, len(self.t)):
            
            err = self.get_error(i)

            u_t = control(err, self.v_d[i], self.w_d[i])
            v = self.v_d[i]*np.cos(err[2])-u_t[0]
            w = self.w_d[i]-u_t[1]
            theta_t = self.theta_d[i] - err[2]

            rospy.loginfo(err)

            #open loop
            #v = self.v_d[i]
            #w=self.w_d[i]
            
            

            #move robot
            self.send_velocities(v, w, theta_t)
            rospy.sleep(10./1000)
        
        #stop after time
        self.send_velocities(0,0,0)


    #publish v, w
    def send_velocities(self, v, w, theta):
        twist_msg = Twist() # Creating a new message to send to the robot
        twist_msg.linear.x = v * np.cos(theta)
        twist_msg.linear.y = v * np.sin(theta)
        twist_msg.angular.z = w
        self.twist_pub.publish(twist_msg)



if __name__ == "__main__":
    try:
        tc=Trajectory_control()
        tc.trajectory_generation_cyrcle()
        tc.unicicle_publish_control_var()
        
    except rospy.ROSInterruptException:
        pass
