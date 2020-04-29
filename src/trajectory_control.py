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
    q=[]
    q_i=[]
    q_f=[]

    

    #methods
    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_control', anonymous=True) #make node
        self.twist_pub = rospy.Publisher('/DD_controller/cmd_vel', Twist, queue_size=10) 

        #rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)


    #current robot pose
    def odometryCb(self,msg):
        x = msg.pose.pose.position[0]
        y = msg.pose.pose.position[1]
        theta = get_angle_pose(msg.pose.pose)
        self.q = np.array([x, y, theta])
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
    def trajectory_generation(self):
        self.q_i = np.array([0,0, -np.pi/3]) #Initial posture (x_i,y_i,theta_i)
        self.q_f = np.array([4,3, np.pi])    #Final posture   (x_f,y_f,theta_f)
        k = 10

        self.t=np.linspace(0,15,1000)
        
        out = compute_pose_inputs(self.q_i, self.q_f, k, self.t)  #return dictionary of waypoint and desire linear/angular vel
        self.x_d = out['x']
        self.y_d = out['y']
        self.v_d = out['v']
        self.w_d = out['w']
        self.theta_d = out['theta']

    # #Run the simulation
    def simulation_error(self): #compute error model and integrate
        q_err = [1,1,0]
        err = odeint(unicycle_error_model, self.q_i + q_err, self.t, args=(self.v_d, self.w_d, self.t))
        rospy.loginfo(err)
        return err

    #postprocessing 
    def unicicle_publish_control_var(self):
        u = []
        x = []
        y = []
        theta = []

        #run simulation
        err = self.simulation_error()

        #comput control inputs variable from error
        for i in np.arange(0, len(self.t)):
            rospy.loginfo(i)            
            u_t = control(err[i,:], self.v_d[i], self.w_d[i])
            v = self.v_d[i]*np.cos(err[i,2])-u_t[0]
            w = self.w_d[i]-u_t[1]
            theta_t = self.theta_d[i] - err[i,2]

            #move robot
            self.send_velocities(v, w, theta_t)
            rospy.sleep(15/1000)
        
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
        tc.trajectory_generation()
        tc.unicicle_publish_control_var()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
