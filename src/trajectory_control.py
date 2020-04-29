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
    

    #methods
    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_control', anonymous=True) #make node
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 

        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)


    #current robot pose
    def odometryCb(msg):
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
        q_i = np.array([0,0, -np.pi/3]) #Initial posture (x_i,y_i,theta_i)
        q_f = np.array([4,3, np.pi])    #Final posture   (x_f,y_f,theta_f)
        k = 10

        self.t=np.linspace(0,15,1000)

        out = compute_pose_inputs(q_i, q_f, k, self.t)  #return dictionary of waypoint and desire linear/angular vel
        self.x_d = out['x']
        self.y_d = out['y']
        self.v_d = out['v']
        self.w_d = out['w']
        self.theta_d = out['theta']

    # #Run the simulation
    # def simulation_error(self):
    #     self.err = odeint(unicycle_error_model, self.q, self.t,args=(self.v_d, self.w_d, self.t))

    #postprocessing 
    def unicicle_publish_control_var(self):
        u = []
        x = []
        y = []
        theta = []
        q_err = [1,1,0]
        for i in np.arange(0, len(self.t)):
            #compute error model and integrate
            err = odeint(unicycle_error_model, self.q + q_err, self.t,args=(self.v_d, self.w_d, self.t))
            rospy.loginfo(err)
            #comput control inputs variable from error
            u_t = control(err, self.v_d[i], self.w_d[i]) #u1,u2
            v = self.v_d[i]*np.cos(err[2])-u_t[0]
            w = self.w_d[i]-u_t[1]
            theta_t = self.theta_d[i] - err[2]
            #move robot
            self.send_velocities(v, w, theta_t)

            rospy.sleep(15/1000)
        
        #stop after time
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_velocities(0,0)
            rate.sleep()


        # self.u.append([v, w])
        # self.theta_t = theta_d[i] - err[i,2]
        # self.x.append(x_d[i] - np.cos(theta_t)*err[i,0] + np.sin(theta_t)*err[i,1])
        # self.y.append(y_d[i] - np.sin(theta_t)*err[i,0] - np.cos(theta_t)*err[i,1])
        # self.theta.append(theta_t)

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
