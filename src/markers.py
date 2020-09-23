#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import numpy as np 
import math

class Marker_print:
    def __init__(self):
        #rospy.init_node ("marker_print")
        self.pub_marker = rospy.Publisher("point_marker", Marker, queue_size=10)
        self.line_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.pub_marker_array = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    
    def create_marker_msg_sphere(self, x, y):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        return(marker)

    def create_marker_msg_line(self, x_arr, y_arr):
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        for i in np.arange(0, len(x_arr)):
            #create point and append
            point = Point()
            point.x = x_arr[i]
            point.y = y_arr[i]
            point.z = 0.0
            marker.points.append(point)
        return(marker)

    def print_marker_point_sphere(self, x, y):          
        marker = self.create_marker_msg_sphere(x,y)  
        self.pub_marker.publish(marker)
        rospy.sleep(0.2)
    
    def print_marker_point_line(self, x_arr, y_arr):          
        marker = self.create_marker_msg_line(x_arr,y_arr)
        rospy.sleep(0.5)
        self.line_marker.publish(marker)
    
    def print_marker_array(self, x_arr, y_arr, time):
        markerArray = MarkerArray()
        for i in np.arange(0, len(time)):
            marker=self.create_marker_msg_sphere(x_arr[i],y_arr[i])
            markerArray.markers.append(marker)
            # Renumber the marker IDs to remain
            id = 0
            for m in markerArray.markers:
                m.id = id
                id += 1
        rospy.sleep(0.3)
        # Publish the MarkerArray
        self.pub_marker_array.publish(markerArray)
    
    def test(self):
        rospy.init_node('line_pub_example')
        pub_line_min_dist = rospy.Publisher('~line_min_dist', Marker, queue_size=1)
        rospy.loginfo('Publishing example line')

        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD

            # marker scale
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            # marker color
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # marker orientaiton
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # marker position
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0

            # marker line points
            marker.points = []
            # first point
            first_line_point = Point()
            first_line_point.x = 0.0
            first_line_point.y = 0.0
            first_line_point.z = 0.0
            marker.points.append(first_line_point)
            # second point
            second_line_point = Point()
            second_line_point.x = 1.0
            second_line_point.y = 1.0
            second_line_point.z = 0.0
            marker.points.append(second_line_point)

            # Publish the Marker
            pub_line_min_dist.publish(marker)

            rospy.sleep(0.5)


        


if __name__ == "__main__":
    marker_class = Marker_print()
    # x = 2.0
    # y = 4.0
    x = np.array([1.0, 2.0, 3.0])
    y = np.array([1.0, 2.0, 3.0])
    try:
        marker_class.print_marker_point(x,y)
        #marker_class.test()

    except rospy.ROSInterruptException:
        pass
    