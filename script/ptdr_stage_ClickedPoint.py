#! /usr/bin/env python

import rospy
import tf
import math
#import numpy
#import cv2
#from math import *

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PointStamped

from tf.transformations import quaternion_about_axis
#from tf.TransformBroadcaster import quaternion_about_axis

class ClickedPointPub:

    def __init__ (self):
        #Publishers
        self.new_point = PointStamped()
        self.new_point.header.frame_id = "world"

        new_point_string = str("/clicked_point")
        print("new_point_string = " + str(new_point_string) )
        self.new_point_pub = rospy.Publisher(new_point_string, PointStamped, queue_size=1)

    def publish_new_point(self, seq, x, y):
        self.new_point.header.seq = seq
        self.new_point.header.stamp = rospy.Time.now()
        print("rospy.Time.now() = " + str(rospy.Time.now()))
        self.new_point.point.x = x
        self.new_point.point.y = y
        print("self.new_point = " + str(self.new_point))
        self.new_point_pub.publish(self.new_point)

    def shutdown_function(self):
        self.new_point.header.stamp = rospy.Time.now()
        self.new_point.point.x = -1.0
        self.new_point.point.y = -4.5
        self.new_point_pub.publish(self.new_point)
    
if __name__ == '__main__':
    rospy.init_node('clicked_point_node')
    #rate = rospy.Rate(10)
    #start_time = rospy.get_rostime()

    clicked_point = ClickedPointPub()
    rospy.on_shutdown(clicked_point.shutdown_function)
    pause = 0.5

    clicked_point.publish_new_point(0, 0, 0)
    rospy.sleep(pause)
    clicked_point.publish_new_point(1, -5, 5)
    rospy.sleep(pause)
    clicked_point.publish_new_point(2, -5, -5)
    rospy.sleep(pause)
    clicked_point.publish_new_point(3, 5, -5)
    rospy.sleep(pause)
    clicked_point.publish_new_point(4, 5, 5)

    while not rospy.is_shutdown():
        
        print("-------------- ^C to start ------------")
        rospy.sleep(5)


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
