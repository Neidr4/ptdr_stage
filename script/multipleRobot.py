#! /usr/bin/env python

import rospy
import tf
import math
#import numpy
#import cv2
#from math import *

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist

from tf.transformations import quaternion_about_axis
#from tf.TransformBroadcaster import quaternion_about_axis

robot0_ns = "robot_0"
robot1_ns = "robot_1"
robot2_ns = "robot_2"

class Robot:

    def __init__ (self, robot_ns_param):
        robot_ns = robot_ns_param
        print("robot_ns = " + str(robot_ns) )
        '''
        node_string = str(robot_ns + "_node")
        rospy.init_node(node_string)

        print("node_string = " + str(node_string) )
        '''
        #Subscribers
        '''
        self.sub_chatter = rospy.Subscriber('/chatter', String, self.callback_chatter)
        self.sub_odomA = rospy.Subscriber('/odomA', Int64, self.callback_odomA)
        self.sub_odomB = rospy.Subscriber('/odomB', Int64, self.callback_odomB)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel)
        '''
        #self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Int32, self.callback_cmd_vel)

        #Publishers
        cmd_vel_string = str("/" + robot_ns + "/cmd_vel")
        print("cmd_vel_string = " + str(cmd_vel_string) )
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_string, Twist, queue_size=1)

        self.command_twist = Twist()

    def routine(self):
        self.publish_cmd_vel()
        rospy.sleep(3)
        self.publish_cmd_vel_stop()
        rospy.sleep(3)

    def publish_cmd_vel_stop(self):
        self.command_twist.linear.x = 0.0
        self.command_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.command_twist)

    def publish_cmd_vel(self):
        self.command_twist.linear.x = 1.0
        self.command_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.command_twist)

    def shutdown_function(self):
        #self.command_int32.data = 0
        self.command_twist.linear.x = 0.0
        self.command_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.command_twist)
    
if __name__ == '__main__':
    rospy.init_node('many_robots_node')
    #rate = rospy.Rate(10)
    #start_time = rospy.get_rostime()

    robot0 = Robot(robot0_ns)
    robot1 = Robot(robot1_ns)
    robot2 = Robot(robot2_ns)
    rospy.on_shutdown(robot0.shutdown_function)
    rospy.on_shutdown(robot1.shutdown_function)
    rospy.on_shutdown(robot2.shutdown_function)
    
    while not rospy.is_shutdown():
        robot0.publish_cmd_vel()
        robot1.publish_cmd_vel()
        robot2.publish_cmd_vel()
        rospy.sleep(3)
        robot0.publish_cmd_vel_stop()
        robot1.publish_cmd_vel_stop()
        robot2.publish_cmd_vel_stop()
        rospy.sleep(3)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
