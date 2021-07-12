#! /usr/bin/env python

import rospy
import tf
import math
#import numpy
#import cv2
#from math import *

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, PoseStamped

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
        self.sub_new_goal = rospy.Subscriber('/new_goal', Twist, self.callback_new_goal)
        '''
        #self.sub_new_goal = rospy.Subscriber('/new_goal', Int32, self.callback_new_goal)

        #Publishers
        self.new_goal = PoseStamped()
        self.new_goal.header.frame_id = "world"

        new_goal_string = str("/" + robot_ns + "/move_base_simple/goal")
        print("new_goal_string = " + str(new_goal_string) )
        self.new_goal_pub = rospy.Publisher(new_goal_string, PoseStamped, queue_size=1)

    def routine(self):
        self.publish_new_goal()
        rospy.sleep(3)
        self.publish_new_goal_stop()
        rospy.sleep(3)

    def publish_new_goal_stop(self):
        self.new_goal.header.seq = 1
        self.new_goal.header.stamp = rospy.Time.now()
        self.new_goal.pose.position.y = -4.5
        self.new_goal.pose.position.x = -4.5
        self.new_goal.pose.orientation.w = 1

        self.new_goal_pub.publish(self.new_goal)

    def publish_new_goal(self):
        self.new_goal.header.seq = 1
        self.new_goal.header.stamp = rospy.Time.now()
        print("rospy.Time.now() = " + str(rospy.Time.now()))
        self.new_goal.pose.position.x = -1.0
        self.new_goal.pose.position.y = -4.5
        self.new_goal.pose.orientation.w = 1
        print("self.new_goal = " + str(self.new_goal))
        self.new_goal_pub.publish(self.new_goal)

    def shutdown_function(self):
        #self.command_int32.data = 0
        self.new_goal.header.seq = 1
        self.new_goal.header.stamp = rospy.Time.now()
        self.new_goal.pose.position.x = -4.5
        self.new_goal.pose.position.y = -4.5
        self.new_goal.pose.orientation.w = 1
        self.new_goal_pub.publish(self.new_goal)
    
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

        robot2.publish_new_goal()
        rospy.sleep(5)
        '''
        robot2.publish_new_goal_stop()
        rospy.sleep(5)
        '''

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
