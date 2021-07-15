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

class movebaseHandler:

    def __init__ (self, robot_ns_param, init_x, init_y):
        self.robot_ns = robot_ns_param
        print("robot_ns = " + str(self.robot_ns) )
        self.init_x = init_x
        self.init_y = init_y

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

        new_goal_string = str("/" + self.robot_ns + "/move_base_simple/goal")
        print("new_goal_string = " + str(new_goal_string) )
        self.new_goal_pub = rospy.Publisher(new_goal_string, PoseStamped, queue_size=1)

        #Subscribers
        self.last_received_pose = PoseStamped()
        buffer_string = str("/" + self.robot_ns + "/move_base_simple/buffer")
        self.sub_movebaseHandler = rospy.Subscriber(buffer_string, PoseStamped, self.callback_movebaseHandler)

    def callback_movebaseHandler(self, msg):
        self.last_received_pose = msg
        #print("self.last_received_pose.pose.position.x: " + str(self.last_received_pose.pose.position.x) )
        #print("self.last_received_pose.pose.position.y: " + str(self.last_received_pose.pose.position.y) )

    def routine(self):
        self.publish_new_goal()
        rospy.sleep(3)
        self.publish_new_goal_stop()
        rospy.sleep(3)

    def publish_new_goal_param(self, x, y):
        self.new_goal.header.seq = 1
        self.new_goal.header.stamp = rospy.Time.now()
        self.new_goal.pose.position.x = x
        self.new_goal.pose.position.y = y
        self.new_goal.pose.orientation.w = 1
        self.new_goal_pub.publish(self.new_goal)

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
        self.new_goal.pose.position.x = -1.0
        self.new_goal.pose.position.y = -4.5
        self.new_goal.pose.orientation.w = 1

        if( (self.last_received_pose.pose.position.x == 0.0) and (self.last_received_pose.pose.position.y == 0.0)):
            rospy.loginfo('No goal published yet for ' + str(self.robot_ns))
        else:
            self.new_goal.pose.position.x = self.last_received_pose.pose.position.x + self.init_x
            self.new_goal.pose.position.y = self.last_received_pose.pose.position.y + self.init_y
            rospy.loginfo('Goal for '  + str(self.robot_ns) + ' has been published to ' + str(self.last_received_pose.pose.position.x) + ', ' + str(self.last_received_pose.pose.position.y) + ' with offset')
            self.new_goal_pub.publish(self.new_goal)

        self.new_goal_pub.publish(self.new_goal)
        rospy.sleep(2)
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

    movebaseHandler0 = movebaseHandler(robot0_ns, 4.5, 4.5)
    movebaseHandler1 = movebaseHandler(robot1_ns, 4.5, -4.5)
    movebaseHandler2 = movebaseHandler(robot2_ns, -4.5, -4.5)

    rospy.on_shutdown(movebaseHandler0.shutdown_function)
    rospy.on_shutdown(movebaseHandler1.shutdown_function)
    rospy.on_shutdown(movebaseHandler2.shutdown_function)

    rospy.loginfo("initilisation")
    movebaseHandler2.publish_new_goal_param( (movebaseHandler2.init_x + 1.1) , (movebaseHandler2.init_y) )
    rospy.sleep(5)

    pause = 15
    
    while not rospy.is_shutdown():
        rospy.loginfo("loop")
        rospy.loginfo("instruction 1")
        movebaseHandler2.publish_new_goal_param(-1.0, -4.5)
        rospy.sleep(pause)
        rospy.loginfo("instruction 2")
        movebaseHandler2.publish_new_goal_param(-4.5, -4.5)
        rospy.sleep(pause)
        rospy.loginfo("instruction 3")
        print("movebaseHandler2.last_received_pose.pose.position.x: " +str(movebaseHandler2.last_received_pose.pose.position.y))
        print("movebaseHandler2.last_received_pose.pose.position.y: " +str(movebaseHandler2.last_received_pose.pose.position.x))
        movebaseHandler2.publish_new_goal_param(movebaseHandler2.last_received_pose.pose.position.y, movebaseHandler2.last_received_pose.pose.position.x)
        rospy.sleep(pause)
        '''
        movebaseHandler2.publish_new_goal()
        rospy.sleep(10)
        movebaseHandler2.publish_new_goal_stop()
        rospy.sleep(5)
        '''

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
