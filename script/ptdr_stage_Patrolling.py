#! /usr/bin/env python

import rospy
import tf
import math
import numpy

from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, PoseStamped, PointStamped, PoseWithCovarianceStamped

robot0_ns = "robot_0"
robot1_ns = "robot_1"
robot2_ns = "robot_2"

array_point_patrol = [ (-4.5, 4.5), (-4.5, -4.5), (4.5, -4.5), (-3.5, -1.5), (4.0, 1.0), (-2.0, 3.0)]


class Patrolling:

    def __init__ (self, robot_ns_param, init_x, init_y):
        self.robot_ns = robot_ns_param
        print("robot_ns = " + str(self.robot_ns) )
        self.init_x = init_x
        self.init_y = init_y

        self.last_run = 0 #15 + init_x + init_y
        self.previous = 0
        self.current_objetive = (init_x, init_y)

        #Publishers
        self.new_goal = PoseStamped()
        self.new_goal.header.frame_id = "world"
        new_goal_string = str("/" + self.robot_ns + "/move_base_simple/goal")
        self.new_goal_pub = rospy.Publisher(new_goal_string, PoseStamped, queue_size=1)

        self.new_point = PointStamped()
        self.new_point.header.frame_id = "world"
        new_point_string = str("/" + self.robot_ns + "/clicked_point")
        self.new_point_pub = rospy.Publisher(new_point_string, PointStamped, queue_size=1)

        #Subscribers
        self.amcl_pose = PoseWithCovarianceStamped()
        self.amcl_pose.header.frame_id = "world"
        amcl_pose_string = str("/" + self.robot_ns + "/amcl_pose")
        self.amcl_pose_sub = rospy.Subscriber( amcl_pose_string, PoseWithCovarianceStamped, self.callback_amcl_pose)


    def callback_amcl_pose(self, msg):
        #print("inside callback_amcl_pose")
        self.amcl_pose = msg


    def publish_new_point(self, seq, x, y):
        self.new_point.header.seq = seq
        self.new_point.header.stamp = rospy.Time.now()
        #print("rospy.Time.now() = " + str(rospy.Time.now()))
        self.new_point.point.x = x
        self.new_point.point.y = y
        #print("self.new_point = " + str(self.new_point))
        self.new_point_pub.publish(self.new_point)


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
        self.new_goal.pose.position.y = self.amcl_pose.pose.pose.position.x
        self.new_goal.pose.position.x = self.amcl_pose.pose.pose.position.y
        self.new_goal.pose.orientation = self.amcl_pose.pose.pose.orientation

        self.new_goal_pub.publish(self.new_goal)


    def check_proximity(self):
        print("inside check_proximity")
        tolerance = 0.25

        abs_x = abs(self.amcl_pose.pose.pose.position.x)
        abs_y = abs(self.amcl_pose.pose.pose.position.y)

        objective_x = abs(self.new_goal.pose.position.x)
        objective_y = abs(self.new_goal.pose.position.y)

        print("check_proximity x and y = " + str(abs_x) + ", " + str(abs_y))
        print("x - tolerance: " + str(abs_x - tolerance))
        print((objective_x - tolerance <= abs_x))
        print("x + tolerance: " + str(abs_x + tolerance))
        print((objective_x + tolerance >= abs_x))
        print("y - tolerance: " + str(abs_y - tolerance))
        print((objective_y - tolerance <= abs_y))
        print("y + tolerance: " + str(abs_y + tolerance))
        print((objective_y + tolerance >= abs_y))

        if( (objective_x - tolerance <= abs_x) and 
            (objective_x + tolerance >= abs_x) and 
            (objective_y - tolerance <= abs_y) and 
            (objective_y + tolerance >= abs_y)):
            print("Goal is close enough from position")
            return True
        else:
            print("Goal is too far from position")
            return False


    def assign_goal(self):
        print("inside assign_goal")

        close_enough = False

        if(self.check_proximity() == True):
            array_point_patrol.append((self.new_goal.pose.position.x, self.new_goal.pose.position.y))
            close_enough = True
            print("Goal has been reach")
            print("Adding value ")

        if ( (len(free_goals) > 0) and (close_enough == True) ):
            self.publish_new_goal_param(free_goals[0][0], free_goals[0][1])
            print("Goal (" + str(free_goals[0][0]) + ", " + str(free_goals[0][1]) + ") has been assigned")
            print("Poping value new free_goals are " + str(free_goals))
            free_goals.pop(0)


    def shutdown_function(self):
        self.publish_new_goal_param(self.init_x, self.init_y)
        rospy.loginfo("Going back to init position")
        rospy.loginfo(str(self.robot_ns) + " shuting down ---------------")
    

if __name__ == '__main__':
    rospy.init_node('patrolling_node')
    #rate = rospy.Rate(10)
    now = rospy.get_time()
    previous_map_update = 0.0

    free_goals = array_point_patrol

    print("init free_goals: " + str(free_goals))
    print("init array_point_patrol: " + str(array_point_patrol))

    rospy.loginfo("initilisation patrolling_node")

    Patrolling0 = Patrolling(robot0_ns, 1.5, 4.5)
    Patrolling1 = Patrolling(robot1_ns, 3.0, 4.0)
    Patrolling2 = Patrolling(robot2_ns, 4.5, 4.5)

    rospy.on_shutdown(Patrolling0.shutdown_function)
    rospy.on_shutdown(Patrolling1.shutdown_function)
    rospy.on_shutdown(Patrolling2.shutdown_function)

    rospy.sleep(5)

    freq = 5

    Patrolling0.publish_new_goal_param(Patrolling0.init_x, Patrolling0.init_y)
    Patrolling1.publish_new_goal_param(Patrolling1.init_x, Patrolling1.init_y)
    Patrolling2.publish_new_goal_param(Patrolling2.init_x, Patrolling2.init_y)
    
    while not rospy.is_shutdown():
        #rospy.loginfo("loop")

        now = rospy.get_time()
        if((now - Patrolling0.last_run) > freq):
            rospy.loginfo("instruction robot_0")
            Patrolling0.assign_goal()
            Patrolling0.last_run = rospy.get_time()
            print("---------------------------------------------------")

        now = rospy.get_time()
        if((now - Patrolling1.last_run) > freq):
            rospy.loginfo("instruction robot_1")
            Patrolling1.assign_goal()
            Patrolling1.last_run = rospy.get_time()
            print("---------------------------------------------------")

        now = rospy.get_time()
        if((now - Patrolling2.last_run) > freq):
            rospy.loginfo("instruction robot_2")
            Patrolling2.assign_goal()
            Patrolling2.last_run = rospy.get_time()
            print("---------------------------------------------------")


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
