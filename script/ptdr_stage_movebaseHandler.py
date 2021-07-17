#! /usr/bin/env python

import rospy
import tf
import math

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, PoseStamped

robot0_ns = "robot_0"
robot1_ns = "robot_1"
robot2_ns = "robot_2"

#global map_merged_item


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

        self.map_merged_item = OccupancyGrid()
        self.sub_map_merged_item = rospy.Subscriber("map_merged", OccupancyGrid, self.callback_map_merged_item)


    def callback_movebaseHandler(self, msg):
        self.last_received_pose = msg
        #print("self.last_received_pose.pose.position.x: " + str(self.last_received_pose.pose.position.x) )
        #print("self.last_received_pose.pose.position.y: " + str(self.last_received_pose.pose.position.y) )


    def callback_map_merged_item(self, msg):
        self.map_merged_item = msg
        rospy.loginfo("map_merged has been fetched")
        #print(map_merged_item)


    def pose_checker(self):
        pxl_x = pxl_y = state_of_pxl = close_free_cell_x = close_free_cell_y = 0
        

        #removing the far decimals 
        informal_res = math.floor(self.map_merged_item.info.resolution * pow(10, 4)) / pow(10, 4)

        #making sure no div 0
        if(self.map_merged_item.info.resolution != 0.0):
            #fetching the pixel position of the point
            pxl_x = self.new_goal.pose.position.x * (1/informal_res)
            pxl_y = self.new_goal.pose.position.y * (1/informal_res)
            print("pose_checker pxl_x=" + str(pxl_x))
            print("pose_checker pxl_y=" + str(pxl_y))
        else:
            print("map_merged_item.info.resolution is " + str(map_merged_item.info.resolution))

        #checking the type of pxl which is the goal
        if (self.map_merged_item.data[pxl_x, self.map_merged_item.info.height - pxl_y] == 1):
            print('value is correct sending the goal')
        else:
            print('value is not correct calculating new goal')
            '''
            for i in range (1, 11):

                if(state_of_pxl == 1):
                    break

                #not sure (i * 2) or (i * 2 + 1)
                for n in range(i * 2 + 1):
                    if(state_of_pxl == 1):
                        break

                    for m in range(i * 2 + 1):
                        state_of_pxl = self.map_merged_item.data[pxl_x - i + n, self.map_merged_item.info.height - pxl_y - i + m]
                        if(state_of_pxl == 1):
                            break
            '''
            #test up to 11 - 1 = 10 rings
            for rings in range (1, 11):
                print("ring number " + str(rings))

                #line left of the pixel, going down
                for length in range(rings * 2):
                    state_of_pxl = self.map_merged_item.data[pxl_x - rings, self.map_merged_item.info.height - pxl_y + rings - length]
                    if(state_of_pxl == 1):
                        close_free_cell_x = pxl_x - rings
                        close_free_cell_y = self.map_merged_item.info.height - pxl_y + rings - length
                        break

                if(state_of_pxl == 1):
                        break

                #line down of the pixel, going left
                for length in range(rings * 2):
                    state_of_pxl = self.map_merged_item.data[pxl_x - rings + length, self.map_merged_item.info.height - pxl_y - rings ]
                    if(state_of_pxl == 1):
                        close_free_cell_x = pxl_x - rings + length
                        close_free_cell_y = self.map_merged_item.info.height - pxl_y - rings
                        break

                if(state_of_pxl == 1):
                        break

                #line right of the pixel, going up
                for length in range(rings * 2):
                    state_of_pxl = self.map_merged_item.data[pxl_x + rings, self.map_merged_item.info.height - pxl_y - rings + length]
                    if(state_of_pxl == 1):
                        close_free_cell_x = pxl_x + rings
                        close_free_cell_y = self.map_merged_item.info.height - pxl_y - rings + length
                        break

                if(state_of_pxl == 1):
                        break

                #line right of the pixel, going left
                for length in range(rings * 2):
                    state_of_pxl = self.map_merged_item.data[pxl_x + rings - length - 1, self.map_merged_item.info.height - pxl_y + rings ]
                    if(state_of_pxl == 1):
                        close_free_cell_x = pxl_x + rings - length
                        close_free_cell_y = self.map_merged_item.info.height - pxl_y + rings
                        break

                if(state_of_pxl == 1):
                        break

        print("close_free_cell_x pxl is: " + str(close_free_cell_x))
        print("close_free_cell_y pxl is: " + str(close_free_cell_y))

        print("previous self.new_goal.pose.position.x is: " + str(self.new_goal.pose.position.x))
        print("previous self.new_goal.pose.position.y is: " + str(self.new_goal.pose.position.y))

        print("new close_free_cell_x * 0.05 is: " + str(close_free_cell_x * 0.05))
        print("new close_free_cell_y * 0.05 is: " + str(close_free_cell_y * 0.05))

        #self.new_goal.pose.position.x = close_free_cell_x * 0.05
        #self.new_goal.pose.position.y = close_free_cell_y * 0.05


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
        self.new_goal.pose.position.y = 4.5
        self.new_goal.pose.position.x = 4.5
        self.new_goal.pose.orientation.w = 1

        self.new_goal_pub.publish(self.new_goal)


    def publish_new_goal(self):
        self.new_goal.header.seq = 1
        self.new_goal.header.stamp = rospy.Time.now()
        self.new_goal.pose.position.x = 4.5
        self.new_goal.pose.position.y = 4.5
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
        self.new_goal.pose.position.x = 4.5
        self.new_goal.pose.position.y = 4.5
        self.new_goal.pose.orientation.w = 1
        self.new_goal_pub.publish(self.new_goal)
    

if __name__ == '__main__':
    rospy.init_node('many_robots_node')
    #rate = rospy.Rate(10)
    #start_time = rospy.get_rostime()

    movebaseHandler0 = movebaseHandler(robot0_ns, 1.5, 4.5)
    movebaseHandler1 = movebaseHandler(robot1_ns, 3.0, 4.0)
    movebaseHandler2 = movebaseHandler(robot2_ns, 4.5, 4.5)

    rospy.on_shutdown(movebaseHandler0.shutdown_function)
    rospy.on_shutdown(movebaseHandler1.shutdown_function)
    rospy.on_shutdown(movebaseHandler2.shutdown_function)

    rospy.loginfo("initilisation")
    movebaseHandler2.publish_new_goal_param( (movebaseHandler0.init_x + 0.5) , (movebaseHandler0.init_y) )

    #map_merged_metadata = MapMetaData()

    #sub_map_merged_meta = rospy.Subscriber("map_merged_updates", MapMetaData, callback_map_merged_meta)

    rospy.sleep(5)

    pause = 15
    
    while not rospy.is_shutdown():
        rospy.loginfo("loop")

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler0.last_received_pose.pose.position.x: " +str(movebaseHandler0.last_received_pose.pose.position.y))
        print("movebaseHandler0.last_received_pose.pose.position.y: " +str(movebaseHandler0.last_received_pose.pose.position.x))
        movebaseHandler0.pose_checker()
        movebaseHandler0.publish_new_goal_param(movebaseHandler0.last_received_pose.pose.position.y, movebaseHandler0.last_received_pose.pose.position.x)
        rospy.sleep(pause)

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler1.last_received_pose.pose.position.x: " +str(movebaseHandler1.last_received_pose.pose.position.y))
        print("movebaseHandler1.last_received_pose.pose.position.y: " +str(movebaseHandler1.last_received_pose.pose.position.x))
        movebaseHandler1.pose_checker()
        movebaseHandler1.publish_new_goal_param(movebaseHandler1.last_received_pose.pose.position.y, movebaseHandler1.last_received_pose.pose.position.x)
        rospy.sleep(pause)

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler2.last_received_pose.pose.position.x: " +str(movebaseHandler2.last_received_pose.pose.position.y))
        print("movebaseHandler2.last_received_pose.pose.position.y: " +str(movebaseHandler2.last_received_pose.pose.position.x))
        movebaseHandler2.pose_checker()
        movebaseHandler2.publish_new_goal_param(movebaseHandler2.last_received_pose.pose.position.y, movebaseHandler2.last_received_pose.pose.position.x)
        rospy.sleep(pause)
        '''
        movebaseHandler0.publish_new_goal()
        rospy.sleep(10)
        movebaseHandler0.publish_new_goal_stop()
        rospy.sleep(5)
        '''

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
