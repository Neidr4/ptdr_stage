#! /usr/bin/env python

import rospy
import tf
import math
import numpy
from PIL import Image

from std_msgs.msg import Float32MultiArray, String, Int32, Int64, Int16
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, PoseStamped, PointStamped

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
        #print("new_goal_string = " + str(new_goal_string) )
        self.new_goal_pub = rospy.Publisher(new_goal_string, PoseStamped, queue_size=1)

        self.new_point = PointStamped()
        self.new_point.header.frame_id = "world"
        new_point_string = str("/" + self.robot_ns + "/clicked_point")
        self.new_point_pub = rospy.Publisher(new_point_string, PointStamped, queue_size=1)

        self.new_map = OccupancyGrid()
        self.new_map.header.frame_id = "world"
        new_map_string = str("/" + self.robot_ns + "/new_map")
        self.new_map_pub = rospy.Publisher(new_map_string, OccupancyGrid, queue_size=1)

        #Subscribers
        self.last_received_pose = PoseStamped()
        buffer_string = str("/" + self.robot_ns + "/move_base_simple/buffer")
        self.sub_movebaseHandler = rospy.Subscriber(buffer_string, PoseStamped, self.callback_movebaseHandler)

        self.map_merged_item = OccupancyGrid()
        self.sub_map_merged_item = rospy.Subscriber("map_merged", OccupancyGrid, self.callback_map_merged_item)

        self.previous = 0


    def callback_movebaseHandler(self, msg):
        self.last_received_pose = msg
        #print("self.last_received_pose.pose.position.x: " + str(self.last_received_pose.pose.position.x) )
        #print("self.last_received_pose.pose.position.y: " + str(self.last_received_pose.pose.position.y) )


    def callback_map_merged_item(self, msg):
        self.map_merged_item = msg
        #rospy.loginfo("map_merged has been fetched")
        #print(self.map_merged_item.info)

    def publish_new_point(self, seq, x, y):
        self.new_point.header.seq = seq
        self.new_point.header.stamp = rospy.Time.now()
        #print("rospy.Time.now() = " + str(rospy.Time.now()))
        self.new_point.point.x = x
        self.new_point.point.y = y
        #print("self.new_point = " + str(self.new_point))
        self.new_point_pub.publish(self.new_point)

    def publish_new_map(self, seq, data_provided):
        self.new_map.header = self.map_merged_item.header
        self.new_map.header.stamp = rospy.Time.now()
        self.new_map.info = self.map_merged_item.info
        self.new_map.data = data_provided
        self.new_map_pub.publish(self.new_map)
        '''
        self.new_map.header.seq = seq
        self.new_map.header.stamp = rospy.Time.now()
        #print("rospy.Time.now() = " + str(rospy.Time.now()))
        self.new_map.header.frame_id = "world"
        self.new_map.info.resolution = 0.05
        self.new_map.info.width = self.map_merged_item.info.width
        self.new_map.info.height = self.map_merged_item.info.height
        self.new_map.info.origin = self.map_merged_item.info.origin
        self.new_map.data = data_provided
        self.new_point_pub.publish(self.new_map)
        '''

    def pose_checker(self):
        objective_x_pxl = objective_y_pxl = close_free_cell_x = close_free_cell_y = 0
        state_of_pxl = 90

        #making sure no div 0
        if(self.map_merged_item.info.resolution != 0.0):
            #fetching the pixel position of the point

            #removing the far decimals 
            conv_m2pxl = 1 / (math.floor(self.map_merged_item.info.resolution * pow(10, 4)) / pow(10, 4))
            conv_pxl2m = (math.floor(self.map_merged_item.info.resolution * pow(10, 4)) / pow(10, 4))

            map_merged_center_x_pxl = int(self.map_merged_item.info.width / 2) 
            map_merged_center_y_pxl = int(self.map_merged_item.info.height / 2)
            print("map_merged_center_pxl: [" + str(map_merged_center_x_pxl) + ", " + str(map_merged_center_y_pxl) + "]")

            map_merged_origin_x_pxl = int(map_merged_center_x_pxl + self.map_merged_item.info.origin.position.x * conv_m2pxl)
            map_merged_origin_y_pxl = int(map_merged_center_y_pxl + self.map_merged_item.info.origin.position.y * conv_m2pxl)
            print("map_merged_origin_pxl: [" + str(map_merged_origin_x_pxl) + ", " + str(map_merged_origin_y_pxl) + "]")
            print("map_merged_origin_m: [" + str(self.map_merged_item.info.origin.position.x) + ", " + str(self.map_merged_item.info.origin.position.y) + "]")

            #Testing pose
            self.new_goal.pose.position.x = 1.0
            self.new_goal.pose.position.y = 4.0
            print("self.new_goal is: (" + str(self.new_goal.pose.position.x) + ", " + str(self.new_goal.pose.position.y) + ")")

            objective_x_m = self.new_goal.pose.position.x + self.map_merged_item.info.origin.position.x
            objective_y_m = self.new_goal.pose.position.y + self.map_merged_item.info.origin.position.y
            print("objective_m: (" + str(objective_x_m) + ", " + str(objective_y_m) + ")")

            objective_x_pxl = int(objective_x_m * conv_m2pxl) + map_merged_center_x_pxl
            objective_y_pxl = int(objective_y_m * conv_m2pxl) + map_merged_center_y_pxl
            print("objective_pxl: [" + str(objective_x_pxl) + ", " + str(objective_y_pxl) + "]")

            # test_x = objective_x_pxl * conv_pxl2m
            # test_y = objective_y_pxl * conv_pxl2m

            # self.publish_new_point(0, test_x, test_y)
            # rospy.sleep(0.5)
            # self.publish_new_point(1, test_x, test_y)
            # rospy.sleep(0.5)
            
            #Transform list into matrix
            data_list = self.map_merged_item.data
            data_matrix = numpy.reshape(data_list, (self.map_merged_item.info.height, self.map_merged_item.info.width))

            data_list_sec = [-1] * len(data_list)
            data_list_third = [-1] * len(data_list)


            # for i in range (len(data_list)):
            #     if(data_list[i] != -1):
            #         data_list_sec[i] = 90

            print("type(data_matrix)" + str(type(data_matrix)))
            print("type(data_list_sec)" + str(type(data_list_sec)))
            print("type(data_list_third)" + str(type(data_list_third)))

            # #Simple test
            # simple_test_x = 8*20
            # simple_test_y = 9*20
            # print("simple test x: " + str(simple_test_x))
            # print("simple test x: " + str(simple_test_y))
            # print("data_matrix = " + str(data_matrix[simple_test_x, simple_test_y]))
            # self.publish_new_point(0, simple_test_x*0.05, simple_test_y*0.05)
            # #print("data_matrix: " + str(data_matrix))

        else:
            print("map_merged_item.info.resolution is " + str(self.map_merged_item.info.resolution))
            print("maybe the map is not published yet?")

        # self.publish_new_point(0, objective_x_m, objective_y_m)
        # self.publish_new_point(1, test_x, test_y)

        #checking the type of pxl which is the goal
        if (data_matrix[objective_y_pxl, objective_x_pxl] == 0):
            print('value is correct sending the goal')

            #create a dashed square
            for rings in range (1, 4):
                for rows in range(rings * 2 + 1):
                    for column in range(rings * 2 + 1):
                        state_of_pxl = data_matrix[objective_x_pxl + rows, objective_y_pxl + column]
                        if (self.previous == 0):
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 90
                            self.previous = 90
                        else:
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 0
                            self.previous = 0

        else:
            print('value is not correct calculating new goal')
            print("objective_pxl: [" + str(objective_x_pxl) + ", " + str(objective_y_pxl) + "]")
            objective_x_pxl -= 10
            objective_y_pxl -= 1
            print("objective_pxl: [" + str(objective_x_pxl) + ", " + str(objective_y_pxl) + "]")


            for rings in range (1, 4):
                for rows in range(rings * 2 + 1):
                    for column in range(rings * 2 + 1):
                        state_of_pxl = data_matrix[objective_x_pxl + rows, objective_y_pxl + column]
                        if (self.previous == 0):
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 90
                            self.previous = 90
                        else:
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 0
                            self.previous = 0
            '''
            for rings in range (1, 11):
                # if(state_of_pxl == 1):
                #     break

                #not sure (rings * 2) or (rings * 2 + 1)
                for rows in range(rings * 2 + 1):
                    # if(state_of_pxl == 1):
                    #     break

                    for column in range(rings * 2 + 1):
                        state_of_pxl = data_matrix[objective_x_pxl + rows, objective_y_pxl + column]
                        if (self.previous == 0):
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 90
                            self.previous = 90
                        else:
                            data_matrix[objective_y_pxl + column, objective_x_pxl + rows] = 0
                            self.previous = 0
                        # if(state_of_pxl == 1):
                        #     break
            
            #test up to 11 - 1 = 10 rings
            for rings in range (1, 51):
                print("Ring number " + str(rings) + ". State of pxl is " + str(state_of_pxl))
                if(state_of_pxl == 0):
                        break

                #line left of the pixel, going down
                for length in range(rings * 2):
                    state_of_pxl = data_matrix[objective_y_pxl + rings - length, objective_x_pxl - rings]
                    data_matrix[objective_x_pxl - rings, objective_y_pxl + rings - length] = 90
                    if(state_of_pxl == 0):
                        close_free_cell_x = objective_x_pxl - rings
                        close_free_cell_y = objective_y_pxl + rings - length
                        break

                if(state_of_pxl == 0):
                        break

                #line down of the pixel, going left
                for length in range(rings * 2):
                    state_of_pxl = data_matrix[objective_x_pxl - rings + length, objective_y_pxl - rings]
                    data_matrix[objective_x_pxl - rings + length, objective_y_pxl - rings] = 90
                    if(state_of_pxl == 0):
                        close_free_cell_x = objective_x_pxl - rings + length
                        close_free_cell_y = objective_y_pxl - rings
                        break

                if(state_of_pxl == 0):
                        break

                #line right of the pixel, going up
                for length in range(rings * 2):
                    state_of_pxl = data_matrix[objective_x_pxl + rings, objective_y_pxl - rings + length]
                    data_matrix[objective_x_pxl + rings, objective_y_pxl - rings + length] = 90
                    if(state_of_pxl == 0):
                        close_free_cell_x = objective_x_pxl + rings
                        close_free_cell_y = objective_y_pxl - rings + length
                        break

                if(state_of_pxl == 0):
                        break

                #line right of the pixel, going left
                for length in range(rings * 2):
                    state_of_pxl = data_matrix[objective_x_pxl + rings - length - 1, objective_y_pxl + rings]
                    data_matrix[objective_x_pxl + rings - length - 1, objective_y_pxl + rings] = 90
                    if(state_of_pxl == 0):
                        close_free_cell_x = objective_x_pxl + rings - length
                        close_free_cell_y = objective_y_pxl + rings
                        break
            '''

        data_list_sec = numpy.concatenate(data_matrix)
        self.publish_new_map(0, data_list_sec)

        print("close_free_cell_x pxl is: " + str(close_free_cell_x))
        print("close_free_cell_y pxl is: " + str(close_free_cell_y))

        print("previous self.new_goal.pose.position.x is: " + str(self.new_goal.pose.position.x))
        print("previous self.new_goal.pose.position.y is: " + str(self.new_goal.pose.position.y))

        print("new close_free_cell_x * 0.05 is: " + str(close_free_cell_x * 0.05))
        print("new close_free_cell_y * 0.05 is: " + str(close_free_cell_y * 0.05))

        self.publish_new_point(2, close_free_cell_x * 0.05 - self.map_merged_item.info.origin.position.x, close_free_cell_y * 0.05 - self.map_merged_item.info.origin.position.y)
        rospy.sleep(0.5)

        #self.new_goal.pose.position.x = close_free_cell_x * 0.05
        #self.new_goal.pose.position.y = close_free_cell_y * 0.05

        print("-----------------------------------------------------------")


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
        #self.new_goal_pub.publish(self.new_goal)
    

if __name__ == '__main__':
    rospy.init_node('movebaseHandler_node')
    #rate = rospy.Rate(10)
    #start_time = rospy.get_rostime()

    rospy.loginfo("initilisation movebaseHandler_node")

    movebaseHandler0 = movebaseHandler(robot0_ns, 1.5, 4.5)
    movebaseHandler1 = movebaseHandler(robot1_ns, 3.0, 4.0)
    movebaseHandler2 = movebaseHandler(robot2_ns, 4.5, 4.5)

    rospy.on_shutdown(movebaseHandler0.shutdown_function)
    rospy.on_shutdown(movebaseHandler1.shutdown_function)
    rospy.on_shutdown(movebaseHandler2.shutdown_function)


    #map_merged_metadata = MapMetaData()

    #sub_map_merged_meta = rospy.Subscriber("map_merged_updates", MapMetaData, callback_map_merged_meta)

    rospy.sleep(5)

    pause = 10
    
    while not rospy.is_shutdown():
        rospy.loginfo("loop")

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler0.last_received_pose.pose.position.x: " +str(movebaseHandler0.last_received_pose.pose.position.y))
        print("movebaseHandler0.last_received_pose.pose.position.y: " +str(movebaseHandler0.last_received_pose.pose.position.x))
        movebaseHandler0.pose_checker()
        #movebaseHandler0.publish_new_goal_param(movebaseHandler0.last_received_pose.pose.position.y, movebaseHandler0.last_received_pose.pose.position.x)
        rospy.sleep(pause)

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler1.last_received_pose.pose.position.x: " +str(movebaseHandler1.last_received_pose.pose.position.y))
        print("movebaseHandler1.last_received_pose.pose.position.y: " +str(movebaseHandler1.last_received_pose.pose.position.x))
        movebaseHandler1.pose_checker()
        #movebaseHandler1.publish_new_goal_param(movebaseHandler1.last_received_pose.pose.position.y, movebaseHandler1.last_received_pose.pose.position.x)
        rospy.sleep(pause)

        rospy.loginfo("instruction robot_0")
        print("movebaseHandler2.last_received_pose.pose.position.x: " +str(movebaseHandler2.last_received_pose.pose.position.y))
        print("movebaseHandler2.last_received_pose.pose.position.y: " +str(movebaseHandler2.last_received_pose.pose.position.x))
        movebaseHandler2.pose_checker()
        #movebaseHandler2.publish_new_goal_param(movebaseHandler2.last_received_pose.pose.position.y, movebaseHandler2.last_received_pose.pose.position.x)
        rospy.sleep(pause)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
