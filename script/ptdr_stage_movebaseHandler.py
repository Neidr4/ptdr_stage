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

        self.last_run = 0.0

        #Publishers
        self.new_goal = PoseStamped()
        self.new_goal.header.frame_id = "world"
        new_goal_string = str("/" + self.robot_ns + "/move_base_simple/goal")
        self.new_goal_pub = rospy.Publisher(new_goal_string, PoseStamped, queue_size=1)

        self.new_point = PointStamped()
        self.new_point.header.frame_id = "world"
        new_point_string = str("/" + self.robot_ns + "/clicked_point")
        self.new_point_pub = rospy.Publisher(new_point_string, PointStamped, queue_size=1)

        self.new_map = OccupancyGrid()
        self.new_map.header.frame_id = "world"
        new_map_string = str("/" + self.robot_ns + "/new_map")
        self.new_map_pub = rospy.Publisher(new_map_string, OccupancyGrid, queue_size=1)

        self.merged_fuse = OccupancyGrid()
        self.merged_fuse.header.frame_id = "world"
        merged_fuse_string = str("/" + self.robot_ns + "/map")
        self.merged_fuse_pub = rospy.Publisher(merged_fuse_string, OccupancyGrid, queue_size=1)

        #Subscribers
        self.last_received_pose = PoseStamped()
        buffer_string = str("/" + self.robot_ns + "/move_base_simple/buffer")
        self.sub_movebaseHandler = rospy.Subscriber(buffer_string, PoseStamped, self.callback_movebaseHandler)

        self.map_merged_item = OccupancyGrid()
        self.sub_map_merged_item = rospy.Subscriber("map_merged", OccupancyGrid, self.callback_map_merged_item)
        self.merged_map_init_width = 0
        self.merged_map_init_height = 0

        self.previous = 0


    def callback_movebaseHandler(self, msg):
        self.last_received_pose = msg
        self.new_goal.pose.position.x = self.last_received_pose.pose.position.x
        self.new_goal.pose.position.y = self.last_received_pose.pose.position.y
        # print("self.last_received_pose.pose.position.x: " + str(self.last_received_pose.pose.position.x) )
        # print("self.last_received_pose.pose.position.y: " + str(self.last_received_pose.pose.position.y) )


    def callback_map_merged_item(self, msg):
        self.map_merged_item = msg
        #if((self.map_merged_item.info.resolution != 0.0) and (self.merged_map_init_width == 0)):
        # if(self.merged_map_init_width == 0):
        #     self.merged_map_init_width = msg.info.width
        #     self.merged_map_init_width = msg.info.height


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


    def publish_merged_fuse(self, seq):
        self.merged_fuse.header = self.map_merged_item.header
        self.merged_fuse.header.stamp = rospy.Time.now()
        self.merged_fuse.info = self.map_merged_item.info
        self.merged_fuse.data = self.map_merged_item.data
        self.merged_fuse_pub.publish(self.merged_fuse)


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


    ###########################
    #Return new map with checkboard at position x, y given
    ###########################
    def goal_pose_writer(self, map_to_check, size_of_square, x, y):
        previous_color = 0
        data_list_to_publish = [-1] * numpy.size(map_to_check)

        for rings in range (1, size_of_square):
            for rows in range(rings * 2 + 1):
                for column in range(rings * 2 + 1):
                    if (previous_color == 0):
                        map_to_check[y - rings + column, x - rings + rows] = 90
                        previous_color = 90
                    else:
                        map_to_check[y - rings + column, x - rings + rows] = 0
                        previous_color = 0

        #Tranforming the map and publishing it
        data_list_to_publish = numpy.concatenate(map_to_check)
        self.publish_new_map(0, data_list_to_publish)


    def anti_wall_system(self, map_to_check, distance, x, y):
        state_of_pxl = 0
        return_x = x
        return_y = y

        # If a wall is encounter in a direction,
        # it will try to go as far as possible on the opposite direction.

        for top in range(1, distance + 1):
            state_of_pxl = map_to_check[y + top, x]
            if(state_of_pxl > 0):
                for offset in range(1, distance + 1):
                    state_of_pxl = map_to_check[y - offset, x]
                    if(state_of_pxl == 0):
                        return_x = x
                        return_y = y - offset
                return return_x, return_y

        for right in range(1, distance + 1):
            state_of_pxl = map_to_check[y, x + right]
            if(state_of_pxl > 0):
                for offset in range(1, distance + 1):
                    state_of_pxl = map_to_check[y, x - offset]
                    if(state_of_pxl == 0):
                        return_x = x - offset
                        return_y = y
                return return_x, return_y

        for bot in range(1, distance + 1):
            state_of_pxl = map_to_check[y - bot, x]
            if(state_of_pxl > 0):
                for offset in range(1, distance + 1):
                    state_of_pxl = map_to_check[y + offset, x]
                    if(state_of_pxl == 0):
                        return_x = x
                        return_y = y + offset
                return return_x, return_y

        for left in range(1, distance + 1):
            state_of_pxl = map_to_check[y, x - left]
            if(state_of_pxl > 0):
                for offset in range(1, distance + 1):
                    state_of_pxl = map_to_check[y, x + offset]
                    if(state_of_pxl == 0):
                        return_x = x + offset
                        return_y = y
                return return_x, return_y

        return return_x, return_y


    ###########################
    #Simple sqauare check over pixels. Overlaping a lot.
    ###########################
    def checker_classic(self, map_to_check, size_of_square, x, y):
        state_of_pxl = -1
        return_x = x
        return_y = y

        #breaks when first discovered pixel is found
        for rings in range (1, size_of_square + 1):
            #print("Checker_classic ring: " + str(rings) + ". State_of_pxl is: " + str(state_of_pxl))
            if(state_of_pxl == 0):
                break

            for rows in range(rings * 2 + 1):
                if(state_of_pxl == 0):
                    break

                for column in range(rings * 2 + 1):
                    state_of_pxl = map_to_check[y - rings + rows, x - rings + column]
                    if(state_of_pxl == 0):
                        return_x, return_y = self.anti_wall_system(map_to_check, 3, x - rings + column, y - rings + rows)
                        # return_x = x + column -rings
                        # return_y = y + rows - rings
                        break

        return return_x, return_y


    ###########################
    #Snail sqauare check over pixels. No overlaping. Not integrated yet.
    ###########################
    def checker_snail(self, map_to_check, size_of_square, x, y):
        print('funtion not integrated')
        '''            
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


    def pose_checker(self):
        #Declaration of local variables
        objective_x_pxl = objective_y_pxl = close_free_cell_x = close_free_cell_y = 0
        objective_x_m = objective_y_pxl = 0.0
        state_of_pxl = 90
        offset_x = -0.55
        offset_y = -0.4
        pose_checker_x = self.new_goal.pose.position.x
        pose_checker_y = self.new_goal.pose.position.y

        #Send to init value instead. Avoid unnecessary travel
        if((pose_checker_x == 0.0) and (pose_checker_y == 0.0)):
            pose_checker_x = self.init_x + 0.2
            pose_checker_y = self.init_y

        #Testing pose
        # pose_checker_x = 1.5
        # pose_checker_y = 3.7
        # rospy.loginfo("Test occuring, self.new_goal is: (" + str(pose_checker_x) + ", " + str(pose_checker_y) + ")")

        #Making sure no div 0
        if(self.map_merged_item.info.resolution != 0.0):
            #fetching the pixel position of the point

            #removing the far decimals 
            conv_m2pxl = 1 / (math.floor(self.map_merged_item.info.resolution * pow(10, 4)) / pow(10, 4))
            conv_pxl2m = (math.floor(self.map_merged_item.info.resolution * pow(10, 4)) / pow(10, 4))

            #Compute center of the map in pxl
            map_merged_center_x_pxl = int(self.map_merged_item.info.width / 2) 
            map_merged_center_y_pxl = int(self.map_merged_item.info.height / 2)
            print("map_merged_center_pxl: [" + str(map_merged_center_x_pxl) + ", " + str(map_merged_center_y_pxl) + "]")

            #Compute the objective pose in the merged map referential
            objective_x_m = float(pose_checker_x + self.map_merged_item.info.origin.position.x)
            objective_y_m = float(pose_checker_y + self.map_merged_item.info.origin.position.y)
            print("objective_m: (" + str(objective_x_m) + ", " + str(objective_y_m) + ")")

            #Correcting shift
            if(objective_x_m > 10):
                objective_x_m -= 30

            if(objective_y_m > 10):
                objective_y_m -= 30

            #Offset for ajusting
            objective_x_m = float(objective_x_m + offset_x)
            objective_y_m = float(objective_y_m + offset_y)
            print("objective_m after offset: (" + str(objective_x_m) + ", " + str(objective_y_m) + ")")

            #Converting objective into pxl
            objective_x_pxl = int(objective_x_m * conv_m2pxl) + map_merged_center_x_pxl
            objective_y_pxl = int(objective_y_m * conv_m2pxl) + map_merged_center_y_pxl
            print("objective_pxl: [" + str(objective_x_pxl) + ", " + str(objective_y_pxl) + "]")
            
            #Transform map list into array
            data_list = self.map_merged_item.data
            reshape_size = (self.map_merged_item.info.height, self.map_merged_item.info.width)
            data_matrix = numpy.reshape(data_list, reshape_size)

            #Send back the first free cell discovered
            close_free_cell_x_pxl, close_free_cell_y_pxl = self.checker_classic(data_matrix, 10, objective_x_pxl, objective_y_pxl)

            #Convert new goal pose from pixel to meters
            close_free_cell_x_m = close_free_cell_x_pxl * conv_pxl2m + self.map_merged_item.info.origin.position.x
            close_free_cell_y_m = close_free_cell_y_pxl * conv_pxl2m + self.map_merged_item.info.origin.position.y

            #Test the position of received and display it on robot_n/new_map
            self.goal_pose_writer(data_matrix, 10, objective_x_pxl, objective_y_pxl)
            print('Test the position of received')

            #Printing results
            print("close_free_cell_pxl is: [" + str(close_free_cell_x_pxl) + ", " + str(close_free_cell_y_pxl) + "]")
            print("previous self.new_goal.pose.position is: (" + str(self.new_goal.pose.position.x) + ", " + str(self.new_goal.pose.position.y) + ")")
            print("previous close_free_cell_m is: (" + str(close_free_cell_x_m) + ", " + str(close_free_cell_y_m) + ")")

            #Displaying by send point
            self.publish_new_point(0, close_free_cell_x_m, close_free_cell_y_m)
            rospy.sleep(1)
            self.publish_new_point(1, close_free_cell_x_m, close_free_cell_y_m)
            rospy.sleep(1)

            pose_checker_x = close_free_cell_x_m
            pose_checker_y = close_free_cell_y_m

        else:
            rospy.loginfo("map_merged_item.info.resolution is " + str(self.map_merged_item.info.resolution))
            rospy.loginfo("Maybe map_merged is not published yet?")

        print("-----------------------------------------------------------")

        return pose_checker_x, pose_checker_y


    def shutdown_function(self):
        rospy.loginfo(str(self.robot_ns) + " shuting down ---------------")
    

if __name__ == '__main__':
    rospy.init_node('movebaseHandler_node')
    #rate = rospy.Rate(10)
    now = rospy.get_time()
    previous_map_update = 0.0

    rospy.loginfo("initilisation movebaseHandler_node")

    movebaseHandler0 = movebaseHandler(robot0_ns, 1.5, 4.5)
    movebaseHandler1 = movebaseHandler(robot1_ns, 3.0, 4.0)
    movebaseHandler2 = movebaseHandler(robot2_ns, 4.5, 4.5)

    rospy.on_shutdown(movebaseHandler0.shutdown_function)
    rospy.on_shutdown(movebaseHandler1.shutdown_function)
    rospy.on_shutdown(movebaseHandler2.shutdown_function)

    rospy.sleep(5)

    pause = 10
    
    while not rospy.is_shutdown():
        rospy.loginfo("loop")

        now = rospy.get_time()
        if((now - movebaseHandler0.last_run) > 15):
            rospy.loginfo("instruction robot_0")
            # print("movebaseHandler0.last_received_pose.pose.position.x: " +str(movebaseHandler0.last_received_pose.pose.position.y))
            # print("movebaseHandler0.last_received_pose.pose.position.y: " +str(movebaseHandler0.last_received_pose.pose.position.x))
            pose_checker_x0, pose_checker_y0 = movebaseHandler0.pose_checker()
            movebaseHandler0.publish_new_goal_param(pose_checker_x0, pose_checker_y0)
            # rospy.sleep(pause)

        now = rospy.get_time()
        if((now - movebaseHandler1.last_run) > 15):
            rospy.loginfo("instruction robot_1")
            # print("movebaseHandler1.last_received_pose.pose.position.x: " +str(movebaseHandler1.last_received_pose.pose.position.y))
            # print("movebaseHandler1.last_received_pose.pose.position.y: " +str(movebaseHandler1.last_received_pose.pose.position.x))
            pose_checker_x1, pose_checker_y1 = movebaseHandler1.pose_checker()
            movebaseHandler1.publish_new_goal_param(pose_checker_x1, pose_checker_y1)
            #  rospy.sleep(pause)

        now = rospy.get_time()
        if((now - movebaseHandler2.last_run) > 15):
            rospy.loginfo("instruction robot_2")
            # print("movebaseHandler2.last_received_pose.pose.position.x: " +str(movebaseHandler2.last_received_pose.pose.position.y))
            # print("movebaseHandler2.last_received_pose.pose.position.y: " +str(movebaseHandler2.last_received_pose.pose.position.x))
            pose_checker_x2, pose_checker_y2 = movebaseHandler2.pose_checker()
            movebaseHandler2.publish_new_goal_param(pose_checker_x2, pose_checker_y2)
            # rospy.sleep(pause)

        now = rospy.get_time()
        if((now - previous_map_update) > 15):
            movebaseHandler0.publish_merged_fuse(0)
            movebaseHandler1.publish_merged_fuse(0)
            movebaseHandler2.publish_merged_fuse(0)
            previous_map_update = now


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
