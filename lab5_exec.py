#!/usr/bin/env python
import copy
import time
import rospy
import rospkg
import os
import random
import cv2
import argparse
import sys
import math

from geometry_msgs.msg import Point
from lab5_spawn_block import *
from lab5_blob_search import *
from lab5_header import *
from lab5_func import *

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 130*PI/180.0]

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

num_configs = 5

# Place holder for block world positions
block_green_world = None
block_yellow_world = None
block_cylinder_world = None
block_rectangle_world = None
# Final goal positions
# PLEASE USE THE FOLLOWING GOAL
z_end_H = 0.032
green_world_goal = [0.2, -0.032]
yellow_world_goal = [0.3, -0.032]
cylinder_world_goal = [0.4, -0.032]
rectangle_world_goal = [0.2, -0.1]
############## Your Code Start Here ##############

"""
TODO: define ROS topic callback funtions for getting the position of blocks
Whenever block_[color]/world publishes info these callback functions are called.
"""
# image (camera) position publisher
# world position subscriber
def gripper_callback(msg):
    
    global digital_gripper
    digital_gripper = msg.DIGIN

############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################

############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates (Point)
    target_xw_yw_zw: where to place the block in global coordinates (Point)

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    global digital_in_0

    xstart = start_xw_yw_zw[0]
    ystart = start_xw_yw_zw[1]
    zstart = start_xw_yw_zw[2]
    zstart_high = zstart + 0.12
    xend = target_xw_yw_zw[0]
    yend = target_xw_yw_zw[1]
    zend = target_xw_yw_zw[2]
    #print('zend',zend)
    zend_high = zend + 0.12
    blobdest_lower = lab_invk(xstart, ystart, zstart, 0)
    blobdest_higher = lab_invk(xstart, ystart, zstart_high, 0)
    blobtarget_lower = lab_invk(xend, yend, zend, 0)
    blobtarget_higher = lab_invk(xend, yend, zend_high, 0)
    move_arm(pub_cmd, loop_rate, go_away, vel, accel)
    time.sleep(5.0)
    move_arm(pub_cmd, loop_rate, blobdest_higher, 4.0, 4.0)
    time.sleep(1)
    move_arm(pub_cmd, loop_rate, blobdest_lower, 4.0, 4.0)
    time.sleep(2.0)
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(1.0)
    if digital_gripper == 0:
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        gripper(pub_cmd, loop_rate, suction_off)
        sys.exit()
    move_arm(pub_cmd, loop_rate, blobdest_higher, 4.0 ,4.0)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, blobtarget_higher, 4.0 ,4.0)
    time.sleep(1)
    move_arm(pub_cmd, loop_rate, blobtarget_lower, 0.5 ,4.0)
    #time.sleep(1.0)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, blobtarget_higher, 4.0 ,4.0)
    move_arm(pub_cmd, loop_rate, go_away, 4.0 ,4.0)
    time.sleep(1.0)

    error = 0

    return error

############### Your Code End Here ###############

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")


    def image_callback(self, data):

        global block_green_world # store found red blocks in this list
        global block_yellow_world # store found green blocks in this list
        global block_cylinder_world
        global block_rectangle_world

        try:
		  # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		# You will need to call blob_search() function to find centers of red blocks
		# and green blocks, and store the centers in xw_yw_R & xw_yw_G respectively.

		# If no blocks are found for a particular color, you can return an empty list,
		# to xw_yw_R or xw_yw_G.

		# Remember, xw_yw_R & xw_yw_G are in global coordinates, which means you will 
		# do coordinate transformation in the blob_search() function, namely, from 
		# the image frame to the global world frame. 
		
        block_yellow_world = blob_search(cv_image, "yellow")
        #print('yellow_block',block_yellow_world)
        block_green_world = blob_search(cv_image, "green")
        #print('green_block',block_green_world)
        block_cylinder_world = blob_search(cv_image, "cylinder")
        block_rectangle_world = blob_search(cv_image, "rectangle")
        
        
def main():

    global go_away
    global block_green_world
    global block_yellow_world
    global block_cylinder_world
    global block_rectangle_world
    global green_world_goal
    global yellow_world_goal
    global cylinder_world_goal
    global rectangle_world_goal
    # Parser
    #parser = argparse.ArgumentParser(description='Please specify if a block is taken away or not')
    #parser.add_argument('--missing', type=str, default='False')
    #parser.add_argument('--block', type=int, default=0)
    #args = parser.parse_args()

    # Check parser
    #if args.missing.lower() == 'true':
    #    missing_block = True
    #elif args.missing.lower() == 'false':
    #    missing_block = False
    #else:
    #    print("Invalid argument for missing block, enter True of False")
    #    sys.exit()

    #if args.block < 0 or args.block > 4:
    #    print("Invalid argument for block.  Enter a number from 0 to 4")
    #    sys.exit()
    #else:
    #    config_idx = args.block - 1

    # Initialize ROS node
    rospy.init_node('lab5_exec', anonymous=True)

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############

    # Define image position publisher
    # rospy.Publisher(?)
    ic = ImageConverter(SPIN_RATE)

    time.sleep(1)
    # Define world position subscriber
    # rospy.Subscriber(?)

    ############## Your Code End Here ###############

    # Initialize ROS pack
    rospack = rospkg.RosPack()

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)


    print('xw_yw_Y: ', block_yellow_world)
    print('xw_yw_G: ', block_green_world)
    print('xw_yw_C: ', block_cylinder_world)
    print('xw_yw_R: ', block_rectangle_world)
    while block_yellow_world != None and block_green_world != None and block_cylinder_world != None :

        i = 0 
        byw = block_yellow_world
        bgw = block_green_world
        bcw = block_cylinder_world
        brw = block_rectangle_world
        while i < len(byw):
            yellow = list(byw[i]) 
            yellow.append(z_end_H)
            yellow_goal = yellow_world_goal
            yellow_goal.append(0)
            if i == 0:
                yellow_goal[2] = z_end_H 
                
            else:
                yellow_goal[2] = 0.015 + 0.032*i
                
            move_block(pub_command, loop_rate, yellow, yellow_goal, vel, accel)
            i = i+1
        j = 0
        while j < len(bgw):
            green = list(bgw[j])
            green.append(z_end_H)
            green_goal = green_world_goal
            green_goal.append(0)
            if i == 0:
                green_goal[2] = z_end_H 
  
            else:
                green_goal[2] = 0.015 + 0.032*i
            
            move_block(pub_command, loop_rate, green, green_goal, vel, accel)
            j = j+1 

        cylinder = list(bcw[0])
        cylinder.append(z_end_H)
        cylinder_goal = cylinder_world_goal
        cylinder_goal.append(0)
        cylinder_goal[2] = z_end_H 
        #print("cylinder",cylinder)
        move_block(pub_command, loop_rate, cylinder, cylinder_goal, vel, accel)

        rectangle = list(brw[0])
        rectangle.append(z_end_H)
        rectangle_goal = rectangle_world_goal
        rectangle_goal.append(0)
        rectangle_goal[2] = z_end_H 
        move_block(pub_command, loop_rate, rectangle, rectangle_goal, vel, accel)

        move_arm(pub_command, loop_rate, go_away, vel, accel)
        rospy.loginfo("Task Completed!")
        print("Use Ctrl+C to exit program")
        rospy.spin()
    

    #time.sleep(5)

    print('complete!')

    ############## Your Code End Here ###############

    # Move arm to away position
    #move_arm(pub_command, loop_rate, go_away, vel, accel)
    #rospy.sleep(1)

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
