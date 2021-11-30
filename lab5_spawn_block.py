#!/usr/bin/env python
import rospy
import rospkg
import os
import yaml
import random
import argparse

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def spawn_block(config_idx, missing_block=False):

    missing_yellow = False
    missing_green = False
    missing_cylinder = False
    missing_rectangle = False

    if missing_block:
        if random.randint(0, 1) == 0:
            missing_yellow = True
        else:
            missing_green = True

    # Initialize ROS pack
    rospack = rospkg.RosPack()

    # Get path to block
    ur_path = rospack.get_path('ur_description')
    lab5_path = rospack.get_path('lab5pkg_py')
    block_yellow_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
    block_green_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
    cylinder_path = os.path.join(ur_path, 'urdf', 'cylinder.urdf')
    rectangle_path = os.path.join(ur_path, 'urdf', 'rectangle.urdf')

    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # Get YAML path
    yaml_path = os.path.join(lab5_path, 'scripts', 'configs',  'positions.yaml')

    # Load block positions
    with open(yaml_path, 'r') as f:
        positions_dict = yaml.load(f)

    # IF YOU USE THOSE POSITIONS DIRECTLY, YOU WILL GET A ZERO
    # NO EXCUSES, AND WE WILL RUN CHECKS
    randnum = random.randint(1,config_idx)
    print(randnum)

    #for i in range(1,randnum):
    #    block_yellow_position[i] = positions_dict['block_yellow_positions'][i]
    #    block_green_position[i] = positions_dict['block_green_positions'][i]

    # Spawn block
    #print(3)
    # First delete all blocks
    for j in range(0,3):
        delete('block_yellow'+str(j))
        delete('block_green'+str(j))
    
    delete('cylinder')
    delete('rectangle')

    # Spawn yellow
    for i in range(0,randnum):
        block_name = 'block_yellow' + str(i)
        block_yellow_position = positions_dict['block_yellow_positions'][i]
        pose = Pose(Point(block_yellow_position[0], block_yellow_position[1], 0), Quaternion(0, 0, 0, 0))
        if not missing_yellow:
            spawn(block_name, open(block_yellow_path, 'r').read(), 'block', pose, 'world')

    # Spawn green
    for i in range(0,randnum):
        block_name = 'block_green' + str(i)
        block_green_position = positions_dict['block_green_positions'][i]
        pose = Pose(Point(block_green_position[0], block_green_position[1], 0), Quaternion(0, 0, 0, 0))
        if not missing_green:
            spawn(block_name, open(block_green_path, 'r').read(), 'block', pose, 'world')   

    block_name = 'cylinder' 
    cylinder_position = positions_dict['cylinder_positions'][1]
    pose = Pose(Point(cylinder_position[0], cylinder_position[1], 0), Quaternion(0, 0, 0, 0))
    if not missing_cylinder:
        spawn(block_name, open(cylinder_path, 'r').read(), 'block', pose, 'world')   

    block_name = 'rectangle' 
    rectangle_position = positions_dict['rectangle_positions'][1]
    pose = Pose(Point(rectangle_position[0], rectangle_position[1], 0), Quaternion(0, 0, 0, 0))
    if not missing_rectangle:
        spawn(block_name, open(rectangle_path, 'r').read(), 'block', pose, 'world')  

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Please specify if a block is taken away or not')
    parser.add_argument('--missing', type=str, default='False')
    parser.add_argument('--block', type=int, default=0)
    args = parser.parse_args()
    #print(1)
    # Check parser
    if args.missing.lower() == 'true':
        missing_block = True
    elif args.missing.lower() == 'false':
        missing_block = False
    else:
        print("Invalid argument for missing block, enter True of False")
        sys.exit()

    if args.block < 0 or args.block > 4:
        print("Invalid argument for block.  Enter a number from 0 to 4")
        sys.exit()
    else:
        config_idx = args.block -1
    #print(2)


    spawn_block(config_idx, missing_block)
    print('end')
