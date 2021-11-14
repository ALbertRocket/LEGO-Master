import rospy
import rospkg
import os
import yaml
import random

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def spawn_block(config_idx, missing_block=False):

    missing_yellow = False
    missing_green = False

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
    block_yellow_position = positions_dict['block_yellow_positions'][config_idx]

    block_green_position = positions_dict['block_green_positions'][config_idx]

    # Spawn block

    # First delete all blocks
    delete('block_yellow')
    delete('block_green')

    # Spawn yellow
    block_name = 'block_yellow'
    pose = Pose(Point(block_yellow_position[0], block_yellow_position[1], 0), Quaternion(0, 0, 0, 0))
    if not missing_yellow:
        spawn(block_name, open(block_yellow_path, 'r').read(), 'block', pose, 'world')

    # Spawn green
    block_name = 'block_green'
    pose = Pose(Point(block_green_position[0], block_green_position[1], 0), Quaternion(0, 0, 0, 0))
    if not missing_green:
        spawn(block_name, open(block_green_path, 'r').read(), 'block', pose, 'world')

if __name__ == '__main__':
    print("Don't try to run me! Look at the lab manual.")
