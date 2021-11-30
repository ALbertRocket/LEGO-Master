# LEGO Master

LEGO Master is a function based on Python and UR3 for dealing with LEGO sorting.

## Members

Liuyang Cheng
Huangchang Ji
Xingchen Zhou

## Usage

This project is aimed to clean the LEGO blocks on the floor and place them in order.

The color and shape of LEGO blocks can be dectected and their position will be recorded.

Based on the positions, UR3 arm can pick blocks and place them at the right place.

### Terminal 1  
#### $ cd ~/catkin_NETID  
#### $ catkin_make  
#### $ source devel/setup.bash  
#### $ roslaunch ur3_driver ur3_gazebo.launch  

### Terminal 2  
#### $ source devel/setup.bash  
#### $ rosrun lab5pkg_py lab5_coordinate_converter.py  

### Terminal 3  
#### $ source devel/setup.bash  
#### $ rosrun lab5pkg_py lab5_exec.py --missing False --block 4

## Demo
Update1: https://www.youtube.com/watch?v=iueSvHNdfyA

Update2: https://www.youtube.com/watch?v=4JLpOHLt_TQ
