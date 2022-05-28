Tidy Up My Room Robot! FYP Project

This repoistory stores the source file for the catkin workspace.

To run on a real Fetch robot:

1. Run the navigation stack on Fetch and give it a map (map5 is for room G14):
roslaunch fetch_navigation fetch_nav.launch map_file:=/home/hrigroup/jake/jsan_ws/src/destination/map5.yaml

2. Open RVIZ on your laptop terminal: 
roscd fetch_navigation/config
export ROS_MASTER_URI=http://fetch1077:11311
rviz -d navigation.rviz

3. Run MoveIt on Fetch:
roslaunch fetch_moveit_config move_group.launch

4. Run the GUI on you laptop:
export ROS_MASTER_URI=http://Fetch1077:11311
export ROS_IP= 160.69.69.113
. ~/jsan_ws/devel/setup.bash
rosrun destination user_input.py

5. Run the overall launch file on Fetch:
source /opt/ros/melodic/setup.sh
. ~/jake/jsan_ws/devel/setup.bash
roslaunch destination destination.launch
