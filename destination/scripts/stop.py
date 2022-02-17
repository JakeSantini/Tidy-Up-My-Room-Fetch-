#!/usr/bin/env python
import rospy, actionlib, sys, tf
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray



def stop():
    # Stop navigation
    # Define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    # Set up the frame parameters
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Moving towards goal
    goal.target_pose.pose.position =  Point(0,0,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Stoping Navigation")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    # Stop manipulation
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    rospy.loginfo("Stopping Manipulation")
    move_group.get_move_action().cancel_all_goals()



# If a start or stop call is received act on this
def callback(data):
    if (data.data == "stop emergency"):
        rospy.loginfo("Initiating %s sequence", data.data)
        stop()



if __name__ == '__main__':
    rospy.init_node('stop', anonymous=True)
    
    while not rospy.is_shutdown():
        rospy.Subscriber("user_input", String, callback)
