#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

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

    rospy.loginfo("Stoping")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            return True

    else:
            return False

    # Stop manipulation

# If a start or stop call is received act on this
def callback(data):
    if (data.data == "stop") or (data.data == "Stop"):
        rospy.loginfo("Initiating %s sequence", data.data)
        stop()

    
# Subscribe to user_input topic
def listener():
    rospy.init_node('stop', anonymous=True)
    rospy.Subscriber("user_input", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
