#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

# Publishes to other nodes that destination has been reached
def talker():
    pub = rospy.Publisher('navigation_return', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Publishing: success")

    # Publish success for 1 seconds
    for x in range(0,10):         
        pub.publish("success")
        rate.sleep()

# Function that initiates navigation when an input is received
def callback(data):
    rospy.loginfo("%s requested", data.data)
    complete = map_navigation(data)

    # If destination reached call talker
    if complete:
        talker()

# Function that subscribes to user_input topic
def listener():
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("user_input", String, callback)
    rospy.spin()


def map_navigation(data):

    # Location coordinates
    xBench = 2
    yBench = 3
    xBox = 1
    yBox = 1

    goalReached = False
    choice = data.data

    # Determine coordinates based off user input
    if (choice == "Bench"):

      goalReached = moveToGoal(xBench, yBench)

    elif (choice == "Box"):

      goalReached = moveToGoal(xBox, yBox)

    else:
      return False

    # Check if destination reached
    if (goalReached):
      rospy.loginfo("Destination Reached")
      return True

    else:
      rospy.loginfo("Destination Not Reached")
      return False


def moveToGoal(xGoal,yGoal):

    # Define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    # Set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Moving towards goal
    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Moving to location")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            return True

    else:
            return False


def shutdown():
      rospy.loginfo("Quit program")
      rospy.sleep()

if __name__ == '__main__':
    try:
        listener() # Call listener
        rospy.spin() # Repeat

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated")

