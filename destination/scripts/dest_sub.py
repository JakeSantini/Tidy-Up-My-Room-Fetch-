#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    map_navigation(data)
    
def listener():
    rospy.init_node('dest_sub', anonymous=True)
    rospy.Subscriber("destination", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def map_navigation(data):
    xBench = 1
    yBench = 1
    xBox = 3
    yBox = 3
    goalReached = False

    choice = 'q'
    choice = data.data

    if (choice == "Bench"):

      goalReached = moveToGoal(xBench, yBench)

    elif (choice == "Box"):

      goalReached = moveToGoal(xBox, yBox)


    if (choice!='q'):

      if (goalReached):
        rospy.loginfo("Destination Reached")
        #rospy.spin()

      else:
        rospy.loginfo("Destination Not Reached")

def moveToGoal(xGoal,yGoal):

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("Destination Reached")
            return True

    else:
            rospy.loginfo("Destination Not Reached")
            return False


def shutdown():
      rospy.loginfo("Quit program")
      rospy.sleep()

if __name__ == '__main__':
    try:

        listener()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("navigation node terminated.")




"""

class map_navigation():

  def __init__(self):

    # declare the coordinates of interest
    self.xBench = 1
    self.yBench = 1
    self.xBox = 3
    self.yBox = 3
    self.goalReached = False

    # initiliaze
    #rospy.init_node('map_navigation', anonymous=False)
    choice = 'q'
    choice = data.data

    if (choice == "Bench"):

      self.goalReached = self.moveToGoal(self.xBench, self.yBench)

    elif (choice == "Box"):

      self.goalReached = self.moveToGoal(self.xBox, self.yBox)


    if (choice!='q'):

      if (self.goalReached):
        rospy.loginfo("Destination Reached")
        #rospy.spin()

      else:
        rospy.loginfo("Destination Not Reached")

    while choice != 'q':
      choice = data.data
      if (choice == "Bench"):

        self.goalReached = self.moveToGoal(self.xBench, self.yBench)

      elif (choice == "Box"):

        self.goalReached = self.moveToGoal(self.xBox, self.yBox)

      if (choice!='q'):

        if (self.goalReached):
          rospy.loginfo("Destination Reached")
          #rospy.spin()

        else:
          rospy.loginfo("Destination Not Reached")


  def shutdown(self):
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)

      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("Destination Reached")
              return True

      else:
              rospy.loginfo("Destination Not Reached")
              return False

"""


