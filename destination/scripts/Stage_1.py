#!/usr/bin/env python
import rospy
import actionlib
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

sc = SoundClient()
rospy.sleep(1)
current_table = 0
current_colour = 'unknown'


# Initialise Tables and Storage Containers
class table:
    def _init_(self, location):
        self.location = location
        self.empty = False

T1 = table([1,1])
T2 = table([2,2])
RS = table([3,3])
BS = table([4,4])
home = table([0,0])

def pick(msg):
    # see how many objects there are to update table.empty
    if (length(msg) == 1) and (current_table == 1):
        T1.empty = True
    if (length(msg) == 1) and (current_table == 2):
        T2.empty = True
           
    # Get tf information
    marker = msg.transforms[0]
    ID = marker.fiducial_id
    if (ID == 100) or (ID == 101):
        current_colour = 'red'
    else:
        current_colour = 'blue'
   
    trans = marker.transform.translation
    rot = marker.transform.rotation

    ######
    pick code
    ######
 

# Subscribe to aruco_detect topics for marker to camera transforms
def pick_listener():
    raise_torso()
    head_tilt()
    open_gripper()
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick)
    

# Navigate to location 
def navigate(location):
    # Define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    # Set up the frame parameters
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Moving towards goal
    goal.target_pose.pose.position =  Point(location[0],location[1],0)
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



def start():
    while T1.empty == False: # If the first table is not empty
        navigate = navigate(T1.location) # navigate to table 1
        if navigate == True:
            current_table = 1
            pick_listener() # pick an object up and return its colour
            if current_colour == 'red':
                navigate(RS.location) # if object is red go to red storage
            else:
                navigate(BS.location) # else the object is blue therefore go to blue container
            place() # place the object in the container

    while T2.empty == False:
        navigate(T2.location)
        if navigate == True:
            current_table = 2
            colour = pick_listener()
            if current_colour == 'red':
                navigate(RS.location)
            else:
                navigate(BS.location)
            place()

    navigate(home.location)
    lowe_torso()
    open_gripper()
    head_reset()
    sc.say('I have finished tidying let me know when you want me to start again')

# Fetch asks if it should start tidying
if __name__ == '__main__':
    rospy.init_node('Main_Control', anonymous=True)
    lower_torso()
    open_gripper()
    head_reset()
    sc.say('Hello my name is Fetch, do you want me to start tidying?: ')
    input_str = raw_input("(yes/no)")

    if input_str == 'yes':
        sc.say('I will begin tidying now')
        start() # start tidying
    else:
        sc.say('Ok let me know when you want me to start')
        input_str = raw_input('Enter yes when you want me to start: ')
        if input_str == 'yes':
            sc.say('I will begin tidying now')
            start()



    

        
        
