#!/usr/bin/env python
import rospy, sys, actionlib, tf
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


# Initialise the sound client so Fetch can speak
sc = SoundClient()
rospy.sleep(1)

# Initialise global variables related to user input
tidy = False

# Initialise Tables and Storage Containers
class table:
    def __init__(self, location, name, objects=1):
        self.location = location
        self.name = name
        self.objects = objects
        self.empty = False

Table1 = table([2,2],'Table 1')
Table2 = table([2,-2],'Table 2')
Tables = [Table1,Table2]
RedStorage = table([3,0],'Red Storage',1)
BlueStorage = table([-2,0],'Blue Storage',1)
home = table([0,0],'Home')



# Function that controls the head tilt
def head_tilt(height):
    rospy.Subscriber("user_input", String, exception_action)
    head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1
    goal.target.point.y = 0
    goal.target.point.z = height
    head_client.send_goal(goal)
    head_client.wait_for_result()



# Function that controls the gripper
def gripper(position):
    rospy.Subscriber("user_input", String, exception_action)
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 50
    goal.command.position = position
    gripper.send_goal(goal)
    gripper.wait_for_result()



# Function that controls the torso and arm tuck
def torso(height):
    rospy.Subscriber("user_input", String, exception_action)
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    # removes from world so in relation to base_link instead
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # name, size, x, y, z
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=0.3)



# Funciton that picks up an object
def pick():
    rospy.Subscriber("user_input", String, exception_action)
    msg = [1]
    torso(0.5)
    head_tilt(0.5)
    gripper(0.1)

    # Determine how many objects and the chosen ones colour
    if (len(msg) == 0):
        empty = True
        colour = None
        return empty, colour
    else:
        if (len(msg) == 1):
            empty = True
        else:
            empty = False
        ID = 100
        if (ID == 100):
            colour = 'red'
            RedStorage.objects -= 1
            rospy.loginfo("Red Object")
        else:
            colour = 'blue'
            BlueStorage.objects -= 1
            rospy.loginfo("Blue Object")

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    # removes from world so in relation to base_link instead
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # name, size, x, y, z
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

    listener = tf.TransformListener()
    listener.waitForTransform("object","base_link", rospy.Time(),rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","object", rospy.Time())

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    gripper_poses = [Pose(Point(trans[0], trans[1], trans[2]+0.1),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0], trans[1], trans[2]),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    rospy.loginfo("Picking Object")

    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=0.3)

    rospy.sleep(1)
    gripper(0)
    rospy.sleep(1)
    torso(0.5)
    rospy.sleep(1)
    torso(0.75)
    head_tilt(1)

    return empty, colour
 


# Function that places an object
def place():
    rospy.Subscriber("user_input", String, exception_action)
    torso(0.5)
    head_tilt(0.5)

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    # removes from world so in relation to base_link instead
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # name, size, x, y, z
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    pose = Pose(Point(0.5,0,1),Quaternion(q[0],q[1],q[2],q[3])) 

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    rospy.loginfo("Placing Object")

    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = pose

    # Move gripper frame to the pose specified
    move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=0.3)

    rospy.sleep(1)
    gripper(0.1)
    rospy.sleep(1)
    torso(0.5)
    rospy.sleep(1)
    torso(0.75)
    head_tilt(1)
    


# Function that navigates to a location 
def navigate(table):
    rospy.Subscriber("user_input", String, exception_action)
    sc.say('I am moving to ' + table.name)

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
    goal.target_pose.pose.position =  Point(table.location[0],table.location[1],0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Moving to " + table.name)
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        return True
    else:
        return False



# Function that sends Fetch home and resets values
def reset():
    rospy.Subscriber("user_input", String, exception_action)
    torso(0.75)
    head_tilt(1)
    gripper(0.1)
    navigate(home)
    Table1.empty == False
    Table2.empty == False
    RedStorage.objects = 1
    BlueStorage.objects = 1
    sc.say('I have finished tidying')
    while True:
        rospy.sleep(1)



# Function that loops through tables to find objects
def start():
    rospy.Subscriber("user_input", String, exception_action)

    for table in Tables:
        while not table.empty: # If the table is not empty
            response = navigate(table) # navigate to table

            if response == True:
                empty, colour = pick() # pick an object up
                table.empty = empty

                if colour == 'red':
                    response = navigate(RedStorage) # if object is red go to red storage
                elif colour == 'blue':
                    response = navigate(BlueStorage) # else the object is blue therefore go to blue container                   
                else:
                    sc.say('Sorry I could not identify the object')
                    break

                if response == True:
                    place() # place the object in the container
                else:
                    sc.say('Sorry I can not reach the container, please take the object from me')
                    gripper(0.1)
                    rospy.sleep(10)
            else:
                sc.say('Sorry I can not reach the table')
                break

            if not RedStorage.objects:
                sc.say('All red objects are complete')
                rospy.sleep(1)
            if not BlueStorage.objects:
                sc.say('All blue objects are complete')
                rospy.sleep(1)

    reset()



def exception_action(data):
    if (data.data == "stop assistance"):
        rospy.loginfo("Actioning exception")
        reset()
    elif (data.data == "stop emergency"):
        rospy.loginfo("Actioning exception")
        # Shutdown rospy
        rospy.signal_shutdown("Emergency Stop Requested")



# Function that initiates start function when prompted by user
def callback(data):
    if (data.data == "start"):
        global tidy
        tidy = True
    elif (data.data == "cancel"):
        rospy.signal_shutdown("Cancel Requested")



# Fetch asks if it should start tidying
if __name__ == '__main__':
    rospy.init_node('Main_Control', disable_signals=True)

    # Wait for start/cancel signal
    rospy.Subscriber("user_input", String, callback)

    # Start Fetch in default position
    torso(0.75)
    head_tilt(1)
    gripper(0.1)
    
    sc.say('Hello my name is Fetch do you want me to start tidying?')
    rospy.sleep(1)

    while not rospy.is_shutdown():
        # Start tidying when prompted
        if tidy:
            start()   
