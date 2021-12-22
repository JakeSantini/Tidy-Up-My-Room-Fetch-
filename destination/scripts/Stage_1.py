#!/usr/bin/env python
import rospy, sys, actionlib, tf
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from tf.transformations import quaternion_from_euler

# Initialise the sound client so Fetch can speak
sc = SoundClient()
rospy.sleep(1)

# Initialise Tables and Storage Containers
class table:
    def _init_(self, location):
        self.location = location
        self.empty = False

T1 = table([1,1])
T2 = table([2,2])
Tables = [T1,T2]
RS = table([3,3])
BS = table([4,4])
home = table([0,0])


# Function that controls the head tilt
def head_tilt(height):
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
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 50
    goal.command.position = position
    gripper.send_goal(goal)
    gripper.wait_for_result()


# Function that controls the torso and arm tuck
def torso(height):
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground") # removes from world so in relation to base_link instead
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0) # name, size, x, y, z
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.32,0.56,0.73,0.13,0,0)

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True)


def pick(msg):
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
        marker = msg.transforms[0]
        ID = marker.fiducial_id
        if (ID == 100) or (ID == 101):
            colour = 'red'
        else:
            colour = 'blue'

    pub = rospy.Publisher('marker_transorm', object)
    rate = rospy.Rate(10) # 10hz

    # Publish command for 1 seconds
    for x in range(0,10):          
        pub.publish(marker)
        rate.sleep()

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.32,0.56,0.73,0.13,0,0)

    listener = tf.TransformListener()
    listener.waitForTransform("object","base_link", rospy.Time(),rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","object", rospy.Time())

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    gripper_poses = [Pose(Point(trans[0], trans[1], trans[2]+0.1),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0], trans[1], trans[2]),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link')

    rospy.sleep(1)
    gripper(0)
    rospy.sleep(1)
    torso(0.5)
    rospy.sleep(1)
    torso(0.05)
    head_tilt(1)

    return empty, colour
 

# Subscribe to aruco_detect topics for marker to camera transforms
def pick_listener():
    torso(0.5)
    head_tilt(0.5)
    gripper(0.1)
    empty, colour = rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick)
    return empty, colour


def place():
    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Define ground plane to avoid collisions
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addCube("table", 1, 1, 0, 0)
    planning_scene.addBox("base", 0.32,0.56,0.73,0.13,0,0)

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    pose = [Pose(Point(1,0,1.5),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = pose

    # Move gripper frame to the pose specified
    move_group.moveToPose(gripper_pose_stamped, 'gripper_link')

    rospy.sleep(1)
    gripper(0.1)
    rospy.sleep(1)
    torso(0.5)
    rospy.sleep(1)
    torso(0.05)
    head_tilt(1)
    

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
    for table in Tables:
        while not table.empty: # If the table is not empty
            navigate = navigate(table.location) # navigate to table
            if navigate == True:
                empty, colour = pick_listener() # pick an object up
                table.empty = empty
                if colour == 'red':
                    navigate = navigate(RS.location) # if object is red go to red storage
                elif colour == 'blue':
                    navigate = navigate(BS.location) # else the object is blue therefore go to blue container
                else:
                    break
                if navigate == True:
                    place() # place the object in the container
                else:
                    sc.say('Sorry I can not reach the container, please take the object from me')
                    gripper(0.1)
                    rospy.sleep(10)
            else:
                sc.say('Sorry I can not reach the table')
                break

    navigate(home.location)
    torso(0.05)
    head_tilt(1)
    gripper(0.1)
    T1.empty == False
    T2.empty == False
    sc.say('I have finished tidying let me know when you want me to start again')


# Fetch asks if it should start tidying
if __name__ == '__main__':
    rospy.init_node('Main_Control')
    # Start Fetch in default position
    torso(0.05)
    head_tilt(1)
    gripper(0.1)
    sc.say('Hello my name is Fetch do you want me to start tidying?')
  
    while True:    
        input_str = raw_input("Shall I Tidy? (yes/no/quit): ")    
        if input_str == 'yes':
            sc.say('I will begin tidying now')
            start() # start tidying
        elif input_str == 'quit':
            break # maybe go home?
        else:
            sc.say('Ok let me know when you want me to start')           
