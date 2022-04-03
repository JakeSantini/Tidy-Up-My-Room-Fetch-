#!/usr/bin/env python
from logging import INFO
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
assistance = True
cont = False

velocity_scale = 1

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
def head_tilt(x,y,z):
    head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
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
def torso(height,pick=True):

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    # Define ground plane to avoid collisions
    # removes from world so in relation to base_link instead
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.removeCollisionObject("table")
    planning_scene.removeCollisionObject("upper_table")
    planning_scene.removeCollisionObject("base")

    # name, size, x, y, z
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    if pick:
        planning_scene.addBox("table", 1,2,0.75,0.85,0,0.4) #2.5cm higher than table
        planning_scene.addBox("upper_table", 1,2,0.05,0.85,0,0.8) # 7.5cm higher than table (don't hit objects)
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)
    rospy.sleep(0.5)

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)



# Function that picks up an object
def pick():

    torso(0.1)
    gripper(0.1)

    # Search for an object
    angles = [0,-0.3,0.3,0]
    for angle in angles:
        head_tilt(1,angle,0.25)
        rospy.sleep(3)
        pick_pub.publish('pick')
        try:
            info = rospy.wait_for_message("markers", String, 10)
            info = info.data
            num_markers = int(info[0])
            ID = int(info[2])
        except:
            return True, 'N/A'
        if num_markers:
            break

    # Determine how many objects and if the table is empty
    if (num_markers == 0):
        gripper(0.1)
        torso(0.05)
        head_tilt(1,0,1)
        return True, 'N/A'
    # Removed because it may have not seen others
    #elif (num_markers == 1):
    #    empty = True               
    else:
        empty = False

    if (ID == 2):
        colour = 'red'
        RedStorage.objects -= 1
        rospy.loginfo("Red Object")
    else:
        colour = 'blue'
        BlueStorage.objects -= 1
        rospy.loginfo("Blue Object")

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    # Get gripper to object transform
    listener = tf.TransformListener()
    listener.waitForTransform("object","base_link", rospy.Time(),rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","object", rospy.Time())

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    gripper_poses = [Pose(Point(trans[0], trans[1], trans[2]+0.4),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0], trans[1], trans[2]+0.3),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0], trans[1], trans[2]+0.4),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    rospy.loginfo("Picking Object")

    # Move gripper frame to the poses specified
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
        planning_scene.removeCollisionObject("upper_table")

    rospy.sleep(1)
    gripper(0)
    # planning_scene.attachCube("object",0.05,0,0,0,'gripper_link','gripper_link')
    rospy.sleep(1)
    torso(0.05)
    head_tilt(1,0,1)

    return empty, colour
 


# Function that places an object
def place():
    torso(0.1,False)
    head_tilt(1,0,0.5)

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

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
    move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)

    rospy.sleep(1)
    gripper(0.1)
    #planning_scene.removeAttachedObject("object")
    #planning_scene.removeCollisionObject("object")
    rospy.sleep(1)
    torso(0.05,False)
    rospy.sleep(1)
    head_tilt(1,0,1)
    


# Function that navigates to a location 
def navigate(table):
    sc.say('I am moving to ' + table.name,'voice_us1_mbrola')

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



# Function that loops through tables to find objects
def clean():

    for table in Tables:
        while not table.empty: # If the table is not empty
            response1 = navigate(table) # navigate to table
            if not assistance:
                break

            if response1 == True:
                empty, colour = pick() # pick an object up
                table.empty = empty

                if colour == 'red':
                    response2 = navigate(RedStorage) # if object is red go to red storage
                elif colour == 'blue':
                    response2 = navigate(BlueStorage) # else the object is blue therefore go to blue container                   
                else:
                    sc.say('Sorry I could not identify the object','voice_us1_mbrola')
                    break

                if response2 == True:
                    place() # place the object in the container
                else:
                    sc.say('Sorry I can not reach the container, please take the object from me','voice_us1_mbrola')
                    rospy.sleep(1)
                    gripper(0.1)
                    sc.say('Press continue once you have taken it','voice_us1_mbrola')
                    while not cont:
                        rospy.sleep(1)
                    global cont
                    cont = False
            else:
                sc.say('Sorry I can not reach the table','voice_us1_mbrola')
                break

            if not RedStorage.objects:
                sc.say('All red objects are complete','voice_us1_mbrola')
                rospy.sleep(1)
            if not BlueStorage.objects:
                sc.say('All blue objects are complete','voice_us1_mbrola')
                rospy.sleep(1)
            
        if not assistance:
            break

    navigate(home)
    sc.say('I have finished tidying','voice_us1_mbrola')
    close_pub.publish('close')
    rospy.sleep(1)
    rospy.loginfo("Finished Tidying")
    rospy.signal_shutdown("Finished Tidying")
    sys.exit(0)



def exception_action(data):
    if ((data.data == "stop assistance") and (tidy == False)):
        rospy.loginfo("Cancelling")
        sc.say('I am turning off','voice_us1_mbrola')
        close_pub.publish('close')
        rospy.sleep(1)
        rospy.signal_shutdown("Cancel Requested")
        sys.exit(0)
    
    elif (data.data == "stop assistance"):
        rospy.loginfo("Actioning Stop Assistance")
        sc.say('I will stop helping you when I am finished with this','voice_us1_mbrola')
        rospy.sleep(1)
        global assistance
        assistance = False

    elif (data.data == "start"):
        rospy.loginfo("Starting")
        global tidy
        tidy = True

    elif (data.data == "stop emergency"):
        rospy.loginfo("Actioning Emergency Stop")
        sc.say('Shutting down','voice_us1_mbrola')
        rospy.sleep(1)
        rospy.signal_shutdown("Emergency Stop Requested")
        sys.exit(0)

    elif (data.data == "continue"):
        rospy.loginfo("Continue Requested")
        global cont
        cont = True



# Fetch asks if it should start tidying
if __name__ == '__main__':
    rospy.init_node('Main_Control', disable_signals=True)
    pick_pub = rospy.Publisher('pick', String, queue_size=10)
    close_pub = rospy.Publisher('close', String, queue_size=10)
    rospy.Subscriber("user_input", String, exception_action)

    # Start Fetch in default position
    torso(0.05)
    head_tilt(1,0,1)
    gripper(0.1)

    sc.say('Hello my name is Fetch, let me know when you want me to start tidying','voice_us1_mbrola')
    rospy.sleep(1)

    while not rospy.is_shutdown():
        # Start tidying when prompted
        if tidy:
            clean()
