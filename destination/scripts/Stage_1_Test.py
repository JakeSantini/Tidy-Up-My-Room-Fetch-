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
from math import pi

rospy.loginfo("Fetch is starting up...")

# Initialise the sound client so Fetch can speak
sc = SoundClient(blocking=True)
rospy.sleep(1)

# Fetch starts in a not tidying, providing assistance state
tidy = False
assistance = True

# Variable for Fetch to continue when stopped
cont = False

# Scaling factor for manipulation speed
velocity_scale = 0.3

# Initialise Tables and Storage Containers
class destination:
    def __init__(self, location, name, objects=1):
        self.location = location
        self.name = name
        self.objects = objects
        self.empty = False

Table1 = destination([0.5,-0.9,-pi/4],'The First Table')
Table2 = destination([2,-2,-pi/4],'The Second Table')
Tables = [Table1]#,Table2]
RedStorage = destination([0,0,-pi/4],'The Red Storage',1)
BlueStorage = destination([0,0,-pi/4],'The Blue Storage',1)
home = destination([-1.169, 0.603,-pi/4],'My Home')



# Function that controls the direction the head/camera is looking
# 'x,y,z' are floats and are the coordinates relative to the base that Fetch will look
def head_tilt(x,y,z):
    rospy.loginfo("Fetch is looking at: " + x + ", " + y + ", " + z)
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
# 'position' is a float ranging from 0-0.1 and is the distance the gripper will open in meters
def gripper(position):
    rospy.loginfo("Fetch is opening/closing gripper to position: " + position)
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 50
    goal.command.position = position
    gripper.send_goal(goal)
    gripper.wait_for_result()



# Function that controls the torso, arm tuck and intiated collision geometry 
# 'height' is the desired height of the torso in meters, it is a float
# 'pick' is a boolean value representing if the collision geometry for picking an object should be initiated
def torso(height,pick=True):
    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    # Define collision geometry in relation to base_link
    rospy.loginfo("Setting up collision geometry")
    # Ground
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # Table
    planning_scene.removeCollisionObject("table")
    # Account for objects on table
    planning_scene.removeCollisionObject("upper_table")
    # Account for slight rotation variance in navigation
    planning_scene.removeCollisionObject("LT_rotation")
    planning_scene.removeCollisionObject("RT_rotation")
    # Wall behind table
    planning_scene.removeCollisionObject("back_wall")
    # Fetch base
    planning_scene.removeCollisionObject("base")

    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    planning_scene.addBox("back_wall", 0.1,3,3,1.3,0,1.25)
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

    # Collision geometry relevant for picking an object
    if pick:
        planning_scene.addBox("table", 1,3,0.75,0.8,0,0.4) 
        planning_scene.addBox("LT_rotation", 0.03,1.25,0.8,0.285,-0.875,0.4) 
        planning_scene.addBox("RT_rotation", 0.03,1.25,0.8,0.285,0.875,0.4) 
        planning_scene.addBox("upper_table", 1,3,0.08,0.8,0,0.8)
    rospy.sleep(1)

    # Arm tuck joint positions
    rospy.loginfo("Fetch is adjusting its arm and/or torso")
    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)



# Function that performs the identification and picking of an object
def pick():
    # Ensure gripper is open and raise torso slightly
    torso(0.1)
    gripper(0.1)

    # Search for an object
    rospy.loginfo("Fetch is searching for an object")
    angles = [0,-0.3,0.3,0]
    for angle in angles:
        head_tilt(1,angle,0.25)
        rospy.sleep(3)

        # Inform marker node to look for objects
        pick_pub.publish('pick')

        try:
            # Request marker information 
            info = rospy.wait_for_message("markers", String, 10)
            info = info.data
            num_markers = int(info[0])
            ID = int(info[2])
        except:
            # No information recieved 
            return True, 'N/A'

        # If Fetch has found a marker stop searching
        if num_markers:
            break

    # If the table is empty return status
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
        rospy.loginfo("Fetch has found a red object")
    else:
        colour = 'blue'
        BlueStorage.objects -= 1
        rospy.loginfo("Fetch has found a blue object")

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    # Get gripper to object transform
    rospy.loginfo("Fetch requested object transforms")
    listener = tf.TransformListener()
    listener.waitForTransform("object","base_link", rospy.Time(),rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","object", rospy.Time())

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)
    gripper_poses = [Pose(Point(trans[0], trans[1], trans[2]+0.1),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0], trans[1], trans[2]+0.02),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    # Move gripper frame to the object
    rospy.loginfo("Fetch starts picking the object up")
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
        planning_scene.removeCollisionObject("upper_table")
        rospy.sleep(1)
    rospy.sleep(1)

    # Close gripper on object
    gripper(0)
    rospy.loginfo("Fetch has the object")

    # Tuck arm
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = Pose(Point(trans[0], trans[1], trans[2]+0.1),Quaternion(q[0],q[1],q[2],q[3]))
    move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
    # planning_scene.attachCube("object",0.05,0,0,0,'gripper_link','gripper_link')
    rospy.sleep(1)
    torso(0.05)
    head_tilt(1,0,1)
    rospy.loginfo("The object is ready for transportation")

    return empty, colour
 


# Function that places an object
def place():
    # Intiate
    torso(0.1,False)
    head_tilt(1,0,0.5)

    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    # Place object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
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
    


# Function that allows Fetch to navigate to a location 
# 'destination' is the requested location to navigate to and is a class
def navigate(destination):
    sc.say('I am moving to ' + destination.name,'voice_us1_mbrola')
    rospy.loginfo("Fetch is moving to " + destination.name)

    # Define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server(rospy.Duration(5))

    # Set up the frame parameters
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Moving towards goal
    goal.target_pose.pose.position =  Point(destination.location[0],destination.location[1],0)
    orientation = Quaternion(*quaternion_from_euler(0,0,destination.location[2]))
    goal.target_pose.pose.orientation = orientation
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(30))

    # Return goal status
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("Fetch reached " + destination.name)
        return True
    else:
        rospy.loginfo("Fetch could not reach " + destination.name)
        return False



# Function that controls the tidying sequence
def clean():
    # Loop through tables
    for table in Tables:
        # If the table is not empty go to it
        while not table.empty:
            response1 = navigate(table)

            # Stop tidying if requested
            if not assistance:
                break

            # If table has been reached find and/or pick object up
            if response1 == True:
                empty, colour = pick()
                table.empty = empty

                # Navigate to appropriate storage container
                if colour == 'red':
                    response2 = navigate(RedStorage)
                elif colour == 'blue':
                    response2 = navigate(BlueStorage)                  
                else:
                    sc.say('I could not find any objects here','voice_us1_mbrola')
                    break

                # If storage container has been reached, place object in it
                if response2 == True:
                    place()
                # If storage container not reached, ask for someone to take the object
                else:
                    sc.say('Sorry I can not reach the container, please take the object from me','voice_us1_mbrola')
                    #rospy.sleep(1)
                    gripper(0.1)
                    sc.say('Press continue once you have taken it','voice_us1_mbrola')
                    while not cont:
                        rospy.sleep(1)
                    global cont
                    cont = False
            # If table can not be reached, try next one
            else:
                sc.say('Sorry I can not reach the table','voice_us1_mbrola')
                break

            # Update status on tidying if a set of colours is complete
            if not RedStorage.objects:
                sc.say('All red objects are complete','voice_us1_mbrola')
                rospy.sleep(1)
            if not BlueStorage.objects:
                sc.say('All blue objects are complete','voice_us1_mbrola')
                rospy.sleep(1)
        
        # Stop tidying if requested
        if not assistance:
            break

    # Tidying has completed, return to home and turn off
    navigate(home)
    sc.say('I have finished tidying','voice_us1_mbrola')
    close_pub.publish('close')
    rospy.sleep(1)
    rospy.loginfo("Fethc has finished Tidying")
    rospy.signal_shutdown("Finished Tidying")
    sys.exit(0)



# Callback function for user input node (button pressed on the GUI)
# 'data' is the information recieved on button press and is a string
def exception_action(data):
    # Tell Fetch to not tidy
    if ((data.data == "stop assistance") and (tidy == False)):
        rospy.loginfo("User chose not to start tidying")
        sc.say('I am turning off','voice_us1_mbrola')
        close_pub.publish('close')
        rospy.sleep(1)
        rospy.signal_shutdown("Cancel Requested")
        sys.exit(0)
    
    # Tell Fetch to stop tidying
    elif (data.data == "stop assistance"):
        rospy.loginfo("User requested Fetch to stop assistance")
        sc.say('I will stop helping you when I am finished with this','voice_us1_mbrola')
        rospy.sleep(1)
        global assistance
        assistance = False

    # Tell Fetch to start tidying
    elif (data.data == "start"):
        rospy.loginfo("User requested Fetch to start tidying")
        global tidy
        tidy = True

    # Tell Fetch to stop moving
    elif (data.data == "stop emergency"):
        rospy.loginfo("User clicked the emergency stop")
        sc.say('Shutting down','voice_us1_mbrola')
        rospy.sleep(1)
        rospy.signal_shutdown("Emergency Stop Requested")
        sys.exit(0)

    # Tell Fetch to continue tidying
    elif (data.data == "continue"):
        rospy.loginfo("User asked Fetch to continue")
        global cont
        cont = True



if __name__ == '__main__':
    # Setup node, publishers and subscribers
    rospy.init_node('Main_Control', disable_signals=True)
    pick_pub = rospy.Publisher('pick', String, queue_size=10)
    close_pub = rospy.Publisher('close', String, queue_size=10)
    rospy.Subscriber("user_input", String, exception_action)

    # Start Fetch in default position
    torso(0.05)
    head_tilt(1,0,1)
    gripper(0.1)
    #rospy.sleep(1)
    navigate(home)
    #rospy.sleep(1)

    # Start tidying when prompted
    sc.say('Hello my name is Fetch, let me know when you want me to start tidying','voice_us1_mbrola')
    #rospy.sleep(1)
    rospy.loginfo('Fetch is ready to start tidying')

    while not rospy.is_shutdown():
        if tidy:
            clean()
