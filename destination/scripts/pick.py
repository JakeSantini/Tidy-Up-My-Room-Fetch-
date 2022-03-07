#!/usr/bin/env python
import rospy, sys, actionlib, tf
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from tf.transformations import quaternion_from_euler

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
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True)

# Function to pickup an object
def pick():
    torso(0.5)
    head_tilt(0.5)
    gripper(0.1)
#def pick(msg):
    # Get tf information
    #marker = msg.transforms[0]
    #ID = marker.fiducial_id   
    #trans = marker.transform.translation
    #rot = marker.transform.rotation
    #print 'Fiducial', ID
    #print 'Translation: ', trans.x, trans.y, trans.z
    #print 'Rotation: ', rot.x, rot.y, rot.z, rot.w

    # transx = 0.00714426165428
    # transy = 0.204051632032
    # transz = 0.55055134137
    # rotx = 0.00188216955915
    # roty = 0.864120680408
    # rotz = -0.498701155419
    # rotw = -0.0677426358516
    #br = tf.TransformBroadcaster()
    #br.sendTransform((transx,transy,transz),(rotx,roty,rotz,rotw),rospy.Time.now(),"object","head_camera_link")
     

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
    planning_scene.addBox("base", 0.33,0.57,0.76,0.13,0,0)

 
    listener = tf.TransformListener()
    listener.waitForTransform("object","base_link", rospy.Time(),rospy.Duration(1))
    (trans_head, rot_head) = listener.lookupTransform("base_link","object", rospy.Time())

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,0)

    #gripper_poses = [Pose(Point(trans_head[0], trans_head[1], trans_head[2]),Quaternion(rot_head[0],rot_head[1],rot_head[2],rot_head[3]))]
    #gripper_poses = [Pose(Point(trans_head[0], trans_head[1], trans_head[2]),Quaternion(0,0,0,1))]  
    gripper_poses = [Pose(Point(trans_head[0], trans_head[1], trans_head[2]+0.1),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans_head[0], trans_head[1], trans_head[2]),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    rospy.loginfo("Picking Object")

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
    rospy.sleep(1)
    gripper(0.1)

# Only calls the pick function once an object has been detected
def pick_callback(msg):

   if len(msg.transforms) > 0:
       counter = 0
       pick(msg)
   elif counter < 50:
       counter += 1
       pick_listener()
   else:
       print("No marker found")
   counter = 0        


# Subscribe to aruco_detect topics for marker to camera transforms
def pick_listener():
    torso(0.5)
    head_tilt(0.5)
    gripper(0.1)
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick_callback)    


if __name__ == '__main__':
    rospy.init_node("pick")
    pick()
