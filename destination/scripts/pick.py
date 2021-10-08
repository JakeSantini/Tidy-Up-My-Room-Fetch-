#!/usr/bin/env python
import rospy, sys, actionlib, tf
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

# Function that tilts the head to look down
def head_tilt():
    head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1
    goal.target.point.y = 0
    goal.target.point.z = 0.5
    head_client.send_goal(goal)
    head_client.wait_for_result()

# Function that resets the head tilt
def head_reset():
    head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = 1
    goal.target.point.y = 0
    goal.target.point.z = 1
    head_client.send_goal(goal)
    head_client.wait_for_result()

# Function that closes the gripper
def close_gripper():
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 10
    goal.command.position = 0
    gripper.send_goal(goal)
    gripper.wait_for_result()

# Function that opens the gripper
def open_gripper():
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 10
    goal.command.position = 0.1
    gripper.send_goal(goal)
    gripper.wait_for_result()

# Take the first marker we see and get its information
def marker_callback(msg):
    marker = msg.transforms[0]
    ID = marker.fiducial_id   
    trans = marker.transform.translation
    rot = marker.transform.rotation
    #print 'Fiducial', ID
    #print 'Translation: ', trans.x, trans.y, trans.z
    #print 'Rotation: ', rot.x, rot.y, rot.z, rot.w

    
    

# Subscribe to aruco_detect topics for marker to camera transforms
def marker_listener():
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, marker_callback)
    rospy.spin()    






if __name__ == '__main__':
    rospy.init_node("pick")
    open_gripper()
    head_tilt()
    marker_listener()
# pick up object + move to arm home position
    head_reset()
#navigation
#place

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

    # This is the wrist link not the gripper itself
    #gripper_frame = 'wrist_roll_link'
    gripper_frame = 'gripper_link'

    # Position and rotation of poses
    #gripper_poses = [Pose(Point(trans.x, trans.y, trans.z + 0.1),Quaternion(rot.x, rot.y, rot.z, rot.w)),pose(Point(trans.x, trans.y, trans.z),Quaternion(rot.x, rot.y, rot.z, rot.w))]
    gripper_poses = [Pose(Point(0.5, 0.5, 1),Quaternion(0,0,0,0.1)),Pose(Point(0.5, 0.5, 1.1),Quaternion(0,0,0,0.1))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()

        if result:
	    # Checking the MoveItErrorCode
	    if result.error_code.val == MoveItErrorCodes.SUCCESS:
	        rospy.loginfo("Success")
	    else:
	    # If you get to this point please search for:
	    # moveit_msgs/MoveItErrorCodes.msg
	        rospy.logerr("Arm goal in state: %s",
	                     move_group.get_move_action().get_state())
        else:
	    rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        move_group.get_move_action().cancel_all_goals()
