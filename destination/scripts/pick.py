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


# Function that raises the torso when the arm is tucked
def raise_torso():
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
    planning_scene.addCube("table", 1, 1, 0, 0) # 50cm away, 50cm high

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [0.5, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True)


# Function that lowers the torso when the arm is tucked
def lower_torso():
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
 
    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [0.01, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True)


# Function to pickup an object
#def pick_callback(msg):
def pick():
    # Get tf information
    #marker = msg.transforms[0]
    #ID = marker.fiducial_id   
    #trans = marker.transform.translation
    #rot = marker.transform.rotation
    #print 'Fiducial', ID
    #print 'Translation: ', trans.x, trans.y, trans.z
    #print 'Rotation: ', rot.x, rot.y, rot.z, rot.w

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

    listener = tf.TransformListener()
    listener.waitForTransform("base_link","head_camera_link", rospy.Time(),rospy.Duration(1))
    (trans_head, rot_head) = listener.lookupTransform("base_link","head_camera_link",rospy.Time(0))

    # Position and rotation of poses
    #gripper_poses = [Pose(Point(trans.x + trans_head[0], trans.y + trans_head[1], trans.z + trans_head[2]),Quaternion(rot.x + rot_head[0], rot.y + rot_head[1], rot.z + rot_head[2], rot.w + rot_head[3])),Pose(Point(trans.x + trans_head[0], trans.y + trans_head[1], trans.z + trans_head[2]),Quaternion(rot.x + rot_head[0], rot.y + rot_head[1], rot.z + rot_head[2], rot.w + rot_head[3]))]


    gripper_poses = [Pose(Point(0 + trans_head[0], 0 + trans_head[1], 0.5 + trans_head[2]),Quaternion(0,0,0,0.1)),Pose(Point(0.5, 0.5, 1.1),Quaternion(0,0,0,0.1))]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link')

    rospy.sleep(1)
    close_gripper()
    raise_torso()
    lower_torso()
    head_reset()


# Subscribe to aruco_detect topics for marker to camera transforms
def pick_listener():
    raise_torso()
    head_tilt()
    open_gripper()
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick_callback)
    rospy.spin()    

def test():
    listener = tf.TransformListener()
    listener.waitForTransform("base_link","head_camera_link", rospy.Time(),rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","head_camera_link",rospy.Time(0))
    print trans[2]


if __name__ == '__main__':
    rospy.init_node("pick")
    #test()
    #pick_listener()
    raise_torso()
    head_tilt()
    open_gripper()
    pick()







