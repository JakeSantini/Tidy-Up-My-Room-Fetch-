#!/usr/bin/env python  
import roslib, rospy, tf
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

def callback(msg):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    trans = msg.transform.translation
    rot = msg.transform.rotation

    # Broadcast for 10 seconds
    for x in range(0,100):
        br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('object_tf_broadcaster')
    rospy.Subscriber("marker_transorm", FiducialTransform, callback)
    rospy.spin()
