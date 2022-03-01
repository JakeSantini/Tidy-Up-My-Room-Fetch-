#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

def pick(msg):
    if len(msg.transforms) > 0:
        # Take the first marker we see and get its information
        marker = msg.transforms[0]
        ID = marker.fiducial_id   
        trans = marker.transform.translation
        rot = marker.transform.rotation
        #print 'Fiducial', ID
        #print 'Translation: ', trans.x, trans.y, trans.z
        #print 'Rotation: ', rot.x, rot.y, rot.z, rot.w
        br = tf.TransformBroadcaster()
        br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"object","head_camera_rgb_optical_frame")

def listener():
    rospy.init_node('marker_sub', anonymous=True)
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick)
    rospy.spin()

if __name__ == '__main__':
    listener()
