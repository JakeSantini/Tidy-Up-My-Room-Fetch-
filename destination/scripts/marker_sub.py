#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

def callback(msg):
    
    # Take the first marker we see and get its information
    marker = msg.transforms[0]
    ID = marker.fiducial_id   
    trans = marker.transform.translation
    rot = marker.transform.rotation
    print 'Fiducial', ID
    print 'Translation: ', trans.x, trans.y, trans.z
    print 'Rotation: ', rot.x, rot.y, rot.z, rot.w
    
"""
    t = TransformStamped()
    t.child_frame_id = "fid%d" % ID
    t.header.frame_id = msg.header.frame_id
    t.header.stamp = imageTime
    t.transform.translation.x = trans.x
    t.transform.translation.y = trans.y
    t.transform.translation.z = trans.z
    t.transform.rotation.x = rot.x
    t.transform.rotation.y = rot.y
    t.transform.rotation.z = rot.z
    t.transform.rotation.w = rot.w
    br.sendTransform(t)
"""
    
def listener():
    rospy.init_node('marker_sub', anonymous=True)
    # Subscribe to aruco_detect topics for marker to camera transforms
    rospy.Subscriber("fiducial_transforms", FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
