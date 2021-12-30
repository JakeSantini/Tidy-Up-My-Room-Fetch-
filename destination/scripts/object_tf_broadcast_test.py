#!/usr/bin/env python  
import roslib, rospy, tf

transx = 0.00714426165428
transy = 0.204051632032
transz = 0.55055134137
rotx = 0.00188216955915
roty = 0.864120680408
rotz = -0.498701155419
rotw = -0.0677426358516

if __name__ == '__main__':
    rospy.init_node('object_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((transx,transy,transz),(rotx,roty,rotz,rotw),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
        rate.sleep()
