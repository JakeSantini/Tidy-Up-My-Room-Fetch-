#!/usr/bin/env python  
import roslib, rospy, tf

transx = 0.0167371502263
transy = 0.166919636705
transz = 0.664620084978
rotx = 0.911681889669
roty = 0.00389002223961
rotz = -0.0466705445037
rotw = 0.408219132394

if __name__ == '__main__':
    rospy.init_node('object_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((transx,transy,transz),(rotx,roty,rotz,rotw),rospy.Time.now(),"object","head_camera_link")
        rate.sleep()
