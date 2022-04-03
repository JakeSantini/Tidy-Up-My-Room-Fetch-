#!/usr/bin/env python
import rospy, tf
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray

pick_status = False

transx = 0.00714426165428
transy = 0.204051632032
transz = 0.55055134137
rotx = 0.00188216955915
roty = 0.864120680408
rotz = -0.498701155419
rotw = -0.0677426358516



def pick(msg):
    for x in range(5):
        if pick_status:
            markers = str(len(msg.transforms))
            rospy.loginfo("Detected " + markers + " objects")
            rate = rospy.Rate(10.0)
            
            if len(msg.transforms) > 0:
                # Take the first marker we see and get its information
                marker = msg.transforms[0]   
                trans = marker.transform.translation
                rot = marker.transform.rotation

                # Publish ID
                ID = str(marker.fiducial_id)
                rospy.loginfo("Broadcasting transforms for ID " + ID)

                # Broadcast for 5 seconds
                br = tf.TransformBroadcaster()

                for x in range(0,50):
                    markers_pub.publish(markers + " " + ID)
                    br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
                    rate.sleep()
                break
            elif x == 4:
                #for x in range(0,50):
                markers_pub.publish(markers + " 0")
                rate.sleep()
    global pick_status
    pick_status = False



def callback(msg):
    global pick_status
    pick_status = True



def pick_sim(msg):
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()

    for x in range(0,100):
        markers_pub.publish("1 1")
        br.sendTransform((transx,transy,transz),(rotx,roty,rotz,rotw),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('marker_sub', anonymous=True)
    markers_pub = rospy.Publisher('markers', String, queue_size=10)

    while not rospy.is_shutdown():
        #rospy.Subscriber('pick', String, callback)
        #rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick)
        rospy.Subscriber('pick', String, pick_sim)  
        rospy.sleep(0.1) 
