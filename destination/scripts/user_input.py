#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from Tkinter import *
from moveit_python import MoveGroupInterface



def start():
    rospy.loginfo("Start Selected")
    pub.publish("start")



def stop_assistance():
    rospy.loginfo("Stop Assistance Selected")
    pub.publish("stop assistance")
 


def stop_emergency():
    rospy.loginfo("Emergency Stop Selected")
    pub.publish("stop emergency")



def cancel():
    rospy.loginfo("Cancel Selected")         
    pub.publish("cancel")



if __name__ == '__main__':
    # Setup node and publisher
    rospy.init_node('user_input', anonymous=True)
    pub = rospy.Publisher('user_input', String, queue_size=10)
    
    while not rospy.is_shutdown():
        
        top = Tk()

        B1 = Button(top, text ="Yes, Tidy", command = lambda : start(), height = 10, width = 50)
        B2 = Button(top, text ="No, Turn Off", command = lambda : cancel(), height = 10, width = 50)
        B3 = Button(top, text ="Stop Assistance", command = lambda : stop_assistance(), height = 10, width = 50)
        B4 = Button(top, text ="Emergency Stop", command = lambda : stop_emergency(), height = 10, width = 50)

        B1.pack(side = LEFT)
        B2.pack(side = LEFT)
        B3.pack(side = LEFT)
        B4.pack(side = LEFT)
        top.mainloop()

        #top.destroy