#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from Tkinter import *
from moveit_python import MoveGroupInterface

"""
# Function that asks for user input and then publishes request
def talker():
    # Setup node and publisher
    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('user_input', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Ask for user input
    input_str = raw_input("Enter Command: ")
    rospy.loginfo("Publishing: " + input_str)

    # Publish command for 1 seconds
    for x in range(0,10):          
        pub.publish(input_str)
        rate.sleep()
    input_str = " "
    talker()

if __name__ == '__main__':
    try:
        talker() # call talker function
        rospy.spin() # Repeat

    except rospy.ROSInterruptException:
        pass
"""
def start():
    rospy.loginfo("Start Selected")
    pub.publish("start")

def stop_assistance():
    rospy.loginfo("Stop Assistance Selected")
    rate = rospy.Rate(10) # 10hz
    # Publish command for 5 seconds
    for x in range(0,50):          
        pub.publish("stop assistance")
        rate.sleep()
 

def stop_emergency():
    rospy.loginfo("Emergency Stop Selected")
    rate = rospy.Rate(100) # 10hz
    # Publish command for 5 seconds
    for x in range(0,50):          
        pub.publish("stop emergency")
        rate.sleep()

def cancel():
    rospy.loginfo("Cancel Selected")
    rate = rospy.Rate(10) # 10hz
    # Publish command for 5 seconds
    for x in range(0,5):          
        pub.publish("cancel")
        rate.sleep()


if __name__ == '__main__':
    # Setup node and publisher
    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('user_input', anonymous=True)
    
    top = Tk()

    B1 = Button(top, text ="Start Tidying", command = lambda : start(), height = 10, width = 50)
    B2 = Button(top, text ="Cancel", command = lambda : cancel(), height = 10, width = 50)
    B3 = Button(top, text ="Stop Assistance", command = lambda : stop_assistance(), height = 10, width = 50)
    B4 = Button(top, text ="Emergency Stop", command = lambda : stop_emergency(), height = 10, width = 50)

    B1.pack(side = LEFT)
    B2.pack(side = LEFT)
    B3.pack(side = LEFT)
    B4.pack(side = LEFT)
    top.mainloop()

    #top.destroy