#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

start = True

def callback(data):
    rospy.loginfo("%s", data.data)
    rospy.Subscriber("destination_return", String, callback)

    if data.data == "success":
        print("nice")
    

# Asks for user input and then publishes request for Fetch to go to that location
def talker():
    pub = rospy.Publisher('destination', String, queue_size=10)
    rospy.init_node('dest_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    global start
    if start:
        destination_str = raw_input("Enter Location: ")
        rospy.loginfo("Publishing: " + destination_str)

        for x in range(0,50):          
            pub.publish(destination_str)
            rate.sleep()

        start = False

    rospy.Subscriber("destination_return", String, callback) # Wait for navigation success signal

if __name__ == '__main__':
    try:
        talker() # call talker function
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
