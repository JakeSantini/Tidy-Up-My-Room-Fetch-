#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

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
