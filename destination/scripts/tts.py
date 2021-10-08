import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from sound_play.libsoundplay import SoundClient

def talk():
    sc = SoundClient()
    path_to_sounds = "/home/jake/jsan_ws/src/audio_common/sound_play/sounds/"
    rospy.init_node('tts', anonymous=False)
    sc.playWave(path_to_sounds+"say-beep.wav")
    #sc.say('Hello World')

if __name__ == '__main__':
    try:
        talk() # call talk function
        rospy.spin() # Repeat

    except rospy.ROSInterruptException:
        pass
