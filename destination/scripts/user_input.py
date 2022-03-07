#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from Tkinter import *


# Function that signals main node to 'start'
def yes():
    rospy.loginfo("Yes Selected")
    pub.publish("start")


# Function that signals main node to 'cancel'
def no():
    rospy.loginfo("No Selected")         
    pub.publish("cancel")


# Function that signals nodes to 'stop assistance'
def stop_assistance():
    rospy.loginfo("Stop Assistance Selected")
    pub.publish("stop assistance")
 

# Function that signals nodes to 'emergency stop'
def stop_emergency():
    rospy.loginfo("Emergency Stop Selected")
    pub.publish("stop emergency")


# Setup node and publisher
rospy.init_node('user_input', anonymous=True)
pub = rospy.Publisher('user_input', String, queue_size=10)

# Tkinter Loop
top = Tk()

#Create 2x2 grid of buttons
Grid.rowconfigure(top, 0, weight=1)
Grid.columnconfigure(top, 0, weight=1)

frame=Frame(top)
frame.grid(row=0, column=0, sticky=N+S+E+W)

btn_names = ['Yes','No','Stop Assistance','Emergency Stop']
btn_commands = [lambda : yes(),lambda : no(),lambda : stop_assistance(),lambda : stop_emergency()]
index = 0

for row in range(2):
    Grid.rowconfigure(frame, row, weight=1)
    for col in range(2):
        Grid.columnconfigure(frame, col, weight=1)
        btn = Button(frame, background='white', height = 20, width = 100, text = btn_names[index], command = btn_commands[index])
        btn.grid(row=row, column=col, sticky=N+S+E+W)
        index += 1

top.mainloop()
    