aruco_detect.launch is the launch file for marker detection. This is where the marker dictionary and camera information are stored 

user_input.py is a node that sets up a GUI and publishes input data to other nodes

stop.py is a node that stops navigation and manipulation when called

pick.py is generic test code that deals with the object manipulation, marker ID and MoveIt!

object_tf_broadcast.py is the node that retrieves the object transorms and data from aruco detect and broadcasts this to the main controllers (test symobolises the simulation version)

Stage_1 is the main control that implements stage 1 of the experiment (test symobolises the simulation version)

Stage_2 is the main control that implements stage 2 of the experiment (test symobolises the simulation version)

CMakeLists and package.xml are the setup scripts for ROS to run everything together
