aruco_detect.launch is the launch file for marker detection, it subscribes to Fetch's camera. destinations is the main package containing my code. Within it... user_input.py will be a node that takes keyboard input and publishes it to the other nodes. tts.py is a generic script that implements text to speech and will be used inside the other scripts. stop.py is a node that stops navigation and manipulation when called. navigation.py sends Fetch to goals in the map. pick.py deals with the object manipulation, marker ID and MoveIt! marker_sub.py is a generic script to read marker info and will be used inside pick.py.