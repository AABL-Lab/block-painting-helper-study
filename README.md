# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks or do other close-quarters manipulation tasks where an extra hand would be useful

Includes ros packages:
- bph_interfaces: custom messages and actions (AI status: handwritten)
- bph_pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace (AI status: original code handwritten based on examples, modifications to make into a service with interactive help from Claude)
- nav_to_goal: a Nav2 node for moving around the room avoiding obstacles to retrieve the desired object (AI status: heavily debugged with Claude)
- bph_statemachine: a SMACH package for high-level control of the system and triggering the other nodes (AI status: initial scaffold structure created by Claude, hand-edited to add state behavior, Claude-added failure state integration with the UI)
**** Also includes bph_statemachine sm_display,
**** an entirely hand-built node with no AI used at all
**** written after I realized that the prior hand-built nodes
**** had later been debugged with AI support.
Replaces SMACH viewer (sort of) which does not work straightforwardly in Kilted


- bph_userinterface: a simple webserver for sending material requests or resolving navigation errors (AI status: Claude-generated to a specification)

- (not connected to the demo) person_finder:  a node using OpenCV and YOLO to find people in the room who might need to be avoided by either the Turtlebot or the manipulator arm (AI status: Claude-generated)



Not included in this repository but used here:
- springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace (AI status: Collaboratively generated with Claude for research, hand-edited, collaboratively debugged with Claude to add max torque generation)
https://github.com/katallen405/springcontroller

In this repository but not run here:
- turtlebot_bringup.launch.py
	This brings up a rosbridge server, the kobuki node, openi2_camera,
	static transforms for the openni_depth_optical_frame and
	openni_rgb_optical_frame, and depthimage_to_laserscan.
	It gets copied to the turtlebot and run with
	ros2 launch turtlebot_bringup.launch.py (not in a package, to disrupt
	the turtlebot ecosystem as little as possible)


# Installation details:
Clone both this package and SpringController to your workspace.

  to run the person_tracker node and rosbridge, you need a virtual environment (venv) created with --system-site-packages
  python3 -m venv --system-site-packages ~/.ros_venv
  source ~/.ros_venv/bin/activate

  Inside the venv, you need to 
  pip install ultralytics 
  pip uninstall opencv-python
  pip install "numpy<2"
  pip install roslibpy
  pip install lap

  In each terminal or in your .bashrc:
  export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
  (this allows ROS to use the venv)
  (This also goes in the launch file, but is needed for debug)

UI: now included in demo.launch:
User interface:
   ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9405 -p "qos_overrides./button.subscription.durability:=volatile" -p "qos_overrides./requestedmaterial.subscription.durability:=volatile" 

second terminal:
  python3 /home/katallen/sandbox/src/block-painting-helper/bph_userinterface/bph_ui_server.py 




######################RUNNING THE FULL STACK ##################

This demo runs on 2-3 computers, I ran it on one attached to the
camera, one laptop mounted on the turtlebot, and one machine running
the demo and webserver.  Make sure they all have the same
$ROS_DOMAIN_ID, that
$CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex><EnableTopicDiscoveryEndpoints>false</EnableTopicDiscoveryEndpoints></Discovery></Domain></CycloneDDS>'
and that $ROS_AUTOMATIC_DISCOVERY_RANGE is set to allow them to talk
to each other (I used subnet)

TURTLEBOT SETUP:
  ssh into the Turtlebot laptop from the machine
  that will be running the demo.launch script:
 - cd into wherever you copied the bringup script and run:
       ros2 launch turtlebot_bringup.launch.py

CAMERA SETUP:
  On your machine local to the camera:
    ros2 run v4l2_camera v4l2_camera_node  --ros-args -r image_raw:=/bph_overhead_camera/image_raw -r video_device:='/dev/video0'

On your static computer:
   To enable rosbridge safely across the local network:
   source ~/.ros_venv/bin/activate
   ssh -L 9090:localhost:9090 baymax@10.5.10.74 (check this IP)

Everything else:
   source ~/.ros_venv/bin/activate
   ros2 launch bph_statemachine demo.launch.py

SM viewer:
ros2 run bph_statemachine sm_display 


STATUS:
- 23 March 2026:  initial implementation 2f38d12
- 30 March 2026:  MoveIt and NavStack assignment b7e4767
- 6 March 2026:  Perception Assignment 19c79af (including venv instructions)
- 22 April 2026: SMACH assignment 9d1622c

