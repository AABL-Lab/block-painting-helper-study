# block-painting-helper
A ros2 multimodal system for helping the user paint on wooden blocks or do other close-quarters manipulation tasks where an extra hand would be useful

Includes ros packages:
- bph_interfaces: custom messages and actions (AI status: handwritten)
- bph_pickmeup:  a MoveIt package for picking up an object and moving it into the user's workspace (AI status: original code handwritten based on examples, modifications to make into a service with interactive help from Claude)
- bph_statemachine: a SMACH package for high-level control of the system and triggering the other nodes (AI status: initial scaffold structure created by Claude, hand-edited to add state behavior, Claude-added failure state integration with the UI)
- bph_userinterface: a simple webserver for sending material requests or resolving navigation errors (AI status: Claude-generated to a specification)
-  person_finder:  a node using OpenCV and YOLO to find people in the room who might need to be avoided (does not localize them into the map yet)

Not included in this repository but used here:
- springcontroller: a platform-independent ROS node for torque control of a robot arm using virtual springs for constraints on the arm's position in the user's workspace (AI status: Collaboratively generated with Claude for research, hand-edited, collaboratively debugged with Claude to add max torque generation)
https://github.com/katallen405/springcontroller

# Installation details:
Clone both this package and SpringController to your workspace.

To run the person_tracker node and rosbridge and the springcontroller visualization, you need a virtual environment (venv) created with --system-site-packages
  python3 -m venv --system-site-packages ~/.springcontroller_venv
  source ~/.springcontroller_venv/bin/activate

  Inside the venv, you need to 
  pip install ultralytics 
  pip uninstall opencv-python
  pip install "numpy<2"
  pip install roslibpy
  pip install lap
  pip install meshcat

  In each terminal or in your .bashrc:
  export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
  (this allows ROS to use the venv)
  (This also goes in the launch file, but is needed for debug)


User interface:
   ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9405 -p "qos_overrides./button.subscription.durability:=volatile" -p "qos_overrides./requestedmaterial.subscription.durability:=volatile" 

second terminal:
  python3 /home/katallen/sandbox/src/block-painting-helper/bph_userinterface/bph_ui_server.


######################RUNNING THE FULL STACK ##################

This demo runs on 2-3 computers, I ran it on one attached to the
camera, one laptop mounted on the turtlebot, and one machine running
the demo and webserver.  Make sure they all have the same
$ROS_DOMAIN_ID, that
$CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex><EnableTopicDiscoveryEndpoints>false</EnableTopicDiscoveryEndpoints></Discovery></Domain></CycloneDDS>'
and that $ROS_AUTOMATIC_DISCOVERY_RANGE is set to allow them to talk
to each other (I used subnet)

On your static computer:
   To enable rosbridge safely across the local network:
   ssh -L 9090:localhost:9090 baymax@10.5.10.74 (check this IP)

Everything else:
   source ~/.springcontroller_venv/bin/activate
   ros2 launch bph_statemachine demo.launch.py

SM viewer:
source ~/.springcontroller_venv/bin/activate
ros2 run bph_statemachine sm_display 


STATUS:
- 14 May 2026: forked from class demo repo to prepare for study

