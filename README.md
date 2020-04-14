# MotorControl
Aruino Code for Motor Controller

#Added 4/14/20 cmd_vel
* topic published to by the teleop_twist_joy node for testing
* Also published to by the navigation stack

#Added 2/15/20 mcVoltageFeedback
* topic is called mcVoltageFeedback

#Added 11/16/19 BNO x Data 
* topic is called mcBNO listen on mcBNO

## DEBUGGING PROCEDURE:
* on a ubuntu install with ros and rosserial_arduino installed:
* 1.) natigate to /opt/ros/melodic/share/teleop_twist_joy/ and runlaunch teleop_twist.launch
* 4.) Start the Serial Communication with the Arduino Mega: 
  rosrun rosserial_arduino serial_node.py /dev/ttyACM0
  
 * You can echo the mcVoltageFeedback and mcBNO topics to see their output
