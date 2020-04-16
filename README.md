# MotorControl
Aruino Code for Motor Controller

#Added 4/14/20 cmd_vel
* topic published to by the teleop_twist_joy node for testing
* Also published to by the navigation stack on the RPi
* Sketch processes the geometry_msgs/Twist command to execute movement instructions from RPi

#Added 2/15/20 mcVoltageFeedback
* topic is called mcVoltageFeedback

#Added 11/16/19 BNO x Data
* topic is called mcBNO listen on mcBNO

##Make sure this sketch is running on the MEGA. The RPi is configured to launch the required nodes to get this sketch working

##If doing development work, the alternate process below can be used to run the arduino using the rosserial_arduino package from a PC.

##Running from a PC for Debugging purposes procedure:

##Run the provided start.sh shell script
##OR do the following manual procedure
* on a ubuntu install with ros and rosserial_arduino installed:
* 1.) natigate to /opt/ros/melodic/share/teleop_twist_joy/ and runlaunch teleop_twist.launch
** Note: This step requires that you first insteall the teleop_twist_joy package found here: https://wiki.ros.org/teleop_twist_joy
** Note: Also required is the joy package found here https://wiki.ros.org/joy

* 2.) Start the Serial Communication with the Arduino Mega: 
  rosrun rosserial_arduino serial_node.py /dev/ttyACM0
  
 * You can echo the mcVoltageFeedback and mcBNO topics to see their output
 ** "rostopic echo mcBNO"
 * These topics publishonce every PERIOD
