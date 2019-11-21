# MotorControl
Aruino Code for Motor Controller

#Added 11/16/19 BNO x Data 
Topic is called mcBNO listen on mcBNO, broadcasts on 10ms delay

## Motor Control is done over two topics:
### mcSpeed takes a std_msgs/Int32 and holds a value of 0 to 255
### mcDirection takes a std_msgs/Int32 and holds a the following values:
  1 -> Move Left
  2 -> Move Right
  3 -> Drive Forward
  4 -> Drive Backward
  0 -> Stop

## DEBUGGING PROCEDURE:
* on a ubuntu install with ros and rosserial_arduino installed:
* 1.) run roscore in terminal
* 2.) open second terminal, and type 
  rosrun rosserial_python serial_node.py /dev/ttyACM0 <- note this COM port can be optained through Arduino IDE
* 3.) Create a listener to validate the responses. The topics are: mcSpeedFeedback and mcDirection Feedback. The commands are either:
  rostopic echo mcSpeedFeedback
  rostopic echo mcDirectionFeedback, depending on what data you wish to validate
* 4.) Publish some data to see the program run. The command is :
  rostopic pub mcSpeed std_msgs/Int32 255 for direction and 
  rostopic pub mcDirection std_msgs/Int32 1 for sleed controll <- NOTE: The backslashes are neccesary as an escape sequence in order for the terminal to properly parse the string
  
 * You should now be able to see the response in the echo terminal. In total, you need four terminals to debug this program
