# MotorControl
Aruino Code for Motor Controller

##Motor Control is done over two topics:
###mcSpeed takes a std_msgs/String and holds a value of "0" to "255"
###mcDirection takes a std_msgs/String and holds a the following values:
  "1000" -> Move Left
  "0100" -> Move Right
  "0010" -> Drive Forward
  "0001" -> Drive Backward

Examples: "0100" 255 would mean turn right at 100% movement speed,
          "0001" 122 would mean drive backward at 50% movement speed

##DEBUGGING PROCEDURE:
on a ubuntu install with ros and rosserial_arduino installed:
1.) run roscore in terminal
2.) open second terminal, and type 
  rosrun rosserial_python serial_node.py /dev/ttyACM0 <- note this COM port can be optained through Arduino IDE
3.) Create a listener to validate the responses. The topics are: mcSpeedFeedback and mcDirection Feedback. The commands are either:
  rostopic echo mcSpeedFeedback
  rostopic echo mcDirectionFeedback, depending on what data you wish to validate
4.) Publish some data to see the program run. The command is :
  rostopic pub mcSpeed std_msgs/String \"255\" for direction and 
  rostopic pub mcDirection std_msgs/String \"0010\" for sleed controll <- NOTE: The backslashes are neccesary as an escape sequence in order for the terminal to properly parse the string
  
  You should now be able to see the response in the echo terminal. In total, you need four terminals to debug this program
