#!/usr/bin/env bash
gnome-terminal -e "roslaunch /opt/ros/melodic/share/teleop_twist_joy/launch/teleop.launch" #Run a launchfile to start the roscore, joy and teleop_twist_joy nodes
gnome-terminal -e "rosrun rosserial_arduino serial_node.py /dev/ttyACM0" #Start the serial connection between the PC and Arduino