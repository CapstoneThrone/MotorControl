#include<ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Twist.h>

ros::NodeHandle nh;

uint8_t count = 1;  //for debugging, count and prev count assure new data are only repeated once to RPI4 over ROSSERIAL
uint8_t prevCount = 0;

String data_str;
String speed_data_str;
String direction_data_str;

//geometry_msgs::Twist mcDirectionFeedback; //Data sent to RPi4
std_msgs::String mcDirectionFeedback;  //use geometry_twist for final code, string is for debugging
std_msgs::String mcSpeedFeedback;

ros::Publisher pubDirection("mcDirectionFeedback", &mcDirectionFeedback);
ros::Publisher pubSpeed("mcSpeedFeedback", &mcSpeedFeedback);

 void doDirectionControll( const std_msgs::String &mcSubDirection) {
  count++;
  data_str = mcSubDirection.data;
  direction_data_str = data_str.substring(0,4);
  speed_data_str = data_str.substring(4,8);
  
  if (direction_data_str == "1000") {
    //turn left
    mcDirectionFeedback.data = "turn left";
  } else if (direction_data_str == "0100"){
    //turn right
    mcDirectionFeedback.data = "turn right";
  } else if (direction_data_str == "0010"){
    //drive forward
    mcDirectionFeedback.data = "drive forward";
  } else if (direction_data_str == "0001") {
    //drive backward
    mcDirectionFeedback.data = "drive backward";
  } else {
    //send error code to RPI4, bad data
    mcDirectionFeedback.data = "Error, bad data";
  }
}

void doSpeedControll(const std_msgs::String mcSubSpeed) {
  count++;
  mcSpeedFeedback.data = mcSubSpeed.data;
}
 
 ros::Subscriber<std_msgs::String> mcSubDirection("mcDirection", &doDirectionControll);
 ros::Subscriber<std_msgs::String> mcSubSpeed("mcSpeed", &doSpeedControll);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pubDirection);
  nh.advertise(pubSpeed);
  nh.subscribe(mcSubDirection);
  nh.subscribe(mcSubSpeed);
}

void loop() {
  if (count > prevCount) {
    pubDirection.publish(&mcDirectionFeedback);
    pubSpeed.publish(&mcSpeedFeedback);
    prevCount = count;
  }
  nh.spinOnce();
  delay(1000);
}
