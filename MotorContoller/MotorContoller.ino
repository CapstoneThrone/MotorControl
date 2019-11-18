#include<ros.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/Twist.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
ros::NodeHandle nh;

//used for detecting new event
uint8_t count = 1;
uint8_t prevCount = 0;

//geometry_msgs::Twist mcDirectionFeedback; //Data sent to RPi4
std_msgs::Int32 mcDirectionFeedback;  //use geometry_twist for final code, string is for debugging
std_msgs::Int32 mcSpeedFeedback;
std_msgs::Float32 mcBNOFeedback;

ros::Publisher pubDirection("mcDirectionFeedback", &mcDirectionFeedback);
ros::Publisher pubSpeed("mcSpeedFeedback", &mcSpeedFeedback);
ros::Publisher pubBNO("mcBNO", &mcBNOFeedback);

void doDirectionControll( const std_msgs::Int32 &mcSubDirection) {
  count++;
  switch(mcSubDirection.data) {
    case 1:
      //Go Forward
      mcDirectionFeedback.data = mcSubDirection.data;
      break;
    case 2:
      //Go Backward
      mcDirectionFeedback.data = mcSubDirection.data;
      break;
    case 3:
      //Turn Right
      mcDirectionFeedback.data = mcSubDirection.data;
      break;
    case 4:
      //Turn Left
      mcDirectionFeedback.data = mcSubDirection.data;
      break;
    default:
      //Stop
      mcDirectionFeedback.data = mcSubDirection.data;
      break;
  }
}

void doSpeedControll(const std_msgs::Int32 mcSubSpeed) {
  count++;
  mcSpeedFeedback.data = mcSubSpeed.data;
}
 
ros::Subscriber<std_msgs::Int32> mcSubDirection("mcDirection", &doDirectionControll);
ros::Subscriber<std_msgs::Int32> mcSubSpeed("mcSpeed", &doSpeedControll);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pubDirection);
  nh.advertise(pubSpeed);
  nh.advertise(pubBNO);
  nh.subscribe(mcSubDirection);
  nh.subscribe(mcSubSpeed);
  // initialize the BNO sensor
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  mcBNOFeedback.data = event.orientation.x;
  pubBNO.publish(&mcBNOFeedback);
  if (count > prevCount) {
    pubDirection.publish(&mcDirectionFeedback);
    pubSpeed.publish(&mcSpeedFeedback);
    prevCount = count;
  }
  nh.spinOnce();
  delay(100);
}
