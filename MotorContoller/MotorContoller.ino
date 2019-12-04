#include<ros.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/Twist.h>
#include<Wire.h>
//#include<Adafruit_Sensor.h>
//#include<Adafruit_BNO055.h>
//#include<utility/imumaths.h>

//Adafruit_BNO055 bno = Adafruit_BNO055(55);
ros::NodeHandle nh;

//used for detecting new event
uint8_t count = 1;
uint8_t prevCount = 0;

//declare pins for motor control
int motorPwmPins[] = {2,3,4,5};
int motorDirectionPins[] = {43, 44, 45, 46, 47, 48, 49, 50};
                          //0   1   2   3   4   5   6   7
/*                    FL(2)    FR(3)  <- Motor PWM pins
 *      Forward       43       45     <- Motor Direction Pins
 *      Reverse       44       46     <- Motor Direction Pins
 *                    RL(4)    RR(5)  <- Motor PWM pins
 *      Forward       47       49     <- Motor Direction Pins
 *      Reverse       48       50     <- Motor Direction Pins
 */
 int moveForward[] = {0,2,4,6};
 int moveBackward[] = {1, 3, 5, 7};
 int turnRight[] = {0, 3, 4, 7};
 int turnLeft[] = {1, 2, 5, 6};
 int dir = 0;
 int intspeed = 0;

//geometry_msgs::Twist mcDirectionFeedback; //Data sent to RPi4
std_msgs::Int32 mcDirectionFeedback;  //use geometry_twist for final code, string is for debugging
std_msgs::Int32 mcSpeedFeedback;
std_msgs::Float32 mcBNOFeedback;

ros::Publisher pubDirection("mcDirectionFeedback", &mcDirectionFeedback);
ros::Publisher pubSpeed("mcSpeedFeedback", &mcSpeedFeedback);

ros::Publisher pubBNO("mcBNO", &mcBNOFeedback);

void doDirectionControll( const std_msgs::Int32 &mcSubDirection) {
  count++;
  mcDirectionFeedback.data = mcSubDirection.data;
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
  /*
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
  
  //Ititialize pins for motor control
  bno.setExtCrystalUse(true);
  */
  for (int i = 0; i < 4; i++) {
    pinMode(motorPwmPins[i], OUTPUT);
  }
  for (int i = 0; i < 8; i++) {
    pinMode(motorDirectionPins[i], OUTPUT);
  }
}

void loop() {
  /*
  sensors_event_t event;
  bno.getEvent(&event);
  mcBNOFeedback.data = event.orientation.x;
  pubBNO.publish(&mcBNOFeedback);
  */
  if (count > prevCount) {
    pubDirection.publish(&mcDirectionFeedback);
    pubSpeed.publish(&mcSpeedFeedback);
    dir = mcDirectionFeedback.data;
    intspeed = mcSpeedFeedback.data;
    prevCount = count;
  }
  switch(dir) {
    case 1: //Forward
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveForward[i]], HIGH);
        digitalWrite(motorDirectionPins[moveBackward[i]], LOW);
        analogWrite(motorPwmPins[i], intspeed);
      }
      break;
    case 2: //Reverse
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveForward[i]], LOW);
        digitalWrite(motorDirectionPins[moveBackward[i]], HIGH);
        analogWrite(motorPwmPins[i], intspeed);
      }
      break;
    case 3: //Turn Right
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[turnRight[i]], HIGH);
        digitalWrite(motorDirectionPins[turnLeft[i]], LOW);
        analogWrite(motorPwmPins[i], intspeed);
      }
      break;
    case 4: //Turn Left
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[turnRight[i]], LOW);
        digitalWrite(motorDirectionPins[turnLeft[i]], HIGH);
        analogWrite(motorPwmPins[i], intspeed);
      }
      break;
    default:  //Stop
        for(int i = 0; i < 8; i++) {
          digitalWrite(motorDirectionPins[i], LOW);
        }
        for (int i = 0; i < 4; i++) {
          analogWrite(motorPwmPins[i], 0);
        }
        
      break;
  }
  nh.spinOnce();
  delay(100);
}
