//#define DEBUGROS 1
//#define DEBUG 1
/**
   This progarm is written for use with an Arduino MEGA 2560 r3.
   Written by the CEE Motor Control Group of the 2019-2020 UW-Stout AMR Capstone Project
   Authors: Jared Wilquet, Kyle Domack, Jacob Hillebrand
   References for Ros on the Arduino -> http://wiki.ros.org/rosserial_arduino
   References for BNO055 Sensor on the Arduino -> https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code?view=all
   Wiring diagrams, system schematics, and all code @ GitHub Repo -> https://github.com/CapstoneThrone/MotorControl.git
   PID Library is Arduino PID Library by Brett Beauregard -> https://playground.arduino.cc/Code/PIDLibrary/
*/

#include<ros.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Int32.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Twist.h>
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<utility/imumaths.h>
#include<PID_v1.h>


//System physical params for PID control
int t = 0;
double wheelDiameterMeters = 0.090;
double wheelCircumferenceMeters = wheelDiameterMeters * 3.14;
double ticksPerRev = 280; //From motor encoder datasheet
double PERIOD = 100; //Period in ms for PID to sample from
double Kp = 2;  //Constant of Proportionality
double Ki = 0;   //Constant of Integration
double Kd = 0;   //Constant of Derivation (ignored for this program)

//Speed, Distance, and Direction Variables
double target_speed = 0; //Speed in Meters/Second (input to Arduino)
double target_tickrate; //get tickrate for required speed
double target_distance = 0; //Distance to move per command (input to Arduino)
double target_ticks_to_distance = 0; //Number of encoder ticks for a given distance
int target_dir = 0; //Control direction

//Individual PID controllers, one for each wheel
double pulsePerPeriodFL = 0;
double newPWMFL = 0;
PID pidFL(&pulsePerPeriodFL, &newPWMFL, &target_tickrate, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodFR = 0;
double newPWMFR = 0;
PID pidFR(&pulsePerPeriodFR, &newPWMFR, &target_tickrate, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodRL = 0;
double newPWMRL = 0;
PID pidRL(&pulsePerPeriodRL, &newPWMRL, &target_tickrate, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodRR = 0;
double newPWMRR = 0;
PID pidRR(&pulsePerPeriodRR, &newPWMRR, &target_tickrate, Kp, Ki, Kd, DIRECT);
//Encoder pins, must not change A's! Maximum amount of interrupt pins are being used on the MEGA 2560
const int FL_A = 2;
const int FL_B = 36;
const int FR_A = 3;
const int FR_B = 37;
const int RL_A = 18;
const int RL_B = 38;
const int RR_A = 19;
const int RR_B = 39;
//pulse counters for each encoder
volatile int pulseFL = 0;
volatile int prevPulseFL = 0;
volatile int pulseFR = 0;
volatile int prevPulseFR = 0;
volatile int pulseRL = 0;
volatile int prevPulseRL = 0;
volatile int pulseRR = 0;
volatile int prevPulseRR = 0;


/**
   INTERRUPTS INTERRUPTS INTERRUPTS INTERRUPTS
*/
void int_FL()
{
  pulseFL++;
}
void int_FR()
{
  pulseFR++;
}
void int_RL()
{
  pulseRL++;
}
void int_RR()
{
  pulseRR++;
}

/**
   END INTERRUPTS
*/


Adafruit_BNO055 bno = Adafruit_BNO055(55);


//declare pins for motor control
const int motorPwmPins[] = {4, 5, 6, 7};
const int motorDirectionPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
                                 //0   1   2   3   4   5   6   7

/**
 *  0 F22,  1 B23, PWM4        2 F24, 3 B25, PWM5
 *
 *  4 F26,  5 B27, PWM6       6 F28, 7 B29, PWM7
*/
//Subsets of pins for movement
const int moveRight[] = {0, 2, 5, 7};
const int moveLeft[] = {1, 3, 4, 6};
const int moveForward[] = {0, 3, 4, 7};
const int moveBackward[] = {1, 2, 5, 6};
const int turnRight[] = {0, 2, 4, 6};
const int turnLeft[] = {1, 3, 5, 7};

//Feedback functions
std_msgs::Float32 mcBNOFeedback;
std_msgs::Float32 mcVoltageFeedback;


ros::NodeHandle nh;
ros::Publisher pubBNO("mcBNO", &mcBNOFeedback);
ros::Publisher pubVoltage("mcVoltage", &mcVoltageFeedback);

#ifdef DEBUGROS
std_msgs::String mcDebugFeedback;
ros::Publisher pubDebug("mcDebug", &mcDebugFeedback);
#endif

//variables for battery monitoring
int voltageDividerInput = A0;
float intermediate;
uint8_t voltageTimer = 0;
uint8_t voltageTimerCount = 10;

void doDirectionControl( const std_msgs::Int32 &mcSubDirection) {
  target_dir = mcSubDirection.data;
  
  //Clear all enable pins before issuing a new command
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(motorDirectionPins[i], LOW);
  }
  
  //Decoding motor direction, setting motor enable pins
  switch (target_dir)
  {
    case 1: //Forward
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveForward[i]], HIGH);
      }
      break;
    case 2: //Reverse
      digitalWrite(26, LOW); // solves unknown error with right weel not runnign backwards
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveBackward[i]], HIGH);
      }
      break;
    case 3: //Turn Right
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[turnRight[i]], HIGH);
      }
      break;
    case 4: //Turn Left
     for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[turnLeft[i]], HIGH);
      }
      break;
    case 5: //Strafe Right
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveRight[i]], HIGH);
      }
      break;
    case 6: //Strafe Left
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorDirectionPins[moveLeft[i]], HIGH);
      }
      break;
    default:  //Stop
      for (int i = 0; i < 8; i++)
      {
        digitalWrite(motorDirectionPins[i], LOW);
      }
      break;
  }
}

void doSpeedControl(const std_msgs::Float32 mcSubSpeed) {
  target_speed = 1.0 * mcSubSpeed.data;
  target_tickrate = 1.0 * (((target_speed / wheelCircumferenceMeters) * 280) / (1000 / PERIOD));
  if (target_speed == 0)
  {
    newPWMFL = 0;
    newPWMFR = 0;
    newPWMRL = 0;
    newPWMRR = 0;
  }
}

void doDistanceControl(const std_msgs::Float32 mcSubDistance) {
  target_distance = mcSubDistance.data;
  target_ticks_to_distance = (target_distance / ( .09 * 3.14 )) * 280;
}

ros::Subscriber<std_msgs::Int32> mcSubDirection("mcDirection", &doDirectionControl);
ros::Subscriber<std_msgs::Float32> mcSubSpeed("mcSpeed", &doSpeedControl);
ros::Subscriber<std_msgs::Float32> mcSubDistance("mcDistance", &doDistanceControl);

void setup() {  //setup code runs on device startup

  //debugging
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Setup Begin");
  #endif
  
  //Set encoder inputs and configure internal resistors for pull-up
  pinMode(FL_A, INPUT_PULLUP);
  pinMode(FL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FL_A), int_FL, FALLING);
  pinMode(FR_A, INPUT_PULLUP);
  pinMode(FR_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FR_A), int_FR, FALLING);
  pinMode(RL_A, INPUT_PULLUP);
  pinMode(RL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RL_A), int_RL, FALLING);
  pinMode(RR_A, INPUT_PULLUP);
  pinMode(RR_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RR_A), int_RR, FALLING);

  //A0 for voltage divider from battery input
  pinMode(voltageDividerInput, INPUT);

  //Initialize ROS node handle, advertizers, and subscribers
  nh.initNode();
  nh.advertise(pubVoltage);
  nh.advertise(pubBNO);
  nh.subscribe(mcSubDirection);
  nh.subscribe(mcSubSpeed);
  nh.subscribe(mcSubDistance);
  #ifdef DEBUGROS
  nh.advertise(pubDebug);
  #endif
  

  // initialize the BNO sensor
  if (!bno.begin())
  {
    while (1);
  }

  //delay to wait for BNO to initialize
  delay(1000);
   
  //Ititialize pins for motor control
  bno.setExtCrystalUse(true);

  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPwmPins[i], OUTPUT);
  }

  for (int i = 0; i < 8; i++)
  {
    pinMode(motorDirectionPins[i], OUTPUT);
  }

  pidFL.SetMode(AUTOMATIC);
  pidFL.SetTunings(Kp, Ki, Kd);
  pidFR.SetMode(AUTOMATIC);
  pidFR.SetTunings(Kp, Ki, Kd);
  pidRL.SetMode(AUTOMATIC);
  pidRL.SetTunings(Kp, Ki, Kd);
  pidRR.SetMode(AUTOMATIC);
  pidRR.SetTunings(Kp, Ki, Kd);

  #ifdef DEBUG
  Serial.println("Setup End");
  #endif
}

void loop() { //loop code runs repeatedly as long as system is up
  
  //Run the PID controller once per period
  if (millis() - t > PERIOD)
  {
    #ifdef DEBUG
    Serial.print("Loop");
    #endif
     
    //battery voltage monitoring
    intermediate += analogRead(voltageDividerInput);
    voltageTimer++;
    if (voltageTimer == voltageTimerCount)
    {
      //calculate average voltage. ADC is 1024 bits, voltage divider gives radio of roughly 5/12 V for prototype
      mcVoltageFeedback.data = intermediate / voltageTimerCount / 1023 * 5;
      //publish updated voltage
      pubVoltage.publish(&mcVoltageFeedback);
      voltageTimer = 0;
      intermediate = 0;
    }

    //read data from BNO055 and publish update
    sensors_event_t event;
    bno.getEvent(&event);
    mcBNOFeedback.data = event.orientation.x;
    pubBNO.publish(&mcBNOFeedback);
    #ifdef DEBUGROS
    mcDebugFeedback.data = "in loop";
    pubDebug.publish(&mcDebugFeedback);
    #endif
    
    pulsePerPeriodFL = (pulseFL - prevPulseFL);
    prevPulseFL = pulseFL;
    pulsePerPeriodFR = (pulseFR - prevPulseFR);
    prevPulseFR = pulseFR;
    pulsePerPeriodRL = (pulseRL - prevPulseRL);
    prevPulseRL = pulseRL;
    pulsePerPeriodRR = (pulseRR - prevPulseRR);
    prevPulseRR = pulseRR;

    //run PID controllers only when needed. Prevents jumping when changing directions due to PID overcompensation
    switch(target_dir)
    {
      case 0:
        break;
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        pidFL.Compute();
        pidRR.Compute();
        pidFR.Compute();
        pidRL.Compute();
        break;
      default:
        break;
    }
   
    //Update duty cycles for each motor drive pin
    analogWrite(motorPwmPins[0], newPWMFL);
    analogWrite(motorPwmPins[1], newPWMFR);
    analogWrite(motorPwmPins[2], newPWMRL);
    analogWrite(motorPwmPins[3], newPWMRR);

    #ifdef DEBUG
      Serial.print(" ");
      Serial.print(newPWMFL);
       Serial.print(" ");
      Serial.print(newPWMFL);
       Serial.print(" ");
      Serial.print(newPWMFL);
       Serial.print(" ");
      Serial.println(newPWMFL);
    #endif
      
    t = millis();

    /**
      Run the ROS serial node cycle once per loop
      NOTE: This HAS to be spinOnce() because the loop
      function is called repeatedly. Using a looping functinon
      casuses undefined behavior.
    */
   nh.spinOnce();
  }
}
