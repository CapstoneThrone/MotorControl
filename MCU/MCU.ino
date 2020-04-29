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
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include<Wire.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BNO055.h>
#include<utility/imumaths.h>
#include<PID_v1.h>

//System physical params for voltage divivder. Voltage of battery / Voltage of 
//divider while system is running
double voltageRatio = 2.876;

//System physical params for PID control
int t = 0;
double wheelDiameterMeters = 0.090;
double wheelCircumferenceMeters = wheelDiameterMeters * 3.14;
double ticksPerRev = 280; //From motor encoder datasheet

/**
 * Period of system cycle. Setting this too low will result in the Arduino MEGA
 * being overloaded and possible loss of control of robot due to a lost serial 
 * connection between it and the rpi. This is due to the interrupt-based encoder reading
 * taking up the time in between program execution.
 */
double PERIOD = 100;

//Defined constants for PID
double Kp = 2;  //Constant of Proportionality
double Ki = 10;  //Constant of Integration
double Kd = 0;  //Constant of Derivation (ignored for this program)
float target_linear_velocity; //read from cmd_vel
float target_angular_velocity;  //read from cmd_vel

//Speed, Distance, and Direction Variables
double  target_tickrate, //get tickrate for required speed
        target_tickrate_FL,
        target_tickrate_FR,
        target_tickrate_RL,
        target_tickrate_RR;
int target_dir = 0; //Control direction
int prev_target_dir = 0; //Store previous direction, used to tell when we need to change enable pin pattern

//Individual PID controllers, one for each wheel
double pulsePerPeriodFL = 0;
double newPWMFL = 0;
PID pidFL(&pulsePerPeriodFL, &newPWMFL, &target_tickrate_FL, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodFR = 0;
double newPWMFR = 0;
PID pidFR(&pulsePerPeriodFR, &newPWMFR, &target_tickrate_FR, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodRL = 0;
double newPWMRL = 0;
PID pidRL(&pulsePerPeriodRL, &newPWMRL, &target_tickrate_RL, Kp, Ki, Kd, DIRECT);
double pulsePerPeriodRR = 0;
double newPWMRR = 0;
PID pidRR(&pulsePerPeriodRR, &newPWMRR, &target_tickrate_RR, Kp, Ki, Kd, DIRECT);

/**
 * Encoder pins, must not change A's! Maximum amount of 
 * interrupt pins are being used on the MEGA 2560. This sketch
 * will not work if these are changed, because digital pins
 * can not keep up with the required sampling rate, and PID control
 * will become innacurate.
 */
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
//Subsets of enable pins for controlling motor directions
const int moveRight[] = {0, 2, 5, 7};
const int moveLeft[] = {1, 3, 4, 6};
const int moveForward[] = {0, 3, 4, 7};
const int moveBackward[] = {1, 2, 5, 6};
const int turnRight[] = {0, 2, 4, 6};
const int turnLeft[] = {1, 3, 5, 7};

//Feedback variables
std_msgs::Float32 mcVoltageFeedback;  //Voltage is a single float
std_msgs::Float32MultiArray mcBNOFeedback;  //Vector of floats for x, y, and z acceleration data
imu::Vector<3> euler; //IMU-Specific datatype for getting raw data from sensor

//Node handle for arduino to spin from
ros::NodeHandle nh;
ros::Publisher pubBNO("mcBNO", &mcBNOFeedback); //Publish orientation data
ros::Publisher pubVoltage("mcVoltage", &mcVoltageFeedback); //Publish voltage Feedback

#ifdef DEBUGROS
std_msgs::String mcDebugFeedback;
ros::Publisher pubDebug("mcDebug", &mcDebugFeedback);
#endif

//variables for battery monitoring
int voltageDividerInput = A0;
float intermediate;
uint8_t voltageTimer = 0;
uint8_t voltageTimerCount = 10;

//Callback executes whenever cmd_vel is recieved
void do_cmd_vel(const geometry_msgs::Twist cmd_vel) {
    //break down the twist angular and linear parts here
    target_linear_velocity = cmd_vel.linear.x; 
    target_angular_velocity = cmd_vel.angular.z;
}

//Subscriber that listens for a cmd_vel message and calls the above function (do_cmd_vel)
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &do_cmd_vel);

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
  nh.subscribe(cmd_vel);

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

  //Initialize and set tunings for PID control
  pidFL.SetMode(AUTOMATIC);
  pidFL.SetTunings(Kp, Ki, Kd);
  pidFR.SetMode(AUTOMATIC);
  pidFR.SetTunings(Kp, Ki, Kd);
  pidRL.SetMode(AUTOMATIC);
  pidRL.SetTunings(Kp, Ki, Kd);
  pidRR.SetMode(AUTOMATIC);
  pidRR.SetTunings(Kp, Ki, Kd);
  
  //MultiArrays require exact memory allocation
  mcBNOFeedback.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*3);
  //Three properties are set, label, size, and stride
  //size is the number of elements, stride is the length of each element relative to the datatype
  //Label is what you will see when the topic is echoed through rostopic in terminal
  mcBNOFeedback.layout.dim[0].label = "X_accel";
  mcBNOFeedback.layout.dim[0].size = 1;
  mcBNOFeedback.layout.dim[0].stride = 1;
  mcBNOFeedback.layout.dim[1].label = "Y_accel";
  mcBNOFeedback.layout.dim[1].size = 1;
  mcBNOFeedback.layout.dim[1].stride = 1;
  mcBNOFeedback.layout.dim[2].label = "Z_accel";
  mcBNOFeedback.layout.dim[2].size = 1;
  mcBNOFeedback.layout.dim[2].stride = 1;
  mcBNOFeedback.layout.data_offset = 0;
  mcBNOFeedback.data = (float *) malloc(sizeof(float)*3);
  mcBNOFeedback.data_length = 3;

  #ifdef DEBUG
  Serial.println("Setup End");
  #endif
}

void loop() { //loop code runs repeatedly as long as system is up
  
  //System runs once per PERIOD defined at top of program. (100ms default)
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
      //calculate average voltage. ADC is 1024 bits, voltageRatio defined above
      mcVoltageFeedback.data = voltageRatio * (intermediate / voltageTimerCount / 1023 * 5);
      //publish updated voltage
      pubVoltage.publish(&mcVoltageFeedback);
      voltageTimer = 0;
      intermediate = 0;
    }

    //read data from BNO055 and publish update
    sensors_event_t event;
    //bno.getEvent(&event);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    mcBNOFeedback.data[0] = euler.x();
    mcBNOFeedback.data[1] = euler.y();
    mcBNOFeedback.data[2] = euler.z();
    pubBNO.publish(&mcBNOFeedback);

    /**
     * Debug PID controllers
     * Compares desired tickrate (commanded) to instantaneously calculated tickrate (real)
     * Also shows linear and angular velocity components reseved from Raspberry Pi
     * To view this info, debug on PC and echo the "mcDebugFeedback" topic
     */
    #ifdef DEBUGROS
    String debugData =  "Liear Velocity: " + String(target_linear_velocity, 4) + 
                        " Angular Velocity: " + String(target_angular_velocity, 4) +
                        " FL_ACTUAL_TICKRATE: " + String(pulsePerPeriodFL, 4) +
                        " DESIRED_TICKRATE: " + String(target_tickrate, 4);
 
    mcDebugFeedback.data = debugData.c_str(); //the string defined for rosserial works with c_strings, convert it
    pubDebug.publish(&mcDebugFeedback);
    #endif

    //Calculate instantaneous pulse rates
    pulsePerPeriodFL = (pulseFL - prevPulseFL);
    prevPulseFL = pulseFL;
    pulsePerPeriodFR = (pulseFR - prevPulseFR);
    prevPulseFR = pulseFR;
    pulsePerPeriodRL = (pulseRL - prevPulseRL);
    prevPulseRL = pulseRL;
    pulsePerPeriodRR = (pulseRR - prevPulseRR);
    prevPulseRR = pulseRR;

    //Calculate desired base tick rate, if robot is moving
    if (target_angular_velocity == 0 && target_linear_velocity == 0)
    {
      //robot is stopped
      target_tickrate = 0;
      target_dir = 0;
    } else 
    {
      //Robot is moving, decide what kind of motion we have (angular, linear, or both)
      if (target_linear_velocity == 0)
      {
        //we are purely angular (rotation but no translation)
        target_tickrate = abs(1.0 * (((target_angular_velocity / 
        wheelCircumferenceMeters) * ticksPerRev) / (1000 / PERIOD)));
      }
      else
      {
        /**
         * we have linear motion (translation), and may have rotation (angular) as well.
         * But we will use the linear velocity as our base and use the angular velocity to modify it.
         */
        target_tickrate = abs(1.0 * (((target_linear_velocity / 
        wheelCircumferenceMeters) * ticksPerRev) / (1000 / PERIOD)));
      }
      
    }
    
    //Set target_tickrate for each wheel based on direction
    if (target_linear_velocity > 0)
    {
      target_dir = 1; //If the linear velocity is positive, the robot is moving forward
      if (target_angular_velocity == 0) //if there is no angular, it is moving straight
      {
        //forward
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate;
      }
      else if (target_angular_velocity > 0) //negative angular means veering towards the left
      {
        //forward and left
        target_tickrate_FL = target_tickrate * abs(target_angular_velocity);
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate * abs(target_angular_velocity);
        target_tickrate_RR = target_tickrate;
        
      } 
      else //The anular velocity must be positive, so we are veering towards the right
      {
        //forward and right
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate * abs(target_angular_velocity);
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate * abs(target_angular_velocity);
      }
    } 
    else if (target_linear_velocity < 0)  //A negative linear velocity means we are reversing
    {
      target_dir = 2;
      if (target_angular_velocity == 0) //No angular velocity means we are moving straight back
      {
        //backwards
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate;
      }
      else if (target_angular_velocity > 0) //A positive angular velocity means we are veering left
      {
        //backwards and left
        target_tickrate_FL = target_tickrate * abs(target_angular_velocity);
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate * abs(target_angular_velocity);
        target_tickrate_RR = target_tickrate;
      } 
      else //We must have a negative angular velocity, so we are veering right
      {
        //backwards and right
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate * abs(target_angular_velocity);
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate * abs(target_angular_velocity);
      }
    } 
    else 
    {
      //Robot is not moving foward or stopped, so it could be turning
      if (target_angular_velocity > 0) //Positive angular means turn left
      {
        //turning left
        target_dir = 3;
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate;
      }
      else if (target_angular_velocity < 0) //Negative angular means turn right
      {
        //turning right
        target_dir = 4;
        target_tickrate_FL = target_tickrate;
        target_tickrate_FR = target_tickrate;
        target_tickrate_RL = target_tickrate;
        target_tickrate_RR = target_tickrate;
      }
    }

    //Read the target direction and enable the correct pins for that movemenet pattern
    if (target_dir != prev_target_dir) {
      prev_target_dir = target_dir;
      if (target_dir == 0)  //Reset all enable pins to stop robot
      {
        //stop
        for(int i = 0; i < 8; i++) 
        {
          digitalWrite(motorDirectionPins[i], LOW);
        }
      } else if (target_dir == 1)
      {
        for (int i = 0; i < 4; i++) 
        {
          digitalWrite(motorDirectionPins[moveForward[i]], HIGH);
        }
      } else if (target_dir == 2)
      {
        for (int i = 0; i < 4; i++) 
        {
          digitalWrite(motorDirectionPins[moveBackward[i]], HIGH);
        }
      } else if (target_dir == 3)
      {
        for (int i = 0; i < 4; i++) 
        {
          digitalWrite(motorDirectionPins[turnLeft[i]], HIGH);
        }
      } else if (target_dir == 4)
      {
        for (int i = 0; i < 4; i++) 
        {
          digitalWrite(motorDirectionPins[turnRight[i]], HIGH);
        }
      } else  //Stop if we recieve an unknown value
      {
         //stop
        for(int i = 0; i < 8; i++)
        {
          digitalWrite(motorDirectionPins[i], LOW);
        }
      }
    }
    
    //run PID controllers only when needed. Prevents jumping when changing directions due to PID overcompensation
    //(The PID controller will think a wheel is stuck when in reality, its enable pin is off so it can't spin
    //This causes the PID to send 255 full duty cycle to motor, which will cause a rough start
    switch(target_dir)
    {
      case 0:
        pidFL.SetMode(AUTOMATIC);
        pidFL.SetTunings(Kp, Ki, Kd);
        pidFR.SetMode(AUTOMATIC);
        pidFR.SetTunings(Kp, Ki, Kd);
        pidRL.SetMode(AUTOMATIC);
        pidRL.SetTunings(Kp, Ki, Kd);
        pidRR.SetMode(AUTOMATIC);
        pidRR.SetTunings(Kp, Ki, Kd);
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
