#include<ros.h>
#include<std_msgs/Byte.h>
#include<std_msgs/String.h>

ros::NodeHandle nh;

uint8_t count = 1;
uint8_t prevCount = 0;
String data_str;
String speed_data_str;
String direction_data_str;
int speed_data;
std_msgs::String mcPubMSG; //Data sent to RPi4

/* mcSubData format
 * First 4 bytes are left, right, forward, and backward
 * 1000 <- left 0100 <- right 0010 <- forward 0001 <- backward
 * 
 * Last 4 bytes are a multiplexed movement speed
 * 
 * Examples: 0100 1111 would mean turn right at 100% movement speed,
 *           0001 1000 would mean drive backward at 50% movement speed
 */

 ros::Publisher pub("mcFeedback", &mcPubMSG);

 void doControl( const std_msgs::String& mcSubData) {
  count++;
  data_str = mcSubData.data;
  //mcPubMSG.data = mcSubData.data;
  direction_data_str = data_str.substring(0,3);
  speed_data_str = data_str.substring(4,7);
  
  if (direction_data_str == "1000") {
    //turn left
    mcPubMSG.data = "turn left";
  } else if (direction_data_str == "0100"){
    //turn right
    mcPubMSG.data = "turn right";
  } else if (direction_data_str == "0010"){
    //drive forward
    mcPubMSG.data = "drive forward";
  } else if (direction_data_str == "0001") {
    //drive backward
    mcPubMSG.data = "drive backward";
  } else {
    //send error code to RPI4, bad data
    mcPubMSG.data = "Error, bad data";
  }
  
 }
 
 ros::Subscriber<std_msgs::String> sub("mcInstruction", &doControl);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
}

long publisher_timer;

void loop() {
  if (count > prevCount) {
    pub.publish(&mcPubMSG);
    prevCount = count;
  }
  nh.spinOnce();
  delay(1000);
}
