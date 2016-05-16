/* Chockabot Control arduino node
 * This node will be responsible for controlling the 
 * linear actuator. it publishes a topic "/linear_actuator_position"
 * showing the current position of the actuator as a value in the range [0,255]
 * and accepts target position by subscribing to "/linear_actuator_input", where
 * if something is published to this it will move to the desired position.
 */
 
#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h> 

//Pin Definitions
#define LINEAR_PIN_OUT 10        //Linear Actuator Digital Pin out
#define LINEAR_PIN_IN  0        //Linear Actuator Analog Pin in

//max/min pulse values in microseconds for the linear actuators defined from datasheet
#define LINEAR50_MIN  1050
#define LINEAR50_MAX  2000

ros::NodeHandle nh;

std_msgs::Int32 cur_pos; // the current position of the linear actuator, will be published as a topic.

void callback( const std_msgs::Int32& pos); // the callback declaration

ros::Publisher pub("linear_actuator_position",&cur_pos); // the current position publisher
ros::Subscriber<std_msgs::Int32> sub("linear_actuator_input", &callback); // Subscriber to look for desired position for the actuator.

Servo linear_acutator;
int target_value = LINEAR50_MIN; // the target position of the actuator.

void setup() 
{ 
  // initializes pin for reading from the actuator.
  pinMode(LINEAR_PIN_IN,  INPUT);
  
  //initialize linear actuator
  linear_acutator.attach(LINEAR_PIN_OUT, LINEAR50_MIN, LINEAR50_MAX);  // attaches/activates the linear actuator as a servo object 
  
  //use the writeMicroseconds to set the linear actuators to their initial position (LINEAR50_MIN)
  linear_acutator.writeMicroseconds(target_value);

  // initialis ROS publishers and subscribers
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  delay(1000);
} 

/*  Main loop of program publishes current position
 *  and listens for topics to come in to tell it 
 *  what position to move to.
 */
void loop() 
{ 
  // gets the current position through the actuator pin.
  int analogValue = analogRead(LINEAR_PIN_IN);
  
  //Map analog input value (0-360) to a value representative of the input value (0-255). 
  cur_pos.data = map(analogValue, 10, 360, 255, 0);
  
  pub.publish(&cur_pos); // publishes current position.
  
  // all ROS communication (publishing, callbacks) is handled here.
  nh.spinOnce();
  nh.spinOnce();
  nh.spinOnce();
  nh.spinOnce();
  
  delay(1000);
} 

/* This callback is executed when someone publishes
 *  to the "/linear_actuator_input" topic. the data published 
 *  is an Int32 in the range (0-255) representing the desired position
 *  of the actuator, where 0 is the lowest position and 255 is the highest.
 *  They can then check the current position of the actuator by subscribing to
 *  "/linear_actuator_position" where they can look at current position values
 *  to tell where the actuator currently is.
 */
void callback( const std_msgs::Int32& pos)
{
  target_value = pos.data;
  
  //Map input value (0-255) to sensor value.
  int newVal = map(255 - target_value, 0, 255, LINEAR50_MAX, LINEAR50_MIN);
    
  //use the writeMicroseconds to write the target value to the linear actuator.
  linear_acutator.writeMicroseconds(newVal);
  
  nh.spinOnce(); // all ROS communication is handled here.
  delay(1000);
}


