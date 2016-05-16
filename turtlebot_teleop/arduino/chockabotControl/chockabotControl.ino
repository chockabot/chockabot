

/***********************************************************************************
 *           RobotGeek Firgelli Linear Actuator Control Demo   
 *           Absolute Analog Control                
 * 
 *
 *  The following sketch will allow you to control a Firgelli Linear actuator using
 *  the RobotGeek Slider and RobotGeek Knob
 *
 *  Products
 *  
 *    http://www.robotgeek.com/robotgeek-geekduino-sensor-kit
 *    http://www.robotgeek.com/robotgeek-slider
 *    http://www.robotgeek.com/robotgeek-rotation-knob
 *    http://www.robotgeek.com/p/power-supply-6vdc-2a.aspx
 *    Firgelli Mini Linear Actuators http://www.robotgeek.com/store/Search.aspx?SearchTerms=firgelli
 *    http://www.robotgeek.com/robotgeek-small-workbench.aspx
 
 *  Wiring
 *    100mm Linear Actuator - Digital Pin 9 
 *    50mm Linear Actuator - Digital Pin 10 
 *
 *    Knob   - Analog Pin 0
 *    Slider - Analog Pin 1 
 *    Jumper for pins 9/10/11 should be set to 'VIN'
 *  
 *
 *  Control Behavior:
 *    Moving the analog controls will move the linear actuators keeping absolute position.
 *
 *  External Resources
 *
 ***********************************************************************************/
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <Servo.h> 

//Pin Definition
#define LINEAR_PIN_OUT 10        //Linear Actuator Digital Pin out
#define LINEAR_PIN_IN  0        //Linear Actuator Digital Pin in

//max/min pulse values in microseconds for the linear actuators
#define LINEAR50_MIN  1050
#define LINEAR50_MAX  2000

ros::NodeHandle nh;

std_msgs::Int32 cur_pos; // the current position of the linear actuator.
std_msgs::String info;

ros::Publisher pub("linear_actuator_position",&cur_pos); // the published current position
ros::Publisher pub_info("linear_actuator_info",&info); // the published current position

Servo linear_acutator;
byte target_value = LINEAR50_MIN;
byte newVal;
int speed = 10;

void callback( const std_msgs::Int32& pos)
{
  // publishes info
  info.data = "in callback ..target val newval = :";
  pub_info.publish(&info);
  nh.spinOnce();

  
  // converts string to int.
  target_value = pos.data;
  // publishes info
  info.data = String(target_value);
  pub_info.publish(&info);
  nh.spinOnce();

  newVal = map(target_value, 0, 255, LINEAR50_MAX, LINEAR50_MIN); //Map String value to sensor value.
  
  info.data = String(newval);
  pub_info.publish(&info);
  nh.spinOnce();
  
  //use the writeMicroseconds to write the target value to the linear actuator.
  linear_acutator.writeMicroseconds(newVal);
  
  nh.spinOnce(); // all ROS communication is handled here.
  delay(1000);
}

ros::Subscriber<std_msgs::Int32> sub("linear_actuator_input", &callback); // the desired position for the actuator.

void setup() 
{ 
  //initialize linear actuator
  linear_acutator.attach(LINEAR_PIN_OUT, LINEAR50_MIN, LINEAR50_MAX);  // attaches/activates the linear actuator as a servo object 
  
  //use the writeMicroseconds to set the linear actuators to their initial position
  linear_acutator.writeMicroseconds(target_value + 50);

  // initialis ROS
  nh.initNode();
  nh.advertise(pub);
  nh.advertise(pub_info);
  nh.subscribe(sub);

  // initializes pins for reading from the actuator.
  pinMode(LINEAR_PIN_IN,  INPUT);

  delay(1000);
} 

void loop() 
{ 
  // gets the current position through the actuator pin.
  cur_pos.data = analogRead(LINEAR_PIN_IN);
  
  //might have to do this to map analog read value to a byte
  //int val = analogRead(0);
  //val = map(val, 0, 1023, 0, 255);
  
  pub.publish(&cur_pos); // publishes current position.
  
  nh.spinOnce(); // all ROS communication is handled here.
  nh.spinOnce();
  nh.spinOnce(); // all ROS communication is handled here.
  nh.spinOnce();
  
  delay(1000);
} 



