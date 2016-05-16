

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
#include <std_msgs/Int32.h>
#include <Servo.h> 

//Pin Definition
#define LINEAR_PIN_OUT 10        //Linear Actuator Digital Pin out
#define LINEAR_PIN_IN  0        //Linear Actuator Digital Pin in

//max/min pulse values in microseconds for the linear actuators
#define LINEAR50_MIN  1050
#define LINEAR50_MAX  2000

ros::NodeHandle nh;

std_msgs::Int32 cur_pos; // the current position of the linear actuator, will be published as a topic.

void callback( const std_msgs::Int32& pos); // the callback declaration

ros::Publisher pub("linear_actuator_position",&cur_pos); // the published current position
ros::Subscriber<std_msgs::Int32> sub("linear_actuator_input", &callback); // the desired position for the actuator.

Servo linear_acutator;
int target_value = LINEAR50_MIN;

void setup() 
{ 
  // initializes pin for reading from the actuator.
  pinMode(LINEAR_PIN_IN,  INPUT);
  
  //initialize linear actuator
  linear_acutator.attach(LINEAR_PIN_OUT, LINEAR50_MIN, LINEAR50_MAX);  // attaches/activates the linear actuator as a servo object 
  
  //use the writeMicroseconds to set the linear actuators to their initial position
  linear_acutator.writeMicroseconds(target_value);

  // initialis ROS publishers and subscribers
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  delay(1000);
} 

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


