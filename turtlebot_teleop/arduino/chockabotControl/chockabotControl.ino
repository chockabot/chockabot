

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
#include <std_msgs/Byte.h>
#include <Servo.h> 

//Pin Definition
#define LINEAR_PIN_OUT 10        //Linear Actuator Digital Pin out
#define LINEAR_PIN_IN  10        //Linear Actuator Digital Pin in

//max/min pulse values in microseconds for the linear actuators
#define LINEAR50_MIN  1050
#define LINEAR50_MAX  2000

ros::NodeHandle nh;

std_msgs::Byte cur_pos; // the current position of the linear actuator.
ros::Publisher pub("linear_actuator_position",&cur_pos); // the published current position
ros::Subscriber<std_msgs::Byte> sub("linear_actuator_input", &callback); // the desired position for the actuator.

Servo linear_acutator;
byte target_value = LINEAR50_MIN;
int speed = 10;

void setup() 
{ 
  //initialize linear actuator
  linear_acutator.attach(LINEAR_PIN_OUT, LINEAR50_MIN, LINEAR50_MAX);  // attaches/activates the linear actuator as a servo object 
  
  //use the writeMicroseconds to set the linear actuators to their initial position
  linear_acutator.writeMicroseconds(target_value);

  // initialis ROS
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  // initializes pins for reading from the actuator.
  pinMode(LINEAR_PIN_IN,  INPUT);

  // delays and sets the current value of the actuator.
  delay(100);
  cur_pos.data = digitalRead(LINEAR_PIN_IN);
} 

void loop() 
{ 
  // gets the current position through the actuator pin.
  cur_pos.data = digitalRead(LINEAR_PIN_IN);
  
  pub.publish(&cur_pos); // publishes current position.
  
  nh.spinOnce(); // all ROS communication is handled here.
  delay(1000);
} 

void messageCb( const std_msgs::Byte& pos){
{
  target_value = pos.data;
  newVal = map(target_value, 0, 255, LINEAR50_MAX, LINEAR50_MIN); //Map analog value from the sensor to the linear actuator
  
  //use the writeMicroseconds to write the target value to the linear actuator.
  linear_acutator.writeMicroseconds(newVal);
  
  nh.spinOnce(); // all ROS communication is handled here.
  delay(1000);
}

