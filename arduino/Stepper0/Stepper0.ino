  // put you// Include the AccelStepper Library
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
int theta=0;


// Define pin connections
const int dirPin = 2;
const int stepPin = 3;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void callback(const std_msgs::Int16& angle){
//  theta = constrain(angle.data, -90, 90);
//  theta = theta*(5/9);
//  theta = theta*5/9;
//  theta = int(theta);
  if(angle.data < 0){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
  }
  func(angle.data);
}
ros::Subscriber<std_msgs::Int16> sub("stepper", &callback);
void func(int angle){
  myStepper.moveTo(angle);
  while(myStepper.distanceToGo() != 0){
    myStepper.run();
    delay(1);
  }
}
void setup() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  pinMode(LED_BUILTIN, OUTPUT);
  myStepper.setMaxSpeed(100);
  myStepper.setAcceleration(100);
  myStepper.setSpeed(50);
//  myStepper.moveTo(100);
  nh.initNode();
  nh.subscribe(sub);
  func(200);
}
void loop() {
  // Change direction once the motor reaches target position
//  if (myStepper.distanceToGo() == 0) 
//    myStepper.moveTo(-myStepper.currentPosition());
//
//  // Move the motor one step
//  myStepper.run();
//  myStepper.moveTo(200);
//  myStepper.run();
  nh.spinOnce();
  delay(10);
}
