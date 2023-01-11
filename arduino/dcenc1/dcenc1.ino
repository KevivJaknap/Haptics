#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int64 enc_val;
ros::Publisher pub("encoder", &enc_val);

volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
const int dir_pin = 9;
const int pwm_pin = 8;

int pwm=0;
void callback(const std_msgs::Int16& Pwm){
  pwm = Pwm.data;
}
ros::Subscriber<std_msgs::Int16> sub("pwm_dc", &callback);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 

  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}
void loop() {
  // Send the value of counter
  if (counter != temp) {
    Serial.println(counter);
    temp = counter;
  }
  enc_val.data = counter;
  pub.publish( &enc_val);
  nh.spinOnce();
  if(pwm < 0){
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, -pwm);
  }
  else{
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, pwm);
  }
  delay(100);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
