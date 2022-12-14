#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN D8 
const char* ssid     = "Vivek's iPhone";
const char* password = "w23z6twe1a6w2";//yS79Ko9QAIkh
// Set the rosserial socket server IP address
IPAddress server(192,168,0,2);
// Set the rosserial socket server port
const uint16_t serverPort = 11414;

ros::NodeHandle nh;
// Make a chatter publisher
sensor_msgs::Imu imu_msg;
std_msgs::Float32 flex;
ros::Publisher flexer("flex_hand", &flex);
ros::Publisher chatter("imu_machine", &imu_msg);
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
    //Wire.begin();    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();   

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    if (devStatus == 0) {  
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();  
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {
  nh.spinOnce();

    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x01) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;            
            
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);


            Serial.print("Orientation X: ");
            Serial.print(q.x);
            Serial.print(", Y: ");
            Serial.print(q.y);
            Serial.print(", Z: ");
            Serial.print(q.z);
            Serial.print(", W: ");
            Serial.print(q.w);
            Serial.println("");
            
            Serial.print("Acceleration X: ");
            Serial.print(aaReal.x * 1/16384. * 9.80665);
            Serial.print(", Y: ");
            Serial.print(aaReal.y * 1/16384. * 9.80665);
            Serial.print(", Z: ");
            Serial.print(aaReal.z * 1/16384. * 9.80665);
            Serial.println("");


            Serial.print("Rotation X: ");
            Serial.print(ypr[0]);
            Serial.print(", Y: ");
            Serial.print(ypr[1]);
            Serial.print(", Z: ");
            Serial.print(ypr[2]);
            Serial.println("");
            imu_msg.orientation.x = q.x;
            imu_msg.orientation.y = q.y;
            imu_msg.orientation.z = q.z;
            imu_msg.orientation.w = q.w;
            imu_msg.angular_velocity.x = ypr[0];
            imu_msg.angular_velocity.y = ypr[1];
            imu_msg.angular_velocity.z = ypr[2];

            imu_msg.linear_acceleration.x =aaReal.x * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.y =aaReal.y * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.z =aaReal.z * 1/16384. * 9.80665;
            flex.data = get_flex();
            if (nh.connected()) {
                Serial.println("Connected");
                flexer.publish( &flex );
                chatter.publish( &imu_msg );
              } else {
                Serial.println("Not Connected");
              }
   
            delay(200);         
    }
}
const int flexPin=A0;
const float VCC = 5;      // voltage at Ardunio 5V line
const float R_DIV = 33000.0;  // resistor used to create a voltage divider
const float flatResistance = 52000.0; // resistance when flat
const float bendResistance = 150000.0;
float get_flex(){
  int ADCflex = analogRead(flexPin);
  float Vflex = ADCflex * VCC / 1023.0;
  float Rflex = R_DIV * (VCC / Vflex - 1.0);

//  float angle = map(Rflex, flatResistance, bendResistance, 0, 90.0);
//  if(angle > 60){
//    return false;
//  }
//  return true;
  return Rflex;
}
