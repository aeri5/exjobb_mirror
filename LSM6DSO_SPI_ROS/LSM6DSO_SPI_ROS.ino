/******************************************************************************

Sparkfun LSM6DSO Arduino library Basic_Readings.ino example converted 
to SPI, implemented as ROS publisher node.

******************************************************************************/

#include "SparkFunLSM6DSO.h"
//#include "Wire.h"
#include "SPI.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

LSM6DSO IMU;                    // init IMU object
ros::NodeHandle nh;             // define ros node handle

const uint8_t slaveSelectPin = 10;          // set pin 10 to slave select
const uint32_t spiPortSpeed = 10000000;     // set SPI port speed to 10 Mhz

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu0", &imu_msg); //init publisher

void setup() {
  //Serial.begin(115200);       // not used on teensy
  //delay(500); 

  pinMode (slaveSelectPin, OUTPUT);       // set slave select pin as output:
  digitalWrite (slaveSelectPin, HIGH);    // init slave select pin to high when idle

  SPI.begin();    // init SPI

  if ( IMU.beginSPI( slaveSelectPin, spiPortSpeed, SPI ) )  // (csPin, spiPortSpeed, &spiPort  
    Serial.println("Ready." );
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( IMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

  nh.initNode();                // init ROS node
  nh.advertise(imu_pub);

  imu_msg.header.frame_id = "/imu0";        // set header frame id
  imu_msg.orientation_covariance[0] = -1;   // set first element of orientation cov matrix to -1 as per
                                            // sensor_msgs/Imu  since we do not estimate orientation
}


void loop()
{ 
  imu_msg.angular_velocity.x = IMU.readFloatGyroX();      // read gyro values
  imu_msg.angular_velocity.y = IMU.readFloatGyroY();
  imu_msg.angular_velocity.z = IMU.readFloatGyroZ();

  imu_msg.linear_acceleration.x = IMU.readFloatAccelX();  // read lin acc values
  imu_msg.linear_acceleration.y = IMU.readFloatAccelY();
  imu_msg.linear_acceleration.z = IMU.readFloatAccelZ();
  
  imu_msg.header.stamp = nh.now();  // timestamp 
  imu_pub.publish( &imu_msg );      // publish on /imu0 topic

  nh.spinOnce();
  delay(5);         // 200 hz
}
