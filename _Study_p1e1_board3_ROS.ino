/*
  Autonomous Vehicles and Pedestrian Interaction with Visual Signals Study - Phase 1

  Experiment 1. Empirical Assessment of light signals for peripheral perception

  This code functions as a ROS node, receiving the vehicle location. It runs on an Ardiuno Uno.

  It is responsible for:
  1) Subscribe to ROS topics... and receive vehicle position
  2) Communicate with main board and transmit necessary information upon request
  
  
  It subscribes to..., and,
  transmits the received location to the Arduino main node (serial TX pin 12) upon request (serial RX pin 11)
  

  The circuit:  

  Request for vehicle location is received from main board via serial RX pin 11
  Vehicle locations are sent to main board (#2) via serial TX pin 12 
  The GND pins on both boards are common
  
  
  Programmer: 
    Henry Chen (Uwaterloo)
    May 2019

  
  This example code is based on 
  https://howtomechatronics.com/tutorials/arduino/arduino-and-hc-05-bluetooth-module-tutorial/
  https://www.teachmemicro.com/arduino-bluetooth/
  
  */

#include <ros.h>
#include <geometry_msgs/Point.h>

#include <SoftwareSerial.h>

SoftwareSerial Serial_ROS(6, 7); // RX, TX

ros::NodeHandle  nh;

int ros_request; // ros request for data
//float current_time;
float vehicle_pos_x;
float vehicle_pos_y;
float vehicle_pos_z;

void messageCb( const geometry_msgs::Point& pos_msg) {  
  vehicle_pos_x = pos_msg.x;
  vehicle_pos_y = pos_msg.y;
  vehicle_pos_z = pos_msg.z;
  
  Serial.println(vehicle_pos_x);
  Serial.println(vehicle_pos_y);
  Serial.println(vehicle_pos_z);
 
}

//subcribe to ROS topic vState
ros::Subscriber<geometry_msgs::Point> sub_pos("vState", messageCb ); 


void setup()
{
  Serial_ROS.begin(9600);  // HC-06 Bluetooth module line for APP control

  Serial.begin(9600);              // Serial Monitor for debugging
  Serial.println("ROS Node Ready...");

  // Note: Serial communication must be initiated before ROS node initization
  nh.initNode();
  nh.subscribe(sub_pos);
  
}

void loop()
{ 
  //current_time = (float) millis();
  
  if (Serial_ROS.available()){ //wait for data received
    
    ros_request=Serial_ROS.read();
    Serial.print("Ros request Received: ");
    Serial.println(ros_request);

    // Receive participant input
    if(ros_request == 61){  // ASCII '='
      Serial.println("Signal Detected!");
      Serial.println("Transmitting...");
      
      //Serial.print("The time is: ");
      //Serial.println(current_time);
      Serial_ROS.println(vehicle_pos_x);
      Serial_ROS.println(vehicle_pos_y);
      Serial_ROS.println(vehicle_pos_z);
    }
    
    if(ros_request == 62){  // ASCII '>'
      Serial.println("Signal Understood!!");
      Serial.println("Transmitting...");
      
      //Serial.print("The time is: ");
      //Serial.print(current_time+'\n');
      //Serial_ROS.println(current_time);
      Serial_ROS.println(vehicle_pos_x);
      Serial_ROS.println(vehicle_pos_y);
      Serial_ROS.println(vehicle_pos_z);
    }   
    
  }

  nh.spinOnce();
  delay(1000);

}
