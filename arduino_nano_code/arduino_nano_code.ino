// https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/ : use string !
#include "Wire.h"

#define PIN_TRIGGER 7
#define PIN_PPS 10

// ROS related 
#include <ros.h>
#include <std_msgs/String.h>

// node handler
ros::NodeHandle nh;

volatile unsigned long trigger_time = 0;
volatile unsigned long imu_time = 0;
unsigned long time_sec = 0;
unsigned long time_nsec = 0;
volatile unsigned long triggerCounter = 0;

void setup() {
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_PPS, OUTPUT);

  Serial.begin(115200);

  // ROS initialization
  nh.getHardware()->setBaud(115200);

  nh.initNode();
}

uint8_t cnt = 1;
uint32_t cnt_imu = 0;
uint32_t cnt_trigger = 0;
long publisher_timer;

void loop() {

  if(micros() > publisher_timer){
    // step 1: request reading from sensor
    publisher_timer = micros() + 5000; // 200 Hz
    //imu_msg.seq  = cnt_imu;
    //imu_msg.data = data_final;
    ++cnt;
    ++cnt_imu;

    // count (200 Hz IMU. 20 Hz image)
    if(cnt > 10){
      cnt = 1;
      digitalWrite(PIN_TRIGGER, HIGH);
      digitalWrite(PIN_TRIGGER, LOW);
      //imu_msg.flag_trigger = 1;
    }
    else{ // non-triggered signal
      //imu_msg.flag_trigger = 0;      
    }
    // pub_imu.publish(&imu_msg);
    nh.spinOnce();
  }

  
  // delay 5 ms (for 200 Hz)
  
}
