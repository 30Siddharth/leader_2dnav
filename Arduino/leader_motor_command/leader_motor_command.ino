
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define PWM_MIN 0
#define PWMRANGE 100

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

float mapPwm(float x, float out_min, float out_max);

// constants for 1st motor
const int inAPin1 = 6;  //orange
const int inBPin1 = 5;  //red
const int PWMPin1 = 4;   //brown

// constants for 2nd motor
const int inAPin2 = 9; //grey
const int inBPin2 = 8;  //white
const int PWMPin2 = 10;  //purple

void motor_cb( const geometry_msgs::Twist& cmd_vel){
  float x_des = cmd_vel.linear.x;
  float yaw_des = cmd_vel.angular.z;

  float scale = 1.3;

  float left = scale*(x_des + 2*yaw_des) / 2;
  float right = scale*(x_des - 2*yaw_des) / 2;

  uint16_t lPwm = mapPwm(fabs(left), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(right), PWM_MIN, PWMRANGE);

  digitalWrite(inAPin2, left > 0);
  digitalWrite(inBPin2, left < 0);
  digitalWrite(inAPin1, right > 0);
  digitalWrite(inBPin1, right < 0);

  lPwm = lPwm*1;
  rPwm = rPwm*1.6;
  if (lPwm<20)
  {
    lPwm = 0;
  } 

  if (rPwm<20)
  {
    rPwm = 0;
  }
  
  analogWrite(PWMPin2, lPwm);
  analogWrite(PWMPin1, rPwm);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", motor_cb);

void setup(){ // setting all pins as outputs
  //pinMode(13, OUTPUT);
  pinMode(inAPin1,OUTPUT);
  pinMode(inBPin1,OUTPUT);
  pinMode(PWMPin1,OUTPUT);

  pinMode(inAPin2,OUTPUT);
  pinMode(inBPin2,OUTPUT);
  pinMode(PWMPin2,OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void stop()
{
  digitalWrite(inAPin2, 0);
  digitalWrite(inBPin2, 0);
  digitalWrite(inAPin1, 0);
  digitalWrite(inBPin1,  0);
  analogWrite(PWMPin2, 0);
  analogWrite(PWMPin1, 0);
}

void loop()
{
  if (!nh.connected()) {
    stop();
  }
  nh.spinOnce();
  delay(100);
}

// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

// NOTES: I think pins A and B each control a wheel, so both must be high or both low when controlling
//        Should have pins A and B on the whole time then PWM controls whether or not they move
//        PWM pins control speed? so vertical scroller will control speed of each side? 
//        Pins A and B from one motor should be set to LOW when not supposed to be moving 
//        Vertical scroller should go from 0 to 5 for 0V to 5V
//        Horizontal scroller can go from 0 to 1 to indicate LOW or HIGH 
//        Should PWM scroller go to negatives to indicate going backwards??
//        In rqt can you change the names to cmd_vel and cmd_vel2?
//        Should the main function still be "void servo_cb"? what should it actually be? just a void loop?
//        If you want to robot to stop then it has to be done simultaneously, maybe 2nd joy pad only controls speed and direction
//        Should pins A and B always be on high, and motors just stop when PWM = 0?
