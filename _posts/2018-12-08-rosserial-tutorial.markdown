variables---
layout: post
title:  "Rosserial Arduino Tutorial"
date:   2018-12-08
categories: ros rosserial turtlesim arduino lcd 
---

# Rosserial Arduino Tutorial

### INTRO

Hi,in this tutorial I want to introduce how to create a first project using Arduino and
rosserial.

The goal of this tutorial is to create a project that controlls the turtlesim simulator of ROS.
The controll is done by 4 buttons that allow the turtle to move rigth, left, forward and backward.

The whole code is this one :

~~~cpp
#include <LiquidCrystal.h>
#include <ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
geometry_msgs::Twist movements;

const int BtPin1 = 6;
const int BtPin2 = 7;
const int BtPin3 = 8;
const int BtPin4 = 9;

int RGT = 0;
int LFT = 0;
int FWD = 0;
int BEH = 0;

void turt_cb( const turtlesim::Pose &turt_msg){
  lcd.setCursor(0,0);
  lcd.print("X position :");
  lcd.print(turt_msg.x);

  lcd.setCursor(0,1);
  lcd.print("Y position :");
  lcd.print(turt_msg.y);
}

ros::Subscriber<turtlesim::Pose> sub("/turtle1/pose", &turt_cb);
ros::Publisher pub("/turtle1/cmd_vel", &movements);


void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(BtPin1, INPUT);
  pinMode(BtPin2, INPUT);
  pinMode(BtPin3, INPUT);
  pinMode(BtPin4, INPUT);

  lcd.begin(16, 2); // set up the LCD's number of columns and rows:
}

void loop() {

  RGT = digitalRead(BtPin1);
  LFT = digitalRead(BtPin2);
  FWD = digitalRead(BtPin3);
  BEH = digitalRead(BtPin4);

  if (RGT == HIGH) {
    movements.linear.x = 0;
    movements.angular.z= 1.57;
    pub.publish( &movements );
    delay(1000);
    movements.linear.x = 2;
    movements.angular.z= 0;
    pub.publish( &movements );

  }

  else if(LFT == HIGH) {
    movements.linear.x = 0;
    movements.angular.z= -1.57;
    pub.publish( &movements );
    delay(1000);
    movements.linear.x = 2.0;
    movements.angular.z= 0;
    pub.publish( &movements );

  }

  else if (FWD == HIGH){
    movements.linear.x = 2.0;
    movements.angular.z= 0;
    pub.publish( &movements );
  }

  else if (BEH == HIGH){
    movements.linear.x = -2.0;
    movements.angular.z= 0;
    pub.publish( &movements );

  }
  nh.spinOnce();
}
~~~
## LIBRARIES

~~~cpp
#include <LiquidCrystal.h>
#include <ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
~~~
LiquidCrystal.h is needed for the LCD screen.
ros.h includes all libraries of the ROS system.
Need to include turtlesim/Pose.h because the node /turtle1/pose has values of type turtlesim/Pose.
Need also to include geometry_msgs/Twist.h because the node /turtle1/cmd_vel recaives values of type geometry_msgs/Twist.

## DECLARATION

~~~cpp
ros::NodeHandle  nh;
~~~
Declare a ROS Node.

~~~cpp
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
~~~
Declare the LCD screen.

~~~cpp
geometry_msgs::Twist movements;

const int BtPin1 = 6;
const int BtPin2 = 7;
const int BtPin3 = 8;
const int BtPin4 = 9;

int RGT = 0;
int LFT = 0;
int FWD = 0;
int BEH = 0;
~~~
Declare the variables movements as type geometry_msgs/Twist
Declare the buttons's pins and 4 variables of type int that are the outputs of the 4 buttons.

## CALLBACK FUNCTION

~~~cpp
void turt_cb( const turtlesim::Pose &turt_msg){
  lcd.setCursor(0,0);
  lcd.print("X position :");
  lcd.print(turt_msg.x);

  lcd.setCursor(0,1);
  lcd.print("Y position :");
  lcd.print(turt_msg.y);
}
~~~
This function is needed in the subscriber initialization and prints on the LCD screen the position of the turtle.
lcd.setCursor(0,0) sets the position of the cursor in the screen. Since the 16x2 screen has 16 columns and 2 rows the X position is printed in the fisr row and the Y position on the second row.
The variables turt_msg.x and turt_msg.y contains the X and Y position of the turtle in the simulator.

~~~cpp
ros::Subscriber<turtlesim::Pose> sub("/turtle1/pose", &turt_cb);
ros::Publisher pub("/turtle1/cmd_vel", &movements);
~~~
Declare a subscriber that subscribes to /turtle1/pose and a publisher that publishes on /turtle1/cmd_vel.

## VOID SETUP

~~~cpp
void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(BtPin1, INPUT);
  pinMode(BtPin2, INPUT);
  pinMode(BtPin3, INPUT);
  pinMode(BtPin4, INPUT);
  lcd.begin(16, 2); // set up the LCD's number of columns and rows:
}
~~~
At first need to initialize the ros node, then nedd to advert that we have a publisher and a subsciber called pus and sub.

Initialize the buttons's pin as input pin and the lcd screen as a 16x2 screen.

## VOID LOOP

~~~cpp
void loop() {

  RGT = digitalRead(BtPin1);
  LFT = digitalRead(BtPin2);
  FWD = digitalRead(BtPin3);
  BEH = digitalRead(BtPin4);

  if (RGT == HIGH) {
    movements.linear.x = 0;
    movements.angular.z= 1.57;
    pub.publish( &movements );
    delay(1000);
    movements.linear.x = 2;
    movements.angular.z= 0;
    pub.publish( &movements );

  }

  else if(LFT == HIGH) {
    movements.linear.x = 0;
    movements.angular.z= -1.57;
    pub.publish( &movements );
    delay(1000);
    movements.linear.x = 2.0;
    movements.angular.z= 0;
    pub.publish( &movements );

  }

  else if (FWD == HIGH){
    movements.linear.x = 2.0;
    movements.angular.z= 0;
    pub.publish( &movements );
  }

  else if (BEH == HIGH){
    movements.linear.x = -2.0;
    movements.angular.z= 0;
    pub.publish( &movements );

  }
  nh.spinOnce();
}
~~~

In the void loop is assigned to the 4 variables RGT, LFT, BEH, FWD, the 4 values of the buttons's inputs and then checked which variables is equal to high which means that the corresponding button is pressed.

On the basis of the pressed button is publish on the topic /turtle1/cmd_vel a different linear and angular velocity.
