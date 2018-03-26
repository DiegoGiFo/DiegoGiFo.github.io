---
layout: post
title:  "Rosserial Tutorial (En)"
date:   2018-03-12 14:00
categories: rosserial arduino lcd ros publisher subscriber
---

## INTRO

Hi,in this tutorial I want to introduce how to create a first project using Arduino and
rosserial.
The goal of this tutorial is to create a project that controls the turtlesim simulator of ROS.
The control is done by 4 buttons that allow the turtle to move rigth, left, forward and backward.

## TOPICS

 - The use of a publisher that publishes on the topic /turtle1/cmd_vel the values of the movements based on the button pressed;
 - The use of a subsriber that subscribes to the topic /turtle1/pose from which receives the position of the turtle and prints it on the LCD screen.

This graph may help in the understanding of the program.
![Ros_graph](https://github.com/DiegoGiFo/Turtle_Cnt_Arduino/blob/master/Vs_2/rosgraph.png?raw=true "Figure 1-1")

## MATERIAL

For the relization of this project is needed:

- Arduino uno;
- LCD screen 16 x 2;
- 4 buttons;
- breadboard.

## CIRCUIT

#### LCD
- RS pin to digital PIN 12
- Enable pin to digital PIN 11
- D4 pin to digital PIN 5
- D5 pin to digital PIN 4
- D6 pin to digital PIN 3
- D7 pin to digital PIN 2
- R/W pin to GROUND
- 10K resistor connecting LED+ to GROUND:
- LED- to GROUND
- VO to GROUND

#### BUTTONS
- Rigth button PIN 6
- Left button PIN 7
- Forward button PIN 8
- Behind button PIN 9

Can download the fritzing scheme here: [Scheme](https://github.com/DiegoGiFo/Turtle_Cnt_Arduino/blob/master/Vs_2/Turtle_Cnt.fzz)

## CODE

The whole code is the following one :

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
Need to include turtlesim/Pose.h because the topic /turtle1/pose has values of type turtlesim/Pose.
Need also to include geometry_msgs/Twist.h because the topic /turtle1/cmd_vel receives values of type geometry_msgs/Twist.

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
Declare the variables movements as type geometry_msgs/Twist.
Declare the buttons's pins and 4 variables of type int that are the inputs of the 4 buttons.

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
lcd.setCursor(0,0) sets the position of the cursor in the LCD screen. Since the 16x2 screen has 16 columns and 2 rows the X position is printed in the first row and the Y position on the second row.
The variables turt_msg.x and turt_msg.y contain the X and Y position of the turtle in the simulator.

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
  lcd.begin(16, 2); // set up the LCD's number of columns and rows
}
~~~
At first need to initialize the ros node, then need to advert that we have a publisher and a subsciber called pub and sub.

Initialize the buttons's pins as inputs pins and the lcd screen as a 16x2 screen.

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

In the void loop are assigned to the 4 variables RGT, LFT, BEH, FWD the 4 values of the buttons's inputs and then checked which variables is equal to high which means that the corresponding button is pressed.

On the basis of the pressed button is published on the topic /turtle1/cmd_vel a different linear and angular velocity.
