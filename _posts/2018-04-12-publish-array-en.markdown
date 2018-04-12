---
layout: post
title:  "Publish array tutorial (En)"
date:   2018-4-12 11:20
categories: ros array multiarray
---

# PUBLISH ARRAY TUTORIAL
Since I tried to publish an array on a ROS node but it was very difficult to find a tutorial I decided
to write one.
In this tutorial is explained how to publish an array of type std_msg/Float32MultiArray.

## CODE
~~~cpp
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::Float32MultiArray msg_arr;
ros::Publisher msg_arr_pub("/array_msg", &msg_arr);

char dim0_label[] = "message_array";
void setup()
{
  nh.initNode();
  msg_arr.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  msg_arr.data_length = 8;
  msg_arr.layout.dim[0].label = dim0_label;
  msg_arr.layout.dim[0].size = 8;
  msg_arr.layout.dim[0].stride = 1*8;
  msg_arr.layout.data_offset = 0;
  msg_arr.data = (float *)malloc(sizeof(float)*8);
  nh.advertise(msg_arr_pub);
}

void loop()
{
  for(int i = 0; i < 8; i++){
    msg_arr.data[i] = i ;
  }

  if((millis()%50) == 1)
  {
    msg_arr_pub.publish( &msg_arr );
  }

  nh.spinOnce();
}
~~~

Now let's analyze code.

## LIBRARIES
~~~cpp
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
~~~
Include the ros_lib and the std_msgs/Float32MultiArray that allows to create an array of type float.

## DECLARATION
~~~cpp
std_msgs::Float32MultiArray msg_arr;
ros::Publisher msg_arr_pub("/array_msg", &msg_arr);
~~~
Declare a variable of type Float32MultiArray and then declare a publisher named /array_msg on which we will
publish the array.

~~~cpp
char dim0_label[] = "message_array";
~~~
Declaration of the array's label.

## VOID SETUP
~~~cpp
nh.initNode();
msg_arr.layout.dim = (std_msgs::MultiArrayDimension *)
malloc(sizeof(std_msgs::MultiArrayDimension)*2);
msg_arr.data_length = 8;
msg_arr.layout.dim[0].label = dim0_label;
msg_arr.layout.dim[0].size = 8;
msg_arr.layout.dim[0].stride = 1*8;
msg_arr.layout.data_offset = 0;
msg_arr.data = (float *)malloc(sizeof(float)*8);
nh.advertise(msg_arr_pub);
~~~
Initialize the ROS node.
Allocate the layout dimension of the array, in this case twice as the dimension of the type std_msg/MultiArrayDimension.
Set the length of the array's data, in this case 8.
Set the label of the array's layout, in this case the previous defined variable dim0_label[].
Set the size of the array's layout, in this case 8.

~~~cpp
msg_arr.layout.dim[0].stride = 1*8;
msg_arr.layout.data_offset = 0;
~~~
????????

Allocate memory for the array's data, in this case 8 times the dimension of the type float since we have 8 values in the array.
Advertise the system that we have a publisher called msg_arr_pub.

## VOID LOOP
~~~cpp
for(int i = 0; i < 8; i++){
  msg_arr.data[i] = i ;
}

if((millis()%50) == 1)
{
  msg_arr_pub.publish( &msg_arr );
}

nh.spinOnce();
~~~
Use a cycle for to assign a value for each cell of the array.
In this example the value of each cell is equal to the cell index.
The array msg_arr is not published continuously but only every 50 milliseconds.
