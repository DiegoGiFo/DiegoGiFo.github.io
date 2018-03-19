---
layout: post
title:  "Rosserial Tutorial (Ita)"
date:   2018-03-12 14:00
categories: rosserial arduino lcd ros publisher subscriber
---

## INTRO

In questo tutorial voglio spiegarvi come creare un semplice progetto basato su rosserial usando Arduino.
L'obiettivo di questo tutorial è quello di creare un progetto in grado di controllare il simulatore turtlesim di ROS.
La tartaruga viene controllata da 4 pulsanti che la fanno muovere a destra,sinistra,avanti ed indietro.

## CONTENUTI

 - L'uso di un publisher che pubblica sul topic /turtle1/cmd_vel i movimenti da far compiere alla tartaruga basandosi sul pulsante che si è premuto;
 - L'uso di un subscriber che si sottoscrive al topic /turtle1/pose dal quale riceve la posizione della tartaruga e la mostra sullo schermo LCD.

![Ros_graph](https://github.com/DiegoGiFo/Turtle_Cnt_Arduino/blob/master/Vs_2/rosgraph.png?raw=true "Figure 1-1")

## MATERIALE

Per la realizzazione di questo progetto serve:

- Arduino Uno;
- schermo LCD 16 x 2;
- 4 bottoni;
- breadboard.

## CIRCUITO

#### SCHERMO LCD
- RS pin -> PIN 12
- Enable pin -> PIN 11
- D4 pin -> PIN 5
- D5 pin -> PIN 4
- D6 pin -> PIN 3
- D7 pin -> PIN 2
- R/W pin -> GROUND
- reistenza da 10K connessa tra LED+ e GROUND:
- LED- -> GROUND
- VO -> GROUND

#### BOTTONI
- Bottone DESTRO -> PIN 6
- bottone SINISTRO -> PIN 7
- bottone AVANTI -> PIN 8
- bottone INDIETRO -> PIN 9

E' possibile scaricare lo schema del circuito a questo link: [Scheme](https://github.com/DiegoGiFo/Turtle_Cnt_Arduino/blob/master/Vs_2/Turtle_Cnt.fzz)

## CODICE

Di seguito è mostrato il codice completo :

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
LiquidCrystal.h è necessaria per il funzionamento dello schermo LCD.
ros.h include tutte le librerie del sistema ROS.
Dobbiamo includere turtlesim/Pose.h perchè il nodo /turtle1/pose ha valori del tipo turtlesim/Pose.
Dobbiamo anche includere geometry_msgs/Twist.h perchè il nodo /turtle1/cmd_vel riceve valuri di tipo geometry_msgs/Twist.

## DICHIARAZIONI

~~~cpp
ros::NodeHandle  nh;
~~~
Dichiarazione del nodo ROS.

~~~cpp
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
~~~
Dichiarazione dello schermo LCD.

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
Dichiarazione della variabile movements di tipo geometry_msgs/Twist
Dichiarazione dei pin ai quali sono connessi i pulsanti, sono di tipo int e corrispondono a variabili di input.

## FUNZIONE DI CALLBACK

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
Questa funzione verrà poi richiamata nell'inizializzazione del subscriber e mostra tramite lo schermo LCD la posizione della tartaruga.
lcd.setCursor(0,0) inizializza la posizione del cursore nello schermo. Sche che lo schermo è 16x2 ha 16 colonne e 2 righe. La posizione X è mostrata nella prima riga e la posizione Y nella seconda.
Le variabili turt_msg.x e turt_msg.y contengono la posizione X e Y della tartaruga nel simulatore.

~~~cpp
ros::Subscriber<turtlesim::Pose> sub("/turtle1/pose", &turt_cb);
ros::Publisher pub("/turtle1/cmd_vel", &movements);
~~~
Dichiarazione del subscriber che s sottoscrive a /turtle1/pose e di un publisher che pubblica su /turtle1/cmd_vel.

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
per prima cosa si deve inizializzare il nodo ROS, successivamente si deve indicare che sono presenti un subsciber e un publisher.

Initializare i pin dei bottoni come input pin e lo schermo LCD di dimensioni 16x2.

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

Nel void loop le variabili RGT, LFT, BEH, FWD sono assegnate ai 4 valori di input provenienti dai rispettivi bottoni e viene controllato quale delle 4 variabili ha valore HIGH.
In base a quale variabile ha il valore HIGH è possibile capire quale bottone sia stato premuto e pubblicare sul topic /turtle1/cmd_vel una velocità angolare e lineare diversa a seconda della direzione in cui la tartaruga si deve muovere.
