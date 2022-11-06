#include "SoftwareSerial.h"
#include "LobotServoController.h"

#define RxPin 8    //Define soft serial port
#define TxPin 9
#define LED   13
#define US    A0   //Ultrasonic sensor

int result;        //Record the number of touches 
double distance;

double max_dis;
int max_dis_loc;
int max_dis_or;

SoftwareSerial mySerial(RxPin, TxPin);
LobotServoController myController(mySerial);
LobotServo servos[6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  //myController.runActionGroup(0, 1);
  //delay(1500);
  
  servos[0].ID = 1;
  servos[1].ID = 2;
  servos[2].ID = 3;
  servos[3].ID = 4;
  servos[4].ID = 5;
  servos[5].ID = 6;

  max_dis_loc = 0;
  max_dis_or = 500;
  max_dis = 0;
  
}

void reset_pos(int wait) {
  servos[0].Position = 0;
  for(int i=1; i<6; i++) {
    servos[i].Position = 500;
  }
  myController.moveServos(servos, 6, 1000);

  do
    delay(1000);
  while(wait);
}

void measure_distance(int loc, int dir) {
  distance = analogRead(US);
  Serial.println(distance);
  if(distance > max_dis) {
    max_dis = distance;
    max_dis_loc = loc;
    max_dis_or = dir;
  }
}


void run()
{
  myController.moveServos(4,1000,3,835,4,950,5,265,6,120);
  //myController.moveServo(6,0,500);

  for(int i=120; i<880; i+=2) {
    myController.moveServo(6,i,1);
    measure_distance(i, 120);
  }
  
  Serial.println(max_dis_loc);
  Serial.println(max_dis_or);
  myController.moveServos(2, 500, 3, max_dis_or, 6,max_dis_loc);

  delay(1000);
  myController.moveServo(5, 190, 500);
  delay(1000);
  myController.moveServo(1, 1000, 100);
  delay(750);
  myController.moveServo(5, 500, 500);
  
  while(1) {
    delay(1000);
  }
  //reset_pos(1);
}

void loop() {
  reset_pos(0);
  //reset_pos(1);
  // put your main code here, to run repeatedly:
  static uint32_t timer_L;
  myController.receiveHandler();
  run();   
}
