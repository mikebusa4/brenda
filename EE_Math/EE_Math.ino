#include "SoftwareSerial.h"
#include "LobotServoController.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#define RxPin 8    //Define soft serial port
#define TxPin 9
#define LED   13
#define US    A0   //Ultrasonic sensor

SoftwareSerial mySerial(RxPin, TxPin);
LobotServoController myController(mySerial);
LobotServo servos[6];

// Arm Constants
const float L1 = 4;
const float L2 = 3.75;
const float L3 = 5.5; // Middle of gripper, change to 6.5 for end of gripper
const float degrees_per_tick = 0.24;

void printHelp() {
  Serial.println("\nAcceptable Commands:");
  Serial.println("\th - Help");
  Serial.println("\tc - Close gripper");
  Serial.println("\to - Open gripper");
  Serial.println("\tr - Reset servos\n");
  Serial.println("Or enter an x and y destination separated by a space (ex: \"4.5 1\")\n");
}


void measure_distance() {
  double distance;
  distance = analogRead(US);
  Serial.println(distance);
}

void move_servos(int time_ms){
  Serial.println("Moving Brenda...");
  myController.moveServos(servos, 6, time_ms);
  delay(time_ms);
  Serial.println("Done\n");
}

void close_gripper() {
  Serial.println("Closing gripper...");
  myController.moveServo(1, 1000, 1000);
  delay(1000);
  Serial.println("Done\n");
}

void open_gripper() {
  Serial.println("Opening gripper...");
  myController.moveServo(1, 0, 1000);
  delay(1000);
  Serial.println("Done\n");
}

void reset_pos(int wait) {
  Serial.println("Resetting servos...");
  servos[0].Position = 0;
  for(int i=1; i<6; i++) {
    servos[i].Position = 500;
  }
  myController.moveServos(servos, 6, 1000);

  do
    delay(1000);
  while(wait);
  Serial.println("Done\n");
}

bool pos_is_legal(int j5, int j4, int j3) {
  if((j5>=125 and  j5<=875) and (j4>=0 and j4<=1000) and (j3>=65 and j3<=1000))
    return true;
  else
    return false;
}

bool verify_pos(float t1, float t2, float t3, float x, float y) {
  float xee = L1*cos(radians(t1)) + L2*cos(radians(t1 + t2)) + L3*cos(radians(t1 + t2 + t3));
  float yee = L1*sin(radians(t1)) + L2*sin(radians(t1 + t2)) + L3*sin(radians(t1 + t2 + t3));

  if (abs(xee-x) < .01 or abs(yee-y) < .01)
    return true;
  else
    return false;
}

float get_float_from_serial() {
  int incomingByte = 0; // for incoming serial data
  char num_as_str[10] = {};
  
  // send data only when you receive data:
  int chars_written = 0;
  while(1) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();

      // Special Cases
      if(incomingByte == (int)'h') {
        printHelp();
        break;
      }
      else if(incomingByte == (int)'c') {
        close_gripper();
        break;
      }

      else if(incomingByte == (int)'o') {
        open_gripper();
        break;
      }

      else if(incomingByte == (int)'r') {
        reset_pos(0);
        break;
      }
      
      // If there is space in the buffer and the incoming byte is a digit 0-9 (ASCII 48-57), a negative sign (ASCII 45), or a decimal point(ASCII 46)
      else if(chars_written < sizeof(num_as_str) && (incomingByte>=48 && incomingByte<=57 || incomingByte == 45 || incomingByte == 46)) {
        num_as_str[chars_written++] = (char)incomingByte;
      }
      else {
        chars_written = 0;
        return atof(num_as_str);
      }
    }
  }
  // Flush input buffer
  Serial.read();
  return (float) 1/0;
}

bool set_servo_pos_from_cartesian(float x, float y) {
  //Set Default Positions for gripper, gripper roll, and turret
  servos[0].Position = 0;
  servos[1].Position = 500;
  servos[5].Position = 500;

  int phi = 0;
  float t1, t2, t3;
  int j3, j4, j5;
  bool valid = false;
  
  while(phi < 360) {
    t2 = degrees(acos((pow((x - L3*cos(radians(phi))),2) + pow((y - L3*sin(radians(phi))),2) - L1*L1 - L2*L2) / (2*L1*L2)));
retry:
    t1 = degrees(acos(((L1 + L2*cos(radians(t2)))*(x - L3*cos(radians(phi))) + (L2*sin(radians(t2)))*(y - L3*sin(radians(phi)))) / (pow((x - L3*cos(radians(phi))),2) + pow((y - L3*sin(radians(phi))),2))));
    if(t2 != t2 || t1 != t1) {
      phi++;
      continue;
    }
    t3 = phi - (t1 + t2);
    
    j5 = ((t1)/degrees_per_tick) + 125;
    j4 = 500 - (t2/degrees_per_tick);
    j3 = ((t3/degrees_per_tick) + 500);
    while(j3 > 1500) {
      j3-=1500;
    }

    if (pos_is_legal(j5, j4, j3) && verify_pos(t1, t2, t3, x, y)) {
      Serial.print("Phi: ");
      Serial.println(phi);
      Serial.print("J5: ");
      Serial.println(j5);
      Serial.print("J4: ");
      Serial.println(j4);
      Serial.print("J3: ");
      Serial.println(j3);
      valid = true;
      break;
    }
    else if (t2 > 0){
      t2*=-1;
      goto retry;     
    }
    phi++;
  }

  if(valid) {
    servos[2].Position = j3;
    servos[3].Position = j4;
    servos[4].Position = j5;
  } else {
    Serial.println("EE Position not possible");
  }
  return valid;
}

bool get_robot_command(float &x, float &y) {
  x = get_float_from_serial();
  if(!isfinite(x)) {
    return false;
  }
  y = get_float_from_serial();
  Serial.flush();

  return true;
}

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

  reset_pos(0);
  
}

void run()
{  
  Serial.println("Enter command or destination (h for help): ");

  float x=0, y=0;
  bool pos_given = get_robot_command(x, y);
  if(pos_given) {
      if(set_servo_pos_from_cartesian(x,y)) {
      move_servos(1500);
    }
  }   
}

void loop() {
  //reset_pos(1);
  // put your main code here, to run repeatedly:
  static uint32_t timer_L;
  myController.receiveHandler();
  run();  
}
