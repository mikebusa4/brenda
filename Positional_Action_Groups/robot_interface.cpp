#include "robot_interface.h"

SoftwareSerial mySerial(RxPin, TxPin);
LobotServoController myController(mySerial);
LobotServo servos[6];

int stored_positions = 0;
stored_position sequence[50];

void setup_controller() {

    mySerial.begin(9600);
    
    servos[0].ID = 1;
    servos[1].ID = 2;
    servos[2].ID = 3;
    servos[3].ID = 4;
    servos[4].ID = 5;
    servos[5].ID = 6;
}

bool pos_is_legal(int j6, int j5, int j4, int j3) {
    if((j6>=0 && j6<=1000) && (j5>=125 &&  j5<=875) && (j4>=0 && j4<=1000) && (j3>=65 && j3<=1000))
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

bool set_servo_pos_from_cartesian(float x, float y, float z) {
    //Set Default Positions for gripper, gripper roll, and turret
    servos[1].Position = 500;

    //TODO: keep track of current position to calculate total motor distance from A->B
    int min_travel;

    int phi = 0;
    float t1, t2, t3;
    int j3, j4, j5, j6;
    bool valid = false;

    // Z 
    float gamma = degrees(atan(z / x));
    if (x<0 and z<0)
        gamma = -180 + gamma;
    else if (x<0)
        gamma = 180 + gamma;

    if(gamma > 120) {
        gamma -= 180;
    }
    else if(gamma < -120) {
        gamma+=180;
    }

    j6 = 500 - (gamma/degrees_per_tick);

    float d = sqrt(x*x + z*z);
    if (x < 0)
        d*=-1;
    
    while(phi < 360) {
        t2 = degrees(acos((pow((d - L3*cos(radians(phi))),2) + pow((y - L3*sin(radians(phi))),2) - L1*L1 - L2*L2) / (2*L1*L2)));
retry:
        t1 = degrees(acos(((L1 + L2*cos(radians(t2)))*(d - L3*cos(radians(phi))) + (L2*sin(radians(t2)))*(y - L3*sin(radians(phi)))) / (pow((d - L3*cos(radians(phi))),2) + pow((y - L3*sin(radians(phi))),2))));
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

        if (pos_is_legal(j6, j5, j4, j3) && verify_pos(t1, t2, t3, d, y)) {
          Serial.print("Phi: ");
          Serial.println(phi);
          Serial.print("J6: ");
          Serial.println(j6);
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
        servos[5].Position = j6;
    } else {
        Serial.println("EE Position not possible");
    }
    return valid;
}

void get_robot_command() {
    float x=0, y=0, z=0;
    myController.receiveHandler();
    x = get_float_from_serial();
    if(!isfinite(x)) {
        Serial.read();
        Serial.flush();
        return;
    }

    y = get_float_from_serial();
    z = get_float_from_serial();

    stored_position collected;
    collected.x = x;
    collected.y = y;
    collected.z = z;

    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.println(z);
    

    sequence[stored_positions++] = collected;
    
    Serial.read();
    Serial.flush();
}

// Robot Actions
void move_servos(int time_ms) {
    Serial.println("Moving BRENDA...");
    myController.moveServos(&servos[1], 6, time_ms);
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

void reset_pos() {
    Serial.println("Resetting servos...");
    servos[0].Position = 0;
    for(int i=1; i<6; i++) {
        servos[i].Position = 500;
    }
    myController.moveServos(servos, 6, 2000);

    delay(2000);
    Serial.println("Done\n");
}

void execute() {
    for(int i=0; i<stored_positions; i++){
        if(sequence[i].x == 100) {
          close_gripper();
          continue;
        }
        if(sequence[i].x == -100) {
          open_gripper();
          continue;
        }
        stored_position to_run = sequence[i];
        if(set_servo_pos_from_cartesian(to_run.x,to_run.y,to_run.z)) {
            move_servos(2000);
        }
    }
    stored_positions = 0;
}

//Serial Interface

void printHelp() {
  Serial.println("\nAcceptable Commands:");
  Serial.println("\th - Help");
  Serial.println("\tc - Close gripper");
  Serial.println("\to - Open gripper");
  Serial.println("\tr - Reset servos\n");
  Serial.println("\tg - Execute Sequence of Commands\n");
  Serial.println("Or enter an x and y destination separated by a space (ex: \"4.5 1\")\n");
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
        stored_position temp = {100,0,0};
        sequence[stored_positions++] = temp;
        Serial.println("Close");
        //close_gripper();
        break;
      }

      else if(incomingByte == (int)'o') {
        stored_position temp = {-100,0,0};
        sequence[stored_positions++] = temp;
        Serial.println("Open");
        //open_gripper();
        break;
      }

      else if(incomingByte == (int)'r') {
        reset_pos();
        break;
      }

      else if(incomingByte == (int)'g') {
        execute();
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
  Serial.flush();
  return (float) 1/0;
}
