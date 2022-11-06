#include "distance_sensor.h"
#include "robot_interface.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setup_controller();
  reset_pos();
  Serial.println("Enter command or destination (h for help): ");
}

void loop()
{  
  get_robot_command();   
}
