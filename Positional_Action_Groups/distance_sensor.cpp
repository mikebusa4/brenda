#include "distance_sensor.h"

float measure_distance() {
  double distance;
  distance = analogRead(US_PIN);

  //TODO: use equation from excel to turn reading into inches
  return distance;
}
