#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <Arduino.h>

#define US_PIN 0   //Ultrasonic sensor pin

/**
 * @brief Measure disance from the ultrasonic sensor
 * 
 * @return float measured distance in inches
 */
float measure_distance();

#endif
