#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "LobotServoController.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

#define RxPin 8    //Define soft serial port
#define TxPin 9

/**
 * @brief struct to hold x,y,z positions for sequencing array
 */
struct stored_position {
  float x;
  float y;
  float z;
};

// Arm Constants
const float L1 = 4;
const float L2 = 3.75;
const float L3 = 5.5; // Middle of gripper, change to 6.5 for end of gripper
const float degrees_per_tick = 0.24;

/**
 * @brief Set the servo ids and start mySerial on setup
 */
void setup_controller();

/**
 * @brief Use forward kinematics to verify the calculated joint angles result in the desired position
 * 
 * @param t1 theta1 (Link 1 from horizontal)
 * @param t2 theta2 (Link 2 from link 1)
 * @param t3 theta3 (Link 3 from link 2)
 * @param x desired x position
 * @param y desired y position
 * @return true position matches
 * @return false position fails
 */
bool verify_pos(float t1, float t2, float t3, float x, float y);

/**
 * @brief Verify the servo positions are legal 
 * 
 * @param j6 turret position
 * @param j5 shoulder position
 * @param j4 elbow 1 position
 * @param j3 elbow 2 position
 * @return true all joint positions are legal
 * @return false at least one joint position is not legal
 */
bool pos_is_legal(int j6, int j5, int j4, int j3);

/**
 * @brief calculate the servo positions from given (x,y,z) coordinates
 * 
 * @param x desired x
 * @param y desired y
 * @param z desired z
 * @return true position is possible for BRENDA to reach
 * @return false position is not possible for BRENDA to reach
 */
bool set_servo_pos_from_cartesian(float x, float y, float z);

/**
 * @brief Get a command from the serial interface
 */
void get_robot_command();

/**
 * @brief request the servos to move and delay as long as the movement takes
 * 
 * @param time_ms delay time in milliseconds
 */
void move_servos(int time_ms);

/**
 * @brief Close the gripper
 */
void close_gripper();

/**
 * @brief Open the gripper
 */
void open_gripper();

/**
 * @brief Reset all servo positions to 500 (arm straight up)
 */
void reset_pos();

/**
 * @brief Execute sequence of commands given by user
 */
void execute();


/**
 * @brief Print list of commands and descriptions
 */
void printHelp();

/**
 * @brief Convert serial string input to float
 * 
 * @return inf - special case, user entered command instead of destination
 * @return float - desired coordinate as float
 */
float get_float_from_serial();

#endif
