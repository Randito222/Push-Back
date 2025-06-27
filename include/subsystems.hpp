#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

/**
Motors for robot
*/

inline pros::Motor Front_Left_1(19); // Bottom Front Left motor
inline pros::Motor Front_Left_2(20); // Top Front Left motor
inline pros::Motor Back_Left_1(18);  // Bottom Back Left motor
inline pros::Motor Back_Left_2(10);  // Top Back Left motor
inline pros::Motor Front_Right_1(-13); // Bottom Front Right motor
inline pros::Motor Front_Right_2(-12); // Top Front Right motor
inline pros::Motor Back_Right_1(-15);  // Bottom Back Right motor
inline pros::Motor Back_Right_2(-2);  // Top Back Right motor

/**
Motor groups for robot
*/

inline pros::MotorGroup All_Drive({19, 20, 18, 10, -13, -12, -15, -2}); // All drive motors
inline pros::MotorGroup Left_side({19, 20, 18, 10}); // Left side motor group
inline pros::MotorGroup Right_side({-13, -12, -15, -2}); // Right side motor group

/**
Sensors
*/

inline pros::Imu IMU(9); // IMU sensor on port 9


// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');