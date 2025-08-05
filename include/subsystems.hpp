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

inline pros::Motor Front_Left_1(-17); // Bottom Front Left motor
inline pros::Motor Front_Left_2(18); // Top Front Left motor
inline pros::Motor Back_Left_1(-15);  // Bottom Back Left motor
inline pros::Motor Back_Left_2(14);  // Top Back Left motor 
inline pros::Motor Front_Right_1(19); // Bottom Front Right motor
inline pros::Motor Front_Right_2(-20); // Top Front Right motor
inline pros::Motor Back_Right_1(12);  // Bottom Back Right motor
inline pros::Motor Back_Right_2(-11);  // Top Back Right motor

inline pros::Motor Intake(21); // Intake motor

/**
Motor groups for robot
*/

inline pros::MotorGroup All_Drive({19, 20, 18, 10, -13, -12, -15, -2}); // All drive motors
inline pros::MotorGroup Left_side({19, 20, 18, 10}); // Left side motor group
inline pros::MotorGroup Right_side({-13, -12, -15, -2}); // Right side motor group

/**
Sensors
*/

inline pros::Imu Inertial(6); // IMU sensor on port 6

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');