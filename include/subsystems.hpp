#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
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
inline pros::Motor MatchLoad(9);
/**
Motor groups for robot
*/

inline pros::MotorGroup FrontLeft({-17,18}); // Front left motors
inline pros::MotorGroup FrontRight({19,-20}); // Front right motors
inline pros::MotorGroup BackLeft({-15,14}); // Back left motors
inline pros::MotorGroup BackRight({12,-11}); // Back right motors


/**
Sensors
*/

inline pros::Imu IMU(6); // IMU sensor on port 6

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

/**
Pneumatics
*/
inline pros::adi::DigitalOut IntakeLift('A');
inline pros::adi::DigitalOut DescoreLeft('B');
inline pros::adi::DigitalOut DescoreRight('C');
inline pros::adi::DigitalOut ScorePiston('D');