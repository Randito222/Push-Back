#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

/**
Motors for robot
*/

inline pros::Motor Front_Left_1(-14); // Bottom Front Left motor
inline pros::Motor Front_Left_2(13); // Top Front Left motor
inline pros::Motor Back_Left_1(-12);  // Bottom Back Left motor
inline pros::Motor Back_Left_2(11);  // Top Back Left motor 
inline pros::Motor Front_Right_1(18); // Bottom Front Right motor
inline pros::Motor Front_Right_2(-17); // Top Front Right motor
inline pros::Motor Back_Right_1(19);  // Bottom Back Right motor
inline pros::Motor Back_Right_2(-20);  // Top Back Right motor

inline pros::Motor FrontIntake(1); // Intake motor
inline pros::Motor Arm(10); // Arm motor
inline pros::Motor MiddleIntake(0);
inline pros::Motor TopIntake(0);
/**
Motor groups for robot
*/

inline pros::MotorGroup FrontLeft({-14,13}); // Front left motors
inline pros::MotorGroup FrontRight({-17,18}); // Front right motors
inline pros::MotorGroup BackLeft({-12,11}); // Back left motors
inline pros::MotorGroup BackRight({19,-20}); // Back right motors


/**
Sensors
*/

inline pros::Imu IMU(6); // IMU sensor on port 6
//inline pros::Imu IMU2(2); // IMU sensor on port 2
inline pros::Optical OP1(7); // Optical sensor on port 7
inline pros::Optical OP2(8); // Optical sensor on port 9
inline pros::Rotation LVerticalTracker(15); // Left Horizontal tracking wheel on port 15
inline pros::Rotation RVerticalTracker(9); // Right Horizontal tracking wheel on port 9
inline pros::Rotation HorizontalTracker(16); // Vertical tracking wheel on port 16

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

/**
Pneumatics
*/
inline pros::adi::DigitalOut TongueMech('B');
inline pros::adi::DigitalOut Descore('D');
inline pros::adi::DigitalOut DescoreLift('C');
inline pros::adi::DigitalOut IntakeLift('A');
