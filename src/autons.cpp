#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EncoderPIDAutos.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(5.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 23.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions


  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void AutonTesting(){
  driveForward_EncoderPID2(42, 0, DRIVE_SPEED, 50, 5000);
  

  
}

void soloAWP(){
}

void RightSideAuton(){
  FrontIntake.move(127);  // Spin the intake 

  turnToHeading_IMUPID(24, TURN_SPEED, 800);

  driveForward_EncoderPID(23, 24, 100, TURN_SPEED, 1500);

  TongueMech.set_value(1);

  driveForward_EncoderPID(21, 20, 50, TURN_SPEED, 2000);

  driveForward_EncoderPID(-36, 20, DRIVE_SPEED, 50, 2000);

  turnToHeading_IMUPID(90, TURN_SPEED, 1200);

  driveForward_EncoderPID2(35, 90, 85, TURN_SPEED, 3500);

  turnToHeading_IMUPID(180, TURN_SPEED, 1200);

  driveForward_EncoderPID(-25, 180, 60, TURN_SPEED, 2000);

  TongueMech.set_value(0);

  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);

  driveForward_EncoderPID(12, 0,DRIVE_SPEED, TURN_SPEED, 3000);

  // TongueMech.set_value(1);

  // driveForward_EncoderPID(50, 180, 80, TURN_SPEED, 5000);


  

}

void LeftSideAuton(){

  FrontIntake.move(127);  // Spin the intake 

  turnToHeading_IMUPID(-35, 50, 1500);

  driveForward_EncoderPID(20, -35, 50, TURN_SPEED, 3000);

  TongueMech.set_value(1);

  driveForward_EncoderPID(22, -35, 50, TURN_SPEED, 3000);

  driveForward_EncoderPID(-28, -35, 40, 50, 5000);

  turnToHeading_IMUPID(-95, 50, 1500);

  driveForward_EncoderPID(32, -95, 50, TURN_SPEED, 3000);

  TongueMech.set_value(0);

  turnToHeading_IMUPID(-89, 50, 1000);

  driveForward_EncoderPID(-25, -177, 50, TURN_SPEED, 3000);

  Arm.move_absolute(-700, 180);
  pros::delay(1200);
  Arm.move_absolute(0, 130);

  driveForward_EncoderPID(5, 180, 50, TURN_SPEED, 3000);

}


void Skills(){
  FrontIntake.move(127);  // Spin the intake 

  turnToHeading_IMUPID(25, 50, 1000);

  driveForward_EncoderPID(20, 25, 50, TURN_SPEED, 2000);

  TongueMech.set_value(1);

  driveForward_EncoderPID(22, 25, 50, TURN_SPEED, 3000);

  driveForward_EncoderPID(-35, 25, 40, 50, 5000);

  turnToHeading_IMUPID(90, 50, 1200);

  driveForward_EncoderPID(31, 90, 50, TURN_SPEED, 3000);

  turnToHeading_IMUPID(180, 50, 1200);

  driveForward_EncoderPID(-27, 180, 50, TURN_SPEED, 2000);

  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);
  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);

  TongueMech.set_value(1);


  turnToHeading_IMUPID(175, TURN_SPEED, 1000);

  driveForward_EncoderPID(34, 175, 50, TURN_SPEED, 10000);

  pros::delay(500);

  driveForward_EncoderPID(-2, 175, 50, TURN_SPEED, 1000);

  driveForward_EncoderPID(2, 175, 50, TURN_SPEED, 1200);

  driveForward_EncoderPID(-20, 175, 50, TURN_SPEED, 2000);

  turnToHeading_IMUPID(85, TURN_SPEED, 1000);

  driveForward_EncoderPID(7, 85, 50, TURN_SPEED, 1200);

  TongueMech.set_value(0);

  turnToHeading_IMUPID(0, TURN_SPEED, 1000);

  driveForward_EncoderPID(80, 0, 50, TURN_SPEED, 6000);

  turnToHeading_IMUPID(-90, TURN_SPEED, 1000);

  driveForward_EncoderPID(9, -90, 50, TURN_SPEED, 4000);

  turnToHeading_IMUPID(0, TURN_SPEED, 1000);

  driveForward_EncoderPID(-12, 0, 50, TURN_SPEED, 4000);

  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);
  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);

  TongueMech.set_value(1);


  turnToHeading_IMUPID(0, TURN_SPEED, 1000);

  driveForward_EncoderPID(34, 0, 50, TURN_SPEED, 10000);

  pros::delay(500);

  driveForward_EncoderPID(-2, 0, 50, TURN_SPEED, 1000);

  driveForward_EncoderPID(2, 0, 50, TURN_SPEED, 1200);

  driveForward_EncoderPID(-20, 0, 50, TURN_SPEED, 2000);

  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);
  Arm.move_absolute(-700, 170);
  pros::delay(1200);
  Arm.move_absolute(0, 200);
  pros::delay(1000);


}

void OffParkAuton(){
  driveStrafe_EncoderPID(-5, 0, 50, 50, 3000);
}

void SkillsSafe(){
  FrontIntake.move(-127);  // Spin the intake
  pros::delay(1000);
  chassis.pid_drive_set(21,70);

  FrontIntake.move(-127);  // Spin the intake
  pros::delay(1000);
}