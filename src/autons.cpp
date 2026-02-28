#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "EncoderPIDAutos.hpp"
#include "XDriveAutos.hpp"
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

  

  
}

void soloAWP(){
}

void RightSideAuton(){
  FrontIntake.move(127);  // Spin the intake
  driveToPoint_XDrive_PID(10,15,0,127,127,1300); // Goes to the three balls near the center
  TongueMech.set_value(1); //Brings the tongue out to hold the balls
  pros::delay(200);

  driveToPoint_XDrive_PID(10,30,0,127,127,2000); // Goes foward to intake the balls
  TongueMech.set_value(0); // Brings the tongue back up

  driveToPoint_XDrive_PID(2, 33, 0, 127, 127, 1000); // Goes to lower goal
  turnToHeading_IMUPID(-55, 127, 500);
  FrontIntake.move(-127); // Spit the balls out into lower goal
  pros::delay(2000);
  FrontIntake.move(127); // Spin the intake back on

  driveToPoint_XDrive_PID(20, 2, 0, 127, 127, 1500); // Goes to lower goal
  driveToPoint_XDrive_PID(38, 2, 0, 110, 127, 1200); // Goes to lower goal
  turnToHeading_IMUPID(-180, 127, 1000);
  TongueMech.set_value(1); //Brings the tongue out to get balls out

  driveToPoint_XDrive_PID(38, -8, -182, 127, 127, 800); // Goes to match loader
  pros::delay(1000); //waits for the match loader to load the balls

  driveToPoint_XDrive_PID(37,33, -180, 127, 127, 1500); // Goes to long goal

  FrontIntake.move(-127);
  Arm.move_absolute(-700, 200);
  pros::delay(1000);
  Arm.move_absolute(0, 200);

  driveToPoint_XDrive_PID(35,1, -180, 127, 127, 2000); // Gets ready to wing
  turnToHeading_IMUPID(0, 127, 1000);


  

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

void RightElimsAuton(){
  FrontIntake.move(127);  // Spin the intake
  driveToPoint_XDrive_PID(12,15,0,127,127,2000); // Goes to the three balls near the center
  TongueMech.set_value(1); //Brings the tongue out to hold the balls
  pros::delay(500);

  driveToPoint_XDrive_PID(12,25,0,127,127,2000); // Goes foward to intake the balls
  TongueMech.set_value(0); // Brings the tongue back up

  driveToPoint_XDrive_PID(6, 28, -45, 127, 127, 1000); // Goes to lower goal

  driveToPoint_XDrive_PID(40, 2, -180, 127, 127, 4000); // Goes to lower goal
  TongueMech.set_value(1); //Brings the tongue out to get balls out
  
  driveToPoint_XDrive_PID(40,10, -182, 127, 127, 1000); // Goes to long goal
  
  Arm.move_absolute(-700, 200);
  pros::delay(1000);
  Arm.move_absolute(0, 200);
  
  driveToPoint_XDrive_PID(40,1, 0, 127, 127, 2000); // Gets ready to wing
}

void OffParkAuton(){
  driveStrafe_EncoderPID(-5, 0, 50, 50, 3000);
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

void SkillsSafe(){
  FrontIntake.move(-127);  // Spin the intake
  pros::delay(1000);
  chassis.pid_drive_set(21,70);

  FrontIntake.move(-127);  // Spin the intake
  pros::delay(1000);
}