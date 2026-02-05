#include "autons.hpp"
#include "Auto.hpp"

#include "Drive.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "main.h"
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

// . . .
// Make your own autonomous functions here!
// . . .


void AutonTesting(){

  resetOdom(drive);
  startHeadingTask(drive, 10);  // if not already running
  driveToPoint(drive, 20, 0, 0, 0.9, 0.7, 2500);



  // FrontIntake.move(127); // Spins intake to grab preload
  // DriveToPoint_OdomPID(12,8,-45, 100, 1500, 40); 

  // FrontIntake.move(127); // Spins intake to grab preload
  // DriveToPoint_OdomPID(12,9,-45, 100, 1500, 40); 

  // DriveToPoint_OdomPID(27, 20, -45, 30, 2000, 20);
  // pros::delay(50);

  // DriveToPoint_OdomPID(27, 20, -130, 60, 2000, 40);

  // IntakeLift.set_value(1); // Lifts intake to score
  // DriveToPoint_OdomPID(14, 26, -130, 80, 3000, 30);
  // Arm.move_absolute(-900,45);  // Stop the intake motor when B is pressed

  // pros::delay(900); // Waits to make sure preload is out
  // IntakeLift.set_value(0); // Lifts intake to score
  // Arm.move_absolute(0,200);  // Stop the intake motor when B is pressed
  // DriveToPoint_OdomPID(50, 0, -130, 80, 2000, 30);

  // TongueMech.set_value(1);
  // DriveToPoint_OdomPID(50, 0, -180, 80, 2000, 30);

  // DriveToPoint_OdomPID(45, -20, -180, 80, 2000, 30);

  // DriveToPoint_OdomPID(45, 10,  -180, 80, 2000, 30);
  
  
}

void soloAWP(){
  // This routine is rewritten to use the same control style as 20164X:
  // field-centric vector drive + heading hold, with slew-limited motor outputs.
  // Coordinates assume: X right +, Y forward +, heading 0 = +Y.

  // 1) Go to preload area
  driveToPoint(drive, 0, 36, 0, 0.85, 0.55, 2500);

  // 2) Turn and intake preload
  turnToHeading(drive, 90, 0.65, 1500);
  FrontIntake.move(127);
  driveToPoint(drive, 6, 36, 90, 0.55, 0.55, 1200);
  pros::delay(400);

  // 3) Back out and score (placeholder path — tune points to your field start)
  FrontIntake.move(0);
  driveToPoint(drive, 0, 0, 90, 0.85, 0.55, 3000);
}

void RightSideAuton(){

  // NOTE: This is a 20164X-style *template* for your right-side auto.
  // You MUST tune these target points to your actual start tile and game plan.
  // The important part is that every move is now:
  //   driveToPoint20164X(...) / turnToHeading20164X(...)

  FrontIntake.move(127);

  // Example path: approach 3 blocks
  driveToPoint(drive, 0, 7, 0, 0.75, 0.55, 1200);
  turnToHeading(drive, 12, 0.60, 1000);
  driveToPoint(drive, 3, 17, 12, 0.60, 0.55, 1600);

  TongueMech.set_value(1);
  pros::delay(150);
  TongueMech.set_value(0);

  // Back out, turn to goal, score
  driveToPoint(drive, 0, 9, 12, 0.70, 0.55, 1200);
  turnToHeading(drive, -55, 0.70, 1200);
  driveToPoint(drive, -6, 20, -55, 0.70, 0.60, 2000);
  FrontIntake.move(-90);
  pros::delay(1800);
  FrontIntake.move(0);
}

void LeftSideAuton(){
  // 20164X-style *template* for your left-side auto.
  // Replace the target points with your real field coordinates.

  FrontIntake.move(127);

  // Example: approach blocks on left
  driveToPoint(drive, 0, 7, 0, 0.75, 0.55, 1200);
  turnToHeading(drive, -15, 0.60, 1000);
  driveToPoint(drive, -6, 22, -15, 0.60, 0.55, 2000);

  // Example scoring sequence
  IntakeLift.set_value(1);
  FrontIntake.move(90);
  Arm.move_absolute(-700, 180);
  pros::delay(1000);
  FrontIntake.move(0);
  Arm.move_absolute(0, 130);
  IntakeLift.set_value(0);
}

void OffParkAuton(){
  // Simple "get off the park" example.
  driveToPoint(drive, -5, 5, 0, 0.85, 0.55, 1500);
}

void Skills(){
  FrontIntake.move(127); // Spins intake to grab preload
  driveToPoint(drive, 3, 21, 0, 0.35, 0.55, 2000);
  driveToPoint(drive, 0, 3, 0, 0.85, 0.55, 1500);
  driveToPoint(drive, 44, 0, 0, 0.85, 0.55, 2500);
  driveToPoint(drive, 0, 0, 180, 0.85, 0.65, 3000);
  driveToPoint(drive, 0, 17, 180, 0.85, 0.65, 1500);
  // pros::delay(500);
  // DriveToPoint_OdomPID(-1, -20, 0, 100, 1500, 40);
  // pros::delay(500);
  // FrontIntake.move(127);
  // pros::delay(3500);
  // DriveToPoint_OdomPID(-1, 22, 180, 100, 1500, 40);
  // pros::delay(500);
}