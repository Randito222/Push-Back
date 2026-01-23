#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "XDrive_PID.hpp"
#include "main.h"
#include "pros/device.hpp"
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

// . . .
// Make your own autonomous functions here!
// . . .


void AutonTesting(){

  FrontIntake.move(127); // Spins intake to grab preload
  DriveToPoint_OdomPID(-7,8,0, 100, 1200, 40); 

  DriveToPoint_OdomPID(-7, 37,  0,30, 4000, 40);

  DriveToPoint_OdomPID(-7, 37,  -90,100, 1000, 40);

  DriveToPoint_OdomPID(-16.5, 37,  -90,100, 1000, 40);

  DriveToPoint_OdomPID(-16.5, 37,  -137,100, 1000, 40);

  IntakeLift.set_value(1);
  Arm.move_absolute(-920,55);  // Stop the intake motor when B is pressed
  pros::delay(950); // Waits to make sure preload is out
  IntakeLift.set_value(0);
  Arm.move_absolute(0,200);  // Stop the intake motor when B is pressed

  DriveToPoint_OdomPID(-16.2, 37,  -180,100, 1000, 40);
  

  DriveToPoint_OdomPID(-46, 0, -180,100, 2000, 40);

  



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
  chassis.pid_drive_set(36, DRIVE_SPEED, true); // Goes towards the preload area
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED); // Turns to face preload
  chassis.pid_wait();
  
  IntakeSpin(); // Spins intake to grab preload

  chassis.pid_drive_set(6, DRIVE_SPEED); // Moves forward to grab preload
  chassis.pid_wait();

  pros::delay(500); // Waits to make sure preload is in intake
  IntakeSpin(); // Stops intake

  chassis.pid_drive_set(-42, DRIVE_SPEED, true); // Backs away from preload area into the side tube
  chassis.pid_wait();

  IntakeSpin(); // Spins intake to outtake preload into the side tube
  pros::delay(500); // Waits to make sure preload is out

  chassis.pid_drive_set(6, DRIVE_SPEED); // Moves forward to clear the side tube
  chassis.pid_wait_quick_chain(); // Quick chain to next movement for faster movement
 
  chassis.pid_turn_set(200, TURN_SPEED); // Turns to face middle balls and middle tube
  chassis.pid_wait();

  chassis.pid_drive_set(35, DRIVE_SPEED, true); // Drives to middle balls and middle tube 
  chassis.pid_wait_until(25); // Waits until 25 inches away to start intaking
  chassis.pid_speed_max_set(50); // Slows down max speed to 50 for better control
  chassis.pid_wait();

  chassis.pid_turn_set(30, TURN_SPEED); // Turns for back faces middle goal
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true); // Backs up to get touch middle goal
  chassis.pid_wait();

  chassis.pid_turn_set(160, TURN_SPEED); // Turns to face the other middle balls
  chassis.pid_wait();

  chassis.pid_drive_set(50, DRIVE_SPEED, true); // Drives to the other middle balls and goes to other side tube
  chassis.pid_wait();

  chassis.pid_turn_set(90, TURN_SPEED); // Turns for back to face the side tube
  chassis.pid_wait();

  chassis.pid_drive_set(-12, DRIVE_SPEED); // Backs up to get touch the side tube and score
  chassis.pid_wait();

  pros::delay(1000); // Waits to make sure preload is out
  IntakeSpin(); // Stops intake
}

void RightSideAuton(){
  // FrontIntake.move(127); // Spins intake to grab 3 blocks

  // chassis.pid_drive_set(7, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();

  // chassis.pid_turn_set(13,TURN_SPEED);
  // chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(23, 50, true); // Goes towards the 3 block area
  // chassis.pid_wait();

  // pros::delay(1000); // Waits to make sure blocks are in intake

  // chassis.pid_drive_set(-8, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();

  // chassis.pid_turn_set(-44, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_drive_set(15, DRIVE_SPEED, true); // Backs away from 3 block area into the side tube
  // chassis.pid_wait_quick_chain();

  // FrontIntake.move(-90); // Outtakes blocks into the goal
  // pros::delay(2100); // Waits to make sure blocks are out
  // FrontIntake.move(0); // Stops intake;

  

  // chassis.pid_turn_set(-55, TURN_SPEED); // Turns to face the other side tube
  // chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(-31, DRIVE_SPEED, true); // Moves forward to clear the side tube
  // chassis.pid_wait_quick_chain();

  // chassis.pid_turn_set(-178, TURN_SPEED); // Moves forward to clear the side tube
  // chassis.pid_wait();

  // TongueMech.set_value(1); // Outtakes blocks into the goal
  // FrontIntake.move(127);

  // chassis.pid_drive_set(14, DRIVE_SPEED, true); // Moves forward to clear the side tube
  // chassis.pid_wait();
  // pros::delay(1000);

  // chassis.pid_drive_set(-20, DRIVE_SPEED, true); // Moves forward to clear the side tube
  // chassis.pid_wait();

  // Arm.move_absolute(-530,130);  // 
  // pros::delay(800);
  // Arm.move_absolute(0,130);  //

}

void LeftSideAuton(){

  FrontIntake.move(127); // Spins intake to grab 3 blocks

  chassis.pid_drive_set(7, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-15,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23, 50, true); // Goes towards the 3 block area
  chassis.pid_wait();

  pros::delay(1000); // Waits to make sure blocks are in intake

  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(226, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-13, DRIVE_SPEED, true); // Backs away from 3 block area into the side tube
  chassis.pid_wait();

  IntakeLift.set_value(1);
  FrontIntake.move(90); // Outtakes blocks into the goal
  Arm.move_absolute(-700, 180);
  pros::delay(1200); // Waits to make sure blocks are out
  FrontIntake.move(0); // Stops intake
  Arm.move_absolute(0, 130);

  chassis.pid_turn_set(55, TURN_SPEED); // Turns to face the other side tube
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(31, DRIVE_SPEED, true); // Moves forward to clear the side tube
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(178, TURN_SPEED); // Moves forward to clear the side tube
  chassis.pid_wait();

  TongueMech.set_value(1); // Outtakes blocks into the goal
  FrontIntake.move(127);

  chassis.pid_drive_set(6, DRIVE_SPEED, true); // Moves forward to clear the side tube
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_drive_set(-20, DRIVE_SPEED, true); // Moves forward to clear the side tube
  chassis.pid_wait();

  Arm.move_absolute(-530,130);  // 
  pros::delay(800);
  Arm.move_absolute(0,130);  //

}

void BruinRightAuto(){
  chassis.pid_drive_set(14, DRIVE_SPEED ); // Goes towards the preload area
  chassis.pid_wait();
  FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
  MiddleIntake.move(127);

  chassis.pid_turn_set(20, TURN_SPEED); // Turns to face preload
  chassis.pid_wait();

  chassis.pid_drive_set(14, 20); // Moves forward to grab preload
  chassis.pid_wait();

  chassis.pid_turn_set(-47, TURN_SPEED); // Turns to face the goal
  chassis.pid_wait();

  chassis.pid_drive_set(11, DRIVE_SPEED); // Backs away from preload area into the side tube
  chassis.pid_wait();

  FrontIntake.move(-127);  // Stop the intake motor when R2 is pressed
  MiddleIntake.move(-127);
  TopIntake.move(-127);
  pros::delay(1500); // Waits to make sure preload is out

  chassis.pid_drive_set(-40, DRIVE_SPEED); // Moves forward to clear the side tube
  chassis.pid_wait();

  MatchLoading();

  chassis.pid_turn_set(2, TURN_SPEED); // Turns to face middle balls and middle tube
  chassis.pid_wait();

  chassis.pid_drive_set(6, DRIVE_SPEED, true); // Drives to middle balls and middle tube 
  chassis.pid_wait_quick_chain();

  FrontIntake.move(0);  // Spin the intake motor when R1 is pressed
  MiddleIntake.move(127);
  chassis.pid_drive_set(-8, DRIVE_SPEED); // Drives to middle balls and middle tube 
  chassis.pid_wait();

  chassis.pid_drive_set(8, DRIVE_SPEED, true); // Drives to middle balls and middle tube
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8, DRIVE_SPEED); // Drives to middle balls and middle tube 
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(8, DRIVE_SPEED, true); // Drives to middle balls and middle tube
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8, DRIVE_SPEED); // Drives to middle balls and middle tube 
  chassis.pid_wait_quick_chain();
}

void BruinLeftAuto(){
  // Add auton code here
}

void Skills(){
  FrontIntake.move(127); // Spins intake to grab preload
  DriveToPoint_OdomPID(3,21,0, 40, 2000, 40);
  DriveToPoint_OdomPID(0,3,0, 100, 1500, 40);
  DriveToPoint_OdomPID(44, 0, 0, 100, 2000, 40);
  DriveToPoint_OdomPID(0, 0, 180, 100, 3000, 40);
  DriveToPoint_OdomPID(0, 17, 180, 100, 1500, 40);
  // pros::delay(500);
  // DriveToPoint_OdomPID(-1, -20, 0, 100, 1500, 40);
  // pros::delay(500);
  // FrontIntake.move(127);
  // pros::delay(3500);
  // DriveToPoint_OdomPID(-1, 22, 180, 100, 1500, 40);
  // pros::delay(500);
}