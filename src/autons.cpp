#include "autons.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "XDrive_PID.hpp"
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

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void Tuning_PID(){
  // IMU.reset();
  // resetOdom();
  //odom_task.resume();
  DriveToPoint_PID(0, 20, 0, 100, 100);
  //DriveToPoint_PID(10, 00, 0, 100, 200);
  //DriveToPoint_PID(20, 10, 90, 100, 100);
  // PID_Movement(80, 100);
  // PID_Turn(90, 90);
  // PID_Strafe(-10, 90);

//   PID_Strafe(-80, 50);
//   //PID_Strafe(20, 50);
//   PID_Turn(173, 90);
//   pros::delay(500);
//   PID_Movement(-37, 60);

//   FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
//   Arm.move_absolute(-530,130);  // 
//   pros::delay(800);
//   Arm.move_absolute(0,130);  // 

// FrontIntake.move(127);  // Stop the intake motor when R2 is pressed

// //PID_Movement(80, 70);
// chassis.pid_drive_set(30, 70);
// chassis.pid_wait();

// chassis.pid_turn_set(-44,60);
// pros::delay(500);

// chassis.pid_drive_set(15, 70);
// pros::delay(500);

// FrontIntake.move(-90);
// pros::delay(500);

// FrontIntake.move(-90);
// pros::delay(500);

// FrontIntake.move(-90);
// pros::delay(3000);

// IntakeLift.set_value(1);
// pros::delay(3000);
// IntakeLift.set_value(0);
// FrontIntake.move(0);
}

void AutonTesting(){

  chassis.pid_drive_set(144, DRIVE_SPEED, true);
  chassis.pid_wait();
  // // === MOVE 1: Forward 36 inches ===
  // pros::Task move1([]() {
  //   x_drive_pid_task(36.0, 0.0, 0.0); // Move to (36, 0) facing 0 degrees
  // });

  // pid_wait_until_distance(20.0); // Wait until within 20 inches
  // Intake.move(120); // Start intake early
  // pros::delay(1000);
  // Intake.move_voltage(0); // Stop intake

  // move1.join(); // Wait until move 1 is fully done


  // // === MOVE 2: Strafe right 24 inches ===
  // pros::Task move2([]() {
  //   x_drive_pid_task(36.0, 24.0, 0.0); // Move to (36, 24)
  // });

  // pid_wait_until_distance(10.0); // Wait until close
  // Intake.move(120); // Spin up flywheel

  // move2.join();


  // // === MOVE 3: Turn in place (rotate to 90 degrees) ===
  // pros::Task rotate([]() {
  //   x_drive_pid_task(36.0, 24.0, 90.0); // Stay in place, rotate to 90 degrees
  // });

  // rotate.join();
  // Intake.move_voltage(0); // Turn off flywheel


  // // === MOVE 4: Move backward to start ===
  // pros::Task move4([]() {
  //   x_drive_pid_task(0.0, 0.0, 90.0); // Return to (0, 0) still facing 90 degrees
  // });

  // pid_wait_until_distance(15.0);
  // move4.join();
  
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

}

void LeftSideAuton(){
 chassis.pid_drive_set(36, DRIVE_SPEED, true); // Goes towards the preload area
  chassis.pid_wait();

  chassis.pid_turn_set(-90, TURN_SPEED); // Turns to face preload
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
 
  chassis.pid_turn_set(-200, TURN_SPEED); // Turns to face middle balls and middle tube
  chassis.pid_wait();

  chassis.pid_drive_set(35, DRIVE_SPEED, true); // Drives to middle balls and middle tube 
  chassis.pid_wait_until(25); // Waits until 25 inches away to start intaking
  chassis.pid_speed_max_set(50); // Slows down max speed to 50 for better control
  chassis.pid_wait();

  chassis.pid_turn_set(-30, TURN_SPEED); // Turns for back faces middle goal
  chassis.pid_wait();

  chassis.pid_drive_set(-8, DRIVE_SPEED, true); // Backs up to get touch middle goal
  chassis.pid_wait();

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
  // Add auton code here

  // FrontIntake.move(-127);  // Spin the intake motor when R1 is pressed
  // chassis.pid_drive_set(30, DRIVE_SPEED ); // Goes towards the preload area
  // chassis.pid_wait();

  // chassis.pid_drive_set(-18, DRIVE_SPEED ); // Goes towards the preload area
  // chassis.pid_wait();

  // PID_Strafe(-80, 100);
  // pros::delay(500);
  // PID_Strafe(40,  100);
}