#include "main.h"
#include "subsystems.hpp"
/*

Drive Controls

**/

void setDrivePower(int fl, int fr, int bl, int br) {
  // Set all motors for each wheel group
  Front_Left_1.move(fl);
  Front_Left_2.move(fl);

  Front_Right_1.move(fr);
  Front_Right_2.move(fr);

  Back_Left_1.move(bl);
  Back_Left_2.move(bl);

  Back_Right_1.move(br);
  Back_Right_2.move(br);
} 

// void IntakeSpin() {
//   // Spin the intake motor
//   Intake.move_velocity(200);  // Set the intake motor to spin at 200 RPM
// }

// Start by storing the robot's current heading as the initial target
double targetAngle = IMU.get_heading();
bool lastButtonState = false;  // Tracks the last state of the button to detect presses

int slewDrive(int target, int current, int rate) {
    if (current < target)
        current += rate;
    else if (current > target)
        current -= rate;

    // Snap when close
    if (abs(target - current) < rate)
        current = target;

    return current;
}

void DriveControl() {

  int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // Forward/Backward
  int strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);   // Left/Right
  int rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Rotation

  // Holonomic drive calculation (X-drive)
  int flTarget = forward + strafe + rotate;
  int frTarget = forward - strafe - rotate;
  int blTarget = forward - strafe + rotate;
  int brTarget = forward + strafe - rotate;

  // Apply slew rate to each wheel
  int flPower = slewDrive(flTarget, flPower, 100);
  int frPower = slewDrive(frTarget, frPower, 100);
  int blPower = slewDrive(blTarget, blPower,  100);
  int brPower = slewDrive(brTarget, brPower, 100);

// Send smoothed values to the motors
  setDrivePower(flPower, frPower, blPower, brPower);

  // Delay to avoid overloading the CPU
  pros::delay(10);
}

// void IntakeReverse(){
//   Intake.move_velocity(-200);
// }

int DescoreLV = -1;
void descoreLeftT(){
  DescoreLV*=-1;

  if (DescoreLV==1){
    DescoreLeft.set_value(1);
  }

  else{
    DescoreLeft.set_value(0);
}
}
// int DescoreRV = -1;
// void descoreRight(){
//   DescoreRV*=-1;

//   if (DescoreRV==1){
//     DescoreRight.set_value(1);
//   }

//   else{
//     DescoreRight.set_value(0);
// }
// }

// int ScoreP = -1;
// void ScoringP(){
//   ScoreP*=-1;

//   if (ScoreP==1){
//     ScorePiston.set_value(1);
//   }

//   else{
//     ScorePiston.set_value(0);
// }
// }

int IntakeScoreV = -1;
void IntakeScoreToggle(){
  IntakeScoreV*=-1;

  if (IntakeScoreV==1){
    IntakeLift.set_value(1);
  }

  else{
    IntakeLift.set_value(0);
  }
}

int MatchLoadV = -1;
void MatchLoading(){
  MatchLoadV*=-1;

  if (MatchLoadV==1){
    TongueMech.set_value(1);
  }

  else{
    TongueMech.set_value(0);
  }
}