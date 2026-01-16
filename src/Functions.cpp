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
  static int flPower = 0, frPower = 0, blPower = 0, brPower = 0;
  const int slewRate = 10;

  double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  double strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  double rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  if (fabs(forward) < 5) forward = 0;
  if (fabs(strafe)  < 5) strafe  = 0;
  if (fabs(rotate)  < 5) rotate  = 0;

  double headingRad = IMU.get_rotation() * M_PI / 180.0;

  // Field -> robot (rotate by -heading)
  double f =  forward * cos(headingRad) + strafe * sin(headingRad);
  double s = -forward * sin(headingRad) + strafe * cos(headingRad);
  double r = rotate;

  // Mix
  double fl = f + s + r;
  double fr = f - s - r;
  double bl = f - s + r;
  double br = f + s - r;

  // Normalize
  double maxMag = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
  if (maxMag > 127.0) {
    double scale = 127.0 / maxMag;
    fl *= scale; fr *= scale; bl *= scale; br *= scale;
  }

  int flTarget = (int)fl;
  int frTarget = (int)fr;
  int blTarget = (int)bl;
  int brTarget = (int)br;

  auto applySlew = [&](int target, int &current) {
    int diff = target - current;
    if (abs(diff) <= slewRate) current = target;
    else current += (diff > 0 ? slewRate : -slewRate);
  };

  applySlew(flTarget, flPower);
  applySlew(frTarget, frPower);
  applySlew(blTarget, blPower);
  applySlew(brTarget, brPower);

  setDrivePower(flPower, frPower, blPower, brPower);
}

void DriveControlBackUp() {
  static int flPower = 0, frPower = 0, blPower = 0, brPower = 0;
  const int slewRate = 10;

  double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  double strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  double rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  if (fabs(forward) < 5) forward = 0;
  if (fabs(strafe)  < 5) strafe  = 0;
  if (fabs(rotate)  < 5) rotate  = 0;

  double fl = forward + strafe + rotate;
  double fr = forward - strafe - rotate;
  double bl = forward - strafe + rotate;
  double br = forward + strafe - rotate;

  // Normalize
  double maxMag = std::max({fabs(fl), fabs(fr), fabs(bl), fabs(br)});
  if (maxMag > 127.0) {
    double scale = 127.0 / maxMag;
    fl *= scale; fr *= scale; bl *= scale; br *= scale;
  }

  int flTarget = (int)fl;
  int frTarget = (int)fr;
  int blTarget = (int)bl;
  int brTarget = (int)br;

  auto applySlew = [&](int target, int &current) {
    int diff = target - current;
    if (abs(diff) <= slewRate) current = target;
    else current += (diff > 0 ? slewRate : -slewRate);
  };

  applySlew(flTarget, flPower);
  applySlew(frTarget, frPower);
  applySlew(blTarget, blPower);
  applySlew(brTarget, brPower);

  setDrivePower(flPower, frPower, blPower, brPower);
}



// void IntakeReverse(){
//   Intake.move_velocity(-200);
// }

void IntakeLiftToggle(){
  if(master.get_digital_new_press(DIGITAL_DOWN)){
      IntakeLiftT *= -1;
      if(IntakeLiftT == 1){
        IntakeLift.set_value(true);
      }
      else{
        IntakeLift.set_value(false);
      }
    }
}

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
  if(master.get_digital_new_press(DIGITAL_RIGHT)){
    MatchLoadV*=-1;

    if (MatchLoadV==1){
      TongueMech.set_value(1);
    }

    else{
      TongueMech.set_value(0);
    }
  }
}

void ArmAction(){
  if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == -1) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-570,150);  // Stop the intake motor when B is pressed
    KnownState=1;
    pros::delay(200);
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
  }
  else if (master.get_digital(DIGITAL_L1) == true && (IntakeLiftT == -1 || IntakeLiftT == 0)){
    FrontIntake.move(-127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-570,150);  // Stop the intake motor when B is released
    pros::delay(500);
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
    Arm.move_absolute(5,200);  // Stop the intake motor when B is released
  
  }
  else if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == 1){
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-700,120);  // Stop the intake motor when B is pressed
    KnownState=1;
    pros::delay(200);
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
  }
  else if (master.get_digital(DIGITAL_B) == false && KnownState == 1){ 
    Arm.move_absolute(5,200);  // Stop the intake motor when B is released
    KnownState=0;
    pros::delay(200);

  }
}