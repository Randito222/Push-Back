#include "Functions.hpp"
#include "main.h"
#include <cmath>

// =============================
// Globals (from header)
// =============================
int IntakeLiftT = -1;
int KnownState  = 0;

// =============================
// Drive helpers
// =============================
void setDrivePower(int fl, int fr, int bl, int br) {
  Front_Left_1.move(fl);
  Front_Left_2.move(fl);
  Front_Right_1.move(fr);
  Front_Right_2.move(fr);
  Back_Left_1.move(bl);
  Back_Left_2.move(bl);
  Back_Right_1.move(br);
  Back_Right_2.move(br);
}

// =============================
// Driver control (FIELD CENTRIC)
// =============================
void DriveControl() {
<<<<<<< HEAD
  static int fl = 0, fr = 0, bl = 0, br = 0;
  constexpr int slew = 50;

  double f = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  double s = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  double r = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  if (std::fabs(f) < 5) f = 0;
  if (std::fabs(s) < 5) s = 0;
  if (std::fabs(r) < 5) r = 0;

  double h = IMU.get_rotation() * M_PI / 180.0;
  double tf =  f * cos(h) + s * sin(h);
  double ts = -f * sin(h) + s * cos(h);

  int tFL = tf + ts + r;
  int tFR = tf - ts - r;
  int tBL = tf - ts + r;
  int tBR = tf + ts - r;

  auto slewApply = [&](int tgt, int &cur) {
    if (cur < tgt) cur += slew;
    else if (cur > tgt) cur -= slew;
    if (std::abs(tgt - cur) < slew) cur = tgt;
=======
  static int flPower = 0, frPower = 0, blPower = 0, brPower = 0;
  const int slewRate = 50;

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
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
  };

  slewApply(tFL, fl);
  slewApply(tFR, fr);
  slewApply(tBL, bl);
  slewApply(tBR, br);

<<<<<<< HEAD
  setDrivePower(fl, fr, bl, br);
}

// =============================
// Driver control (ROBOT CENTRIC)
// =============================
void DriveControlBackUp() {
  static int fl = 0, fr = 0, bl = 0, br = 0;
  constexpr int slew = 50;
=======
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


>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9

  int f = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int s = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  int r = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  int tFL = f + s + r;
  int tFR = f - s - r;
  int tBL = f - s + r;
  int tBR = f + s - r;

  auto slewApply = [&](int tgt, int &cur) {
    if (cur < tgt) cur += slew;
    else if (cur > tgt) cur -= slew;
    if (std::abs(tgt - cur) < slew) cur = tgt;
  };

  slewApply(tFL, fl);
  slewApply(tFR, fr);
  slewApply(tBL, bl);
  slewApply(tBR, br);

  setDrivePower(fl, fr, bl, br);
}

// =============================
// Intake / mechanisms
// =============================
void IntakeSpin() {
  FrontIntake.move(127);
}

void IntakeReverse() {
  FrontIntake.move(-127);
}

void IntakeLiftToggle() {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    IntakeLiftT *= -1;
    IntakeLift.set_value(IntakeLiftT == 1);
  }
}

void descoreLeftT() {
  static int state = -1;
  state *= -1;
  DescoreLeft.set_value(state == 1);
}

void IntakeScoreToggle() {
  static int state = -1;
  state *= -1;
  IntakeLift.set_value(state == 1);
}

void MatchLoading() {
  static int state = -1;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    state *= -1;
    TongueMech.set_value(state == 1);
  }
}

<<<<<<< HEAD
// =============================
// Arm
// =============================
void ArmAction() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    Arm.move_absolute(-600, 180);
    KnownState = 1;
  } else if (KnownState == 1) {
    Arm.move_absolute(0, 180);
    KnownState = 0;
  }
}
=======
void ArmAction(){
  if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == -1) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-570,200);  // Stop the intake motor when B is pressed
    KnownState=1;
    pros::delay(200);
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
  }
  else if (master.get_digital(DIGITAL_L1) == true && (IntakeLiftT == -1 || IntakeLiftT == 0)){
    FrontIntake.move(-127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-570,200);  // Stop the intake motor when B is released
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
>>>>>>> 03c2fb1e071a3f655c89c1b43e686c9ef89060f9
