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
  };

  slewApply(tFL, fl);
  slewApply(tFR, fr);
  slewApply(tBL, bl);
  slewApply(tBR, br);

  setDrivePower(fl, fr, bl, br);
}

// =============================
// Driver control (ROBOT CENTRIC)
// =============================
void DriveControlBackUp() {
  static int fl = 0, fr = 0, bl = 0, br = 0;
  constexpr int slew = 50;

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
