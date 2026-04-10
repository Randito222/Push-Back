#include "Functions.hpp"
#include <math.h>
#include "main.h"
#include "pros/misc.h"
#include "subsystems.hpp"

void IntakeSpin() {
  if(master.get_digital(DIGITAL_R2)) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
  }
  else if(master.get_digital(DIGITAL_R1)) {
    FrontIntake.move(-57);  // Spin the intake motor in reverse when R2 is pressed
  } 
  else if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == -1) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-590,100);
    HoodLift.set_value(1);  // Stop the intake motor when B is pressed
    // KnownState=1;
  }
  else if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == 1){
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-700,30);
    HoodLift.set_value(1);  // Stop the intake motor when B is pressed
    KnownState=1;
  }
  else if (master.get_digital(DIGITAL_B) == false && KnownState == 1){ 
    Arm.move_absolute(5,100);
    HoodLift.set_value(0);  // Stop the intake motor when B is released
    KnownState=0;
    pros::delay(200);

  }
  else{
    FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
    Arm.move_absolute(5,100); 
    HoodLift.set_value(0); // Stop the intake motor when B is released
  }
}
/*

Drive Controls

**/

void setDrivePower(int fl, int fr, int bl, int br) {
  // Set all motors for each wheel group
  Front_Right_1.move(fr);
  Front_Right_2.move(fr);

  Back_Right_1.move(br);
  Back_Right_2.move(br);

  Front_Left_1.move(fl);
  Front_Left_2.move(fl);

  Back_Left_1.move(bl);
  Back_Left_2.move(bl);

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

static double wrapPi(double a){
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;   // FIX: must be -M_PI here
  return a;
}

void DriveControlUnified(bool fieldCentric) {
  // Persistent output (slew)
  static int flPower = 0, frPower = 0, blPower = 0, brPower = 0;
  const int slewRate = 50;

  // Controller input
  double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  double strafe  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  double rotate  = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  // Deadzones (raw stick noise)
  if (std::fabs(forward) < 5) forward = 0;
  if (std::fabs(strafe)  < 5) strafe  = 0;
  if (std::fabs(rotate)  < 5) rotate  = 0;

  // Capture initial zero when field-centric turns on
  if (fieldCentric && !lastFC) {
    fcZeroRad = IMU.get_heading() * M_PI / 180.0;   // prefer heading for driver FC
  }
  lastFC = fieldCentric;

  // Manual re-zero (still useful, but shouldn't be required often)
  if (master.get_digital_new_press(DIGITAL_A)) {
    fcZeroRad = IMU.get_heading() * M_PI / 180.0;
  }

  // Field-centric transform (field -> robot)
  if (fieldCentric) {
    // Use bounded heading for stability in driver control
    double headingRad = IMU.get_heading() * M_PI / 180.0;
    headingRad = wrapPi(headingRad - fcZeroRad);

    const double c = std::cos(headingRad);
    const double s = std::sin(headingRad);

    const double tempForward =  forward * c + strafe * s;
    const double tempStrafe  = -forward * s + strafe * c;

    forward = tempForward;
    strafe  = tempStrafe;

    // Post-transform deadband (kills drift-induced tiny strafe)
    if (std::fabs(forward) < 8) forward = 0;
    if (std::fabs(strafe)  < 8) strafe  = 0;

    // --- AUTO-DRIFT COMP (no manual reset needed most of the time) ---
    // Only learn when driver is NOT rotating and is mostly pushing forward.
    const double rotDeadband = 6;        // rotate stick deadband
    const double learnRate   = 0.0008;   // rad per loop (tune 0.0004..0.0012)
    const double minCmd      = 25;       // must be actually driving
    const double fwdBias     = 2.0;      // "mostly forward" gate

    bool driverRotating = (std::fabs(rotate) > rotDeadband);
    bool driverDriving  = (std::fabs(forward) + std::fabs(strafe) > minCmd);
    bool mostlyForward  = (std::fabs(forward) > std::fabs(strafe) * fwdBias);

    if (!driverRotating && driverDriving && mostlyForward) {
      // After transform, strafe should be ~0 for a straight push.
      // Nudge zero to cancel persistent strafe caused by IMU bias drift.
      fcZeroRad += (-strafe / 127.0) * learnRate;
      fcZeroRad = wrapPi(fcZeroRad);
    }
  }

  // Mix
  double fl = forward + strafe + rotate;
  double fr = forward - strafe - rotate;
  double bl = forward - strafe + rotate;
  double br = forward + strafe - rotate;

  // Normalize
  double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br)});
  if (maxMag > 127.0) {
    const double scale = 127.0 / maxMag;
    fl *= scale; fr *= scale; bl *= scale; br *= scale;
  }

  // Targets
  int flTarget = (int)fl;
  int frTarget = (int)fr;
  int blTarget = (int)bl;
  int brTarget = (int)br;

  // Slew (diff-based)
  auto applySlew = [&](int target, int &current) {
    int diff = target - current;
    if (std::abs(diff) <= slewRate) current = target;
    else current += (diff > 0 ? slewRate : -slewRate);
  };

  applySlew(flTarget, flPower);
  applySlew(frTarget, frPower);
  applySlew(blTarget, blPower);
  applySlew(brTarget, brPower);

  setDrivePower(flPower, frPower, blPower, brPower);

  pros::delay(30);
}




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

bool bothExtended = false;     // have both pistons been deployed?
bool descoreState = false;    // current descore toggle state
bool holdingY = false;

uint32_t yPressStart = 0;
const uint32_t HOLD_TIME_MS = 500;

void descoring() {
    bool yPressed = master.get_digital(DIGITAL_Y);

    // =============================
    // Button just pressed
    // =============================
    if (master.get_digital_new_press(DIGITAL_Y)) {
        yPressStart = pros::millis();
        holdingY = true;

        // If both pistons are not yet extended, extend both
        if (!bothExtended) {
            Descore.set_value(0);
            DescoreLift.set_value(0);
            bothExtended = true;
            descoreState = false;
        }
        // After first press, toggle ONLY descore
        else {
            descoreState = !descoreState;
            DescoreLift.set_value(descoreState);
        }
    }

    // Holding Y (check for reset)
    if (holdingY && yPressed) {
        if (pros::millis() - yPressStart >= HOLD_TIME_MS) {
            // Retract both
            Descore.set_value(1);
            DescoreLift.set_value(0);

            bothExtended = false;
            descoreState = false;
            holdingY = false;
        }
    }

    // Button released
    if (!yPressed) {
        holdingY = false;
    }
}


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
  if(master.get_digital(DIGITAL_B) == true) {
    FrontIntake.move(127);
    HoodLift.set_value(1);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-590,200);  // Stop the intake motor when B is pressed
    // KnownState=1;
  }
  else {
    HoodLift.set_value(0);
    FrontIntake.move(0);
    Arm.move_absolute(0,200);
  }

  // else if (master.get_digital(DIGITAL_L1) == true && (IntakeLiftT == -1 || IntakeLiftT == 0)){
  //   FrontIntake.move(-127);  // Spin the intake motor when R1 is pressed
  //   Arm.move_absolute(-570,140);  // Stop the intake motor when B is released
  //   pros::delay(500);
  //   FrontIntake.move(0);  // Stop the intake motor when R2 is pressed
  //   Arm.move_absolute(5,200);  // Stop the intake motor when B is released
  
  // }
  // else if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == 1){
  //   FrontIntake.move(127); 
  //   HoodLift.set_value(1); // Spin the intake motor when R1 is pressed
  //   Arm.move_absolute(-700,120);  // Stop the intake motor when B is pressed
  //   KnownState=1;
  // }
  // else if (master.get_digital(DIGITAL_B) == false && KnownState == 1){
  //   HoodLift.set_value(0);
  //   Arm.move_absolute(5,200);  // Stop the intake motor when B is released
  //   KnownState=0;
  //   pros::delay(200);
  // }
  // else {
  //   HoodLift.set_value(0);
  //   Arm.move_absolute(5,200);  // Stop the intake motor when B is released
  // }
}