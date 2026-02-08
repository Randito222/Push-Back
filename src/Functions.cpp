#include <math.h>
#include "main.h"
#include "pros/misc.h"
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

static double fcZeroRad = 0.0;
static bool lastFC = false;

static double wrapPi(double a){
  while (a> M_PI) a-=2.0 *M_PI;
  while (a < M_PI) a+=2.0 *M_PI;
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

  // Deadzones
  if (std::fabs(forward) < 5) forward = 0;
  if (std::fabs(strafe)  < 5) strafe  = 0;
  if (std::fabs(rotate)  < 5) rotate  = 0;

  if (fieldCentric && !lastFC) {
    fcZeroRad = IMU.get_rotation() * M_PI / 180.0;
  }
  lastFC = fieldCentric;

  if(master.get_digital_new_press(DIGITAL_A)){
    fcZeroRad = IMU.get_rotation() * M_PI / 180.0;
  }

  // Field-centric transform (field -> robot)
  if (fieldCentric) {
    double headingRad = IMU.get_rotation() * M_PI / 180.0;
    headingRad = wrapPi(headingRad-fcZeroRad);
    const double c = std::cos(headingRad);
    const double s = std::sin(headingRad);

    const double tempForward =  forward * c + strafe * s;
    const double tempStrafe  = -forward * s + strafe * c;

    forward = tempForward;
    strafe  = tempStrafe;

    if(std::fabs(forward) < 3) forward =0;
    if(std::fabs(strafe) < 3) strafe =0;
  }

  // Mix
  double fl = forward + strafe + rotate;
  double fr = forward - strafe - rotate;
  double bl = forward - strafe + rotate;
  double br = forward + strafe - rotate;

  // Normalize (IMPORTANT: do this for BOTH modes)
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

  // Slew (diff-based = stable)
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

  pros::delay(30);  // Small delay for stability
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
const uint32_t HOLD_TIME_MS = 1000;

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
            Descore.set_value(1);
            DescoreLift.set_value(1);
            bothExtended = true;
            descoreState = true;
        }
        // After first press, toggle ONLY descore
        else {
            descoreState = !descoreState;
            Descore.set_value(descoreState);
        }
    }

    // Holding Y (check for reset)
    if (holdingY && yPressed) {
        if (pros::millis() - yPressStart >= HOLD_TIME_MS) {
            // Retract both
            Descore.set_value(0);
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
  if(master.get_digital(DIGITAL_B) == true && IntakeLiftT == -1) {
    FrontIntake.move(127);  // Spin the intake motor when R1 is pressed
    Arm.move_absolute(-590,200);  // Stop the intake motor when B is pressed
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