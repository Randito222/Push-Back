#include "Drive.hpp"

#include "Functions.hpp"   // setDrivePower(...)
#include "subsystems.hpp"  // IMU (port 6)

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// Global drive instance
// ------------------------------------------------------------
FieldXDrive drive(&IMU);

// ------------------------------------------------------------
// Internal helpers
// ------------------------------------------------------------
namespace {
int slewStep(int target, int current, int maxStep) {
  int diff = target - current;
  if (std::abs(diff) <= maxStep) return target;
  return current + (diff > 0 ? maxStep : -maxStep);
}

// Single background task for heading updates.
struct HeadingTaskState {
  FieldXDrive* drive{nullptr};
  int periodMs{10};
  pros::Task* task{nullptr};
};

HeadingTaskState g_state;

void headingTaskFn(void*) {
  uint32_t t = pros::millis();
  while (true) {
    if (g_state.drive) g_state.drive->updateHeading();
    pros::Task::delay_until(&t, g_state.periodMs);
  }
}
}  // namespace

// ------------------------------------------------------------
// Member: apply slew + push to motors
// powers array order: {LF, RB, RF, LB}
// setDrivePower expects: (FL, FR, BL, BR)
// ------------------------------------------------------------
void FieldXDrive::setMotorPowers(const std::array<int, 4>& powers) {
  int lf = powers[0];
  int rb = powers[1];
  int rf = powers[2];
  int lb = powers[3];

  // Slew limit per wheel
  lf = slewStep(lf, prevLF_, maxAccelPerLoop_);
  rb = slewStep(rb, prevRB_, maxAccelPerLoop_);
  rf = slewStep(rf, prevRF_, maxAccelPerLoop_);
  lb = slewStep(lb, prevLB_, maxAccelPerLoop_);

  // Push-Back setDrivePower order is (fl, fr, bl, br)
  setDrivePower(lf, rf, lb, rb);

  prevLF_ = lf;
  prevRB_ = rb;
  prevRF_ = rf;
  prevLB_ = lb;
}

// ------------------------------------------------------------
// Background heading task starter (replaces start20164XHeadingTask)
// ------------------------------------------------------------
void startHeadingTask(FieldXDrive& d, int periodMs) {
  if (g_state.task) return;
  g_state.drive = &d;
  g_state.periodMs = std::max(5, periodMs);
  g_state.task = new pros::Task(headingTaskFn, nullptr, "hdg");
}
