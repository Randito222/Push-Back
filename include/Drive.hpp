#pragma once

#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <cmath>
#include <algorithm>

// Field-centric X-drive helper.
// - Cached IMU heading (updated in a background task)
// - Sine-based X-drive mixing (smooth diagonals)
// - Per-wheel slew limiting (repeatability)
//
// Output units: PROS "move" units [-127, 127].
// setMotorPowers uses setDrivePower(fl, fr, bl, br) from Functions.cpp
class FieldXDrive {
 public:
  explicit FieldXDrive(pros::Imu* imu) : imu_(imu) {}

  // Call once at init. Blocks until IMU done calibrating.
  void initializeImu(int inertialPeriodMs = 10) {
    if (!imu_) return;
    imu_->reset();
    while (imu_->is_calibrating()) pros::delay(inertialPeriodMs);
    updateHeading();
  }

  // Update cached heading. Call from a background task at a consistent rate.
  void updateHeading() {
     if (!imu_) return;

  // get_rotation() respects set_rotation(0)
  double deg = imu_->get_rotation();  // can be >360 or negative depending on PROS version
  double rad = deg * (M_PI / 180.0);

  // wrap to [0, 2pi)
  rad = std::remainder(rad, 2.0 * M_PI);
  if (rad < 0) rad += 2.0 * M_PI;

  headingRad_ = rad;
}

  // Heading in radians, range [0, 2pi).
  double getHeadingRad() const {
    double a = std::remainder(headingRad_, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a;
  }

  // Calculate wheel powers for field-centric motion.
  // moveHeadingRad: direction of travel on FIELD (0=+X/right, pi/2=+Y/forward)
  // moveSpeed: 0..1
  // rotationHeadingRad: target heading on FIELD (0..2pi)
  // rotationSpeed: 0..1 (max turn effort)
  std::array<int, 4> calculateMotorPowers(double moveHeadingRad,
                                         double moveSpeed,
                                         double rotationHeadingRad,
                                         double rotationSpeed) const {
    moveSpeed     = std::clamp(moveSpeed,     0.0, 1.0);
    rotationSpeed = std::clamp(rotationSpeed, 0.0, 1.0);

    // If move+rot > 1, scale both down proportionally (original feel)
    if (moveSpeed + rotationSpeed > 1.0) {
      double total = moveSpeed + rotationSpeed;
      moveSpeed /= total;
      rotationSpeed /= total;
    }

    auto wrap2pi = [](double a) {
      a = std::remainder(a, 2.0 * M_PI);
      if (a < 0) a += 2.0 * M_PI;
      return a;
    };

    const double h = getHeadingRad();
    const double rotHeading = wrap2pi(rotationHeadingRad);

    // Original rotation sign logic (kept)
    double rotSpeedSigned;
    double diff = wrap2pi(h - rotHeading);
    if (diff > M_PI) diff -= 2.0 * M_PI;  // [-pi, pi)

    if ((h - rotHeading) > M_PI || ((h - rotHeading) < 0.0 && (h - rotHeading) > -M_PI)) {
      rotSpeedSigned = rotationSpeed;
    } else {
      rotSpeedSigned = -rotationSpeed;
    }

    // Field-centric sine mix
    double lf = std::sin(moveHeadingRad + M_PI / 4.0 + h);
    double rb = std::sin(moveHeadingRad + M_PI / 4.0 + h);
    double rf = std::sin(moveHeadingRad - M_PI / 4.0 + h);
    double lb = std::sin(moveHeadingRad - M_PI / 4.0 + h);

    double mult = 1.0 / std::max({std::fabs(lf), std::fabs(rf), std::fabs(lb), std::fabs(rb), 1e-6});

    int lfInt = (int)std::floor(lf * 127.0 * moveSpeed * mult) - (int)std::floor(127.0 * rotSpeedSigned);
    int rbInt = (int)std::floor(rb * 127.0 * moveSpeed * mult) + (int)std::floor(127.0 * rotSpeedSigned);
    int rfInt = (int)std::floor(rf * 127.0 * moveSpeed * mult) + (int)std::floor(127.0 * rotSpeedSigned);
    int lbInt = (int)std::floor(lb * 127.0 * moveSpeed * mult) - (int)std::floor(127.0 * rotSpeedSigned);

    lfInt = std::clamp(lfInt, -127, 127);
    rbInt = std::clamp(rbInt, -127, 127);
    rfInt = std::clamp(rfInt, -127, 127);
    lbInt = std::clamp(lbInt, -127, 127);

    return {lfInt, rbInt, rfInt, lbInt};
  }

  // Apply slew + send to drivetrain (defined in Drive.cpp).
  void setMotorPowers(const std::array<int, 4>& powers);

  void stop() {
    setMotorPowers({0,0,0,0});
    prevLF_ = prevRB_ = prevRF_ = prevLB_ = 0;
  }

  void setMaxAccelPerLoop(int step) { maxAccelPerLoop_ = std::max(1, step); }

  int getMaxAccelPerLoop() const { return maxAccelPerLoop_; }

  // Slew state is public-read for debugging if needed.
  int prevLF_{0}, prevRB_{0}, prevRF_{0}, prevLB_{0};

 private:
  pros::Imu* imu_{nullptr};
  volatile double headingRad_{0.0};
  int maxAccelPerLoop_{10};
};

// Global field-centric drive instance (defined in ONE .cpp)
extern FieldXDrive drive;

// Start a background task that calls drive.updateHeading() every periodMs.
// Safe to call multiple times; it will only start once.
void startHeadingTask(FieldXDrive& drive, int periodMs = 10);
