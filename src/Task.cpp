#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "subsystems.hpp"
// #include "pros/competition.hpp"

void Drive_Controls_swap() {
  bool fieldCentric = true;   // true = field, false = robot
  bool lastX = false;

  uint32_t lastToggleMs = 0;
  const uint32_t cooldownMs = 200;

  // Print once
  master.clear_line(2);
  master.print(2, 0, "Mode: Field");

  while (true) {
    bool xNow = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    if (xNow && !lastX) {
      uint32_t now = pros::millis();
      if (now - lastToggleMs >= cooldownMs) {
        lastToggleMs = now;
        fieldCentric = !fieldCentric;

        master.clear_line(2);
        master.print(2, 0, fieldCentric ? "Mode: Field" : "Mode: Robot");

        // Optional haptic
        master.rumble(fieldCentric ? "." : "..");
      }
    }

    DriveControlUnified(fieldCentric);

    lastX = xNow;
    pros::delay(10); // loop timing here, not inside the drive function
  }
}
