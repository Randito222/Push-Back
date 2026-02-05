#pragma once

void driveToPoint_XDrive_PID(
    double txIn,
    double tyIn,
    double targetHeadingDeg,
    int maxDrive = 110,
    int maxTurn  = 70,
    int timeoutMs = 3000
);

void turnToHeading_PID(
    double targetHeadingDeg,
    int maxTurn = 90,
    int timeoutMs = 1500
);
