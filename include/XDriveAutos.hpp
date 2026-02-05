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

void driveFieldInches(
    double northIn,          // + = north (+Y), - = south
    double eastIn,           // + = east (+X),  - = west
    double holdHeadingDeg,   // keep facing this heading while moving
    int maxDrive,
    int maxTurn,
    int timeoutMs
);