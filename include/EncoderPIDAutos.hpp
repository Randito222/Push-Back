// EncoderPIDAutos.hpp
#pragma once

#include "main.h"

// Drive encoder helpers
void resetDriveEncoders();
double getDriveAvgDegForward();
double getDriveAvgDegStrafe();

// Encoder-based PID moves (inches)
void driveForward_EncoderPID(double targetInches, double holdHeadingDeg, int maxDrive, int maxTurn, int timeoutMs);
void driveStrafe_EncoderPID(double targetInches,
                                        double holdHeadingDeg,
                                        int maxDrive,
                                        int maxTurn,
                                        int timeoutMs) ;

// IMU-based turn PID (degrees)
void turnToHeading_IMUPID(double targetDeg, int maxPower, int timeoutMs);

void Drive_EncoderPID(double targetInchesY,
                                         double targetInchesX,
                                         double holdHeadingDeg,
                                         int maxDrive,
                                         int maxTurn,
                                         int timeoutMs);