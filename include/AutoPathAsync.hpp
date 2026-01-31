#pragma once
#include "XDrive_PID.hpp"

void followPathAsync(const char* filenameOrAsset,
                     HeadingMode headingMode = HeadingMode::FACE_TARGET,
                     double finalHeadingDeg = 0.0);

bool isPathFollowing();
void waitPathDone(int timeout_ms = 6000);
void cancelPath();
