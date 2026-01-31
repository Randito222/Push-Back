#pragma once
#include "XDrive_PID.hpp"

// One-line default: FACE_TARGET + defaults
void followPath(const char* filenameOrAsset);

// Optional: choose heading mode and final heading
void followPath(const char* filenameOrAsset, HeadingMode mode, double finalHeadingDeg = 0.0);

// Async versions
void followPathAsync(const char* filenameOrAsset,
                     HeadingMode mode = HeadingMode::FACE_TARGET,
                     double finalHeadingDeg = 0.0);

bool isPathFollowing();
void waitPathDone(int timeout_ms = 6000);
void cancelPath();
