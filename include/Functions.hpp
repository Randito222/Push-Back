#pragma once
#include "subsystems.hpp"

// =============================
// Drive
// =============================
void setDrivePower(int fl, int fr, int bl, int br);
void DriveControl();
void DriveControlBackUp();

// =============================
// Intake / Mechanisms
// =============================
void IntakeSpin();
void IntakeReverse();

// Intake lift toggle state
extern int IntakeLiftT;
void IntakeLiftToggle();

// =============================
// Scoring / Pneumatics
// =============================
void descoreLeftT();
// void descoreRight();   // removed unless used
void IntakeScoreToggle();
void ScoringP();
void MatchLoading();

// =============================
// Arm
// =============================
extern int KnownState;
void ArmAction();
