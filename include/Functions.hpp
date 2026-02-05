#pragma once

// Only include what you need here.
// If setDrivePower needs motor objects, include subsystems.hpp in the .cpp instead.
void setDrivePower(int fl, int fr, int bl, int br);

void IntakeSpin();
void DriveControl();
void DriveControlBackUp();
void IntakeReverse();

inline int IntakeLiftT = -1;
void IntakeLiftToggle();

void descoring();
void IntakeScoreToggle();
void ScoringP();
void MatchLoading();

inline int KnownState = 0;
void ArmAction();
