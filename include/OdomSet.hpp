#pragma once

// Field pose (inches, radians)
// Requested VEX Gaming Positioning System mapping:
//   odomX = forward / up-field axis
//   odomY = right   / across-field axis
// Heading:
//   odomTheta radians, 0 points along +odomX, positive towards +odomY.
extern double odomX;
extern double odomY;
extern double odomTheta;

void updateOdom();
void resetOdom();                 // resets to (0,0,0)
void printOdom();

// Reset to a specific pose (degrees or radians)
void resetOdomPose(double x_in, double y_in, double thetaDeg);
void resetOdomPoseRad(double x_in, double y_in, double thetaRad);

// Task entry
void odomTask(void* ignore);
