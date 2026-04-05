#pragma once


void calcLeft();
void calcRight();
void calcFront();
void calcBack();
void resetOdom(bool Front, bool Back, bool Left, bool Right);
/** After resetOdom(), returns x,y,theta passed to chassis.setPose (distance-based). */
bool getLastDistanceResetPose(double& outX, double& outY, double& outTheta);
double mmToInches(double mm);
