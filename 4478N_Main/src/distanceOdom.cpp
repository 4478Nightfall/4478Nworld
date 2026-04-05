#include "distanceOdom.h"
#include "devices.h"
#include "lemlib/api.hpp"
#include <cmath>

using namespace pros;
using namespace lemlib;

// ---------- distance odom ----------
// 140x140 field centered on 0,0 -- walls at +/-70. heading is lemlib theta (+Y is 0)
// body +x right +y fwd. calcs figure which wall youre facing and snap x or y

// sensor to center along each face (in) -- should match header constants
double frontOffset = 8.0;
double backOffset = 4.0;
double leftOffset = 5.5;
double rightOffset = 5.0;
// along face from center (in); + = forward on L/R, + = right on F/B; scaled by cos(angle to bore)
double frontEdgeOffset = 0;
double backEdgeOffset = 0;
double leftEdgeOffset = 0;
double rightEdgeOffset = 0;
const double fieldHalfSize = 71.0; // in
static double theta = 0.0;
static double x = chassis.getPose().x;
static double y = chassis.getPose().y;

static double lastDistResetX = 0.0;
static double lastDistResetY = 0.0;
static double lastDistResetTheta = 0.0;
static bool lastDistResetValid = false;

// misc helpers
double mmToInches(double mm) { return mm / 25.4; }
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
double norm360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0) {
        deg += 360.0;
    }
    return deg;
}
double smallestAngleDiffDeg(double aDeg, double bDeg) {
    double d = std::fabs(aDeg - bDeg);
    if (d > 180.0) {
        d = 360.0 - d;
    }
    return d;
}
// slant range * cos to get perp dist to wall
double perpAlongNormal(double rangeInches, double boreDeg, double normalDeg) {
    const double d = smallestAngleDiffDeg(norm360(boreDeg), norm360(normalDeg));
    return rangeInches * std::cos(degreesToRadians(d));
}
// edge offset along the face: +forward on L/R, +right on F/B (body); project onto boresight
static double edgeOnBore(double boreDeg, double alongFaceDeg, double edgeIn) {
    const double d = smallestAngleDiffDeg(norm360(boreDeg), norm360(alongFaceDeg));
    return edgeIn * std::cos(degreesToRadians(d));
}

// one calc per sensor - branches are which wall based on heading
void calcLeft(){
    const double boreL = theta - 90.0;
    double leftReading =
        (mmToInches(leftDistance->get()) + leftOffset + edgeOnBore(boreL, theta, leftEdgeOffset));

    if(theta <= 45 || theta > 315){
        x = -fieldHalfSize + perpAlongNormal(leftReading, boreL, 270.0);
    }
    else if(theta >= 135 && theta <= 225){
        x = fieldHalfSize - perpAlongNormal(leftReading, boreL, 90.0);
    }
    else if (theta >= 45 && theta <= 135){
        y = fieldHalfSize - perpAlongNormal(leftReading, boreL, 0.0);
    }
    else if(theta >= 225 && theta <= 315){
        y = -fieldHalfSize + perpAlongNormal(leftReading, boreL, 180.0);
    }
} 


void calcRight(){
    const double boreR = theta + 90.0; 
    double rightReading =
        (mmToInches(rightDistance->get()) + rightOffset + edgeOnBore(boreR, theta, rightEdgeOffset));

    if(theta <= 45 || theta > 315){
        x = fieldHalfSize - perpAlongNormal(rightReading, boreR, 90.0);
    }
    else if(theta >= 135 && theta <= 225){
        x = -fieldHalfSize  + perpAlongNormal(rightReading, boreR, 270.0);
    }
    else if (theta >= 45 && theta <= 135){
        y = -fieldHalfSize + perpAlongNormal(rightReading, boreR, 180.0);
    }
    else if(theta >= 225 && theta <= 315){
        y = fieldHalfSize - perpAlongNormal(rightReading, boreR, 0.0);
    }

}

void calcFront(){
    const double boreF = theta;
    double frontReading = (mmToInches(frontDistance->get()) + frontOffset +
                          edgeOnBore(boreF, theta + 90.0, frontEdgeOffset));

    if(theta >= 45 && theta <= 135){
        x = fieldHalfSize - perpAlongNormal(frontReading, boreF, 90.0);
    }
    else if(theta >= 225 && theta <= 315){
        x = -fieldHalfSize + perpAlongNormal(frontReading, boreF, 270.0);
    }
    else if (theta <= 45 || theta > 315){
        y = fieldHalfSize - perpAlongNormal(frontReading, boreF, 0.0);
    }
    else if(theta >= 135 && theta <= 225){
        y = -fieldHalfSize + perpAlongNormal(frontReading, boreF, 180.0);
    }
}

void calcBack(){
    const double boreB = theta + 180.0;
    double backReading = (mmToInches(backDistancePtr->get()) + backOffset +
                          edgeOnBore(boreB, theta + 90.0, backEdgeOffset));

    if(theta >= 45 && theta <= 135){
        x = -fieldHalfSize + perpAlongNormal(backReading, boreB, 270.0);
    }
    else if(theta >= 225 && theta <= 315){
        x = fieldHalfSize - perpAlongNormal(backReading, boreB, 90.0);
    }
    else if (theta <= 45 || theta > 315){
        y = -fieldHalfSize + perpAlongNormal(backReading, boreB, 180.0);
    }
    else if(theta >= 135 && theta <= 225){
        y = fieldHalfSize - perpAlongNormal(backReading, boreB, 0.0);
    }
}

// call from auton or tester -- bools = which sensors to trust, sets pose from walls
void resetOdom(bool Front, bool Back, bool Left, bool Right){
    theta = chassis.getPose().theta;
    x = 0.0;
    y = 0.0;

    if(Front){
        calcFront();
    }
    if(Back){
        calcBack(); 
    }
    if(Left){
        calcLeft();
    }
    if(Right){
        calcRight();
    }

    const double poseTheta = chassis.getPose().theta;
    chassis.setPose(x, y, poseTheta);
    lastDistResetX = x;
    lastDistResetY = y;
    lastDistResetTheta = poseTheta;
    lastDistResetValid = true;
}

bool getLastDistanceResetPose(double& outX, double& outY, double& outTheta) {
    if (!lastDistResetValid) {
        return false;
    }
    outX = lastDistResetX;
    outY = lastDistResetY;
    outTheta = lastDistResetTheta;
    return true;
}