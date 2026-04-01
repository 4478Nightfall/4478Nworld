#include "devices.h"
#include "opticalAlign.h"
#include "pros/distance.hpp"
#include "moveFunctions.h"
#include "pros/imu.hpp"
#include <cmath>
#include "moveFunctions.h"
using namespace pros;

// ---------- long goal align ----------
// dist mm, angles deg unless noted
const double minGoalDist = 680.0;
const double maxGoalDist = 1000.0;
const double turnSpd = 30.0;
const double backupSpd = 100.0;
const double alignTolerance = 4.0;
const double maxScanAngle = 40.0;
const double scanStartAngle = 255.0;
const double scanEndAngle = 360.0;
const double extraTurnAngle = 0.5;
const int sustainedDetectFrames = 4;



bool detectsGoal() {
    int objSize = backDistance.get_object_size();
    const int minMediumSize = 30;
    const int maxMediumSize = 69;
    return (objSize >= minMediumSize && objSize <= maxMediumSize);
}

double headingOfGoal; 

void MLthenGoal(double heading){
    bool goalDetected = false;
    const double minDetectHeading = -100.0;
    const double maxDetectHeading = -75.0;
    chassis.turnToHeading(heading, 800, {.maxSpeed = 90},false);
       left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while (chassis.isInMotion()){
        bool isDetected = detectsGoal();
        double currHeading = chassis.getPose().theta;
        if (isDetected && currHeading >= minDetectHeading && currHeading <= maxDetectHeading){
                headingOfGoal = chassis.getPose().theta;
                goalDetected = true;
                chassis.cancelAllMotions();
                break;
            }
        else {
        }
        pros::delay(20);
    }
    
    if (goalDetected){
        drivePID(20, 35, 1000);
        turnToHeadingSmart(headingOfGoal, 800);
        backIntoLongGoalML();
        
    }
    else {
        drivePID(20, 35, 1200);
alignToLongGoal();
    }
}

bool scanForGoal(double& targetHeading) {
    double startHeading = scanStartAngle;
    double endHeading = scanEndAngle;
    bool goalFound = false;

    turnToHeadingSmart(startHeading, 500, {.minSpeed = 100,.earlyExitRange = 5},false);
    turnToHeadingSmart(endHeading, 1000, {.maxSpeed = 28});

    int detectCount = 0;
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu1.get_heading();
        bool isDetected = detectsGoal();

        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Detected: %s (%d/%d)", isDetected ? "YES" : "NO", detectCount, sustainedDetectFrames);
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, endHeading);

        if (isDetected) {
            detectCount++;
            if (detectCount >= sustainedDetectFrames) {
                chassis.cancelAllMotions();
                targetHeading = currHeading;
                goalFound = true;
                left_motors.brake();
                right_motors.brake();
                pros::delay(150);
                return true;
            }
        } else {
            detectCount = 0;
        }

        pros::delay(20);
    }

    left_motors.brake();
    right_motors.brake();
    return goalFound;
}

void backIntoGoal(double targetHeading, bool constantSpeed = false) {
    const double backupTimeout = 950;    // Max time for backup attempt
    const double minApproachBeforeFinishMs = 450; // After this, losing target usually means we're already at goal
    double startTime = pros::millis();
    int lostObjCount = 0;                 // Count frames where goal isn't seen
    const int lostObjThreshold = 5;       // Try recovery after this many lost frames
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while (pros::millis() - startTime < backupTimeout) {
       
        
        double currHeading = imu1.get_heading();
        double headingErr = targetHeading - currHeading;
        
        // Normalize heading error to [-180, 180]
        while (headingErr > 180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;
        
        // Simple proportional correction (1.5 = P gain)
        double correction = headingErr * 1.5;
         
        // Limit correction power
        if (correction > 30) correction = 30;
        if (correction < -30) correction = -30;
        
        // Constant speed, or slow down after 600ms when not constant
        double elapsedTime = pros::millis() - startTime;
        double currentSpeed = constantSpeed ? backupSpd : ((elapsedTime >= 600) ? 30.0 : backupSpd);
        
        // Apply correction differentially
        double leftPwr = -currentSpeed + correction;
        double rightPwr = -currentSpeed - correction;
        
        left_motors.move(leftPwr);
        right_motors.move(rightPwr);
 
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
}

void alignToLongGoal() {
    double targetHeading = 0;
    // Try to find goal, if found back into it
    if (scanForGoal(targetHeading)) {
        backIntoGoal(targetHeading);
    } else {
        // No goal found - stop safely
        left_motors.brake();
        right_motors.brake();
    }
}

bool scanForGoalOpposite(double& targetHeading) {
    // Sweep from left to right looking for goal (opposite side - centered around 180)
    double startHeading = 75.0;  // Left scan limit (180 - 35)
    double endHeading = 150;     // Right scan limit (180 + 35)
    bool goalFound = false;

    turnToHeadingSmart(startHeading, 500, {.minSpeed = 100,.earlyExitRange = 5},false);
    turnToHeadingSmart(endHeading, 1000, {.maxSpeed = 28});

    int detectCount = 0;
    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu.get_heading();
        bool isDetected = detectsGoal();

        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Detected: %s (%d/%d)", isDetected ? "YES" : "NO", detectCount, sustainedDetectFrames);
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, endHeading);

        if (isDetected) {
            detectCount++;
            if (detectCount >= sustainedDetectFrames) {
                chassis.cancelAllMotions();
                targetHeading = currHeading;
                goalFound = true;
                left_motors.brake();
                right_motors.brake();
                pros::delay(150);
                return true;
            }
        } else {
            detectCount = 0;
        }

        pros::delay(20);
    }

    left_motors.brake();
    right_motors.brake();
    return goalFound;
}

void backIntoGoalOpposite(double targetHeading) {
    const double backupTimeout = 900;    // Max time for backup attempt
    double startTime = pros::millis();
    int lostObjCount = 0;                 // Count frames where goal isn't seen
    const int lostObjThreshold = 5;       // Try recovery after this many lost frames
    
    while (pros::millis() - startTime < backupTimeout) {
        
        
        double currHeading = imu1.get_heading();
        double headingErr = targetHeading - currHeading;
        
        // Normalize heading error to [-180, 180]
        while (headingErr > 180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;
        
        // Simple proportional correction (1.5 = P gain)
        double correction = headingErr * 1.5;
         
        // Limit correction power
        if (correction > 30) correction = 30;
        if (correction < -30) correction = -30;
        
        // Slow down to 30 after 1 second has elapsed
        double elapsedTime = pros::millis() - startTime;
        double currentSpeed = (elapsedTime >= 650) ? 30.0 : backupSpd;
        
        // Apply correction differentially
        double leftPwr = -currentSpeed + correction;
        double rightPwr = -currentSpeed - correction;
        
        left_motors.move(leftPwr);
        right_motors.move(rightPwr);
 
        
        pros::delay(20);
    }
    
    left_motors.brake();
    right_motors.brake();
}

void alignToLongGoalOpposite() {
    double targetHeading = 180.0;  // Default target heading for opposite side
    // Try to find goal, if found back into it
    if (scanForGoalOpposite(targetHeading)) {
        backIntoGoalOpposite(targetHeading);
    } else {
        // No goal found - stop safely
        left_motors.brake();
        right_motors.brake();
    }
}