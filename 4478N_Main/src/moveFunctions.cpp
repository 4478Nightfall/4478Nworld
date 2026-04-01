#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton.h"
#include "moveFunctions.h"
#include <string>
#include <iostream>
#include <thread>
#include <algorithm>
#include <atomic>
#include <memory>
#include "pros/rtos.hpp"

using namespace pros;
using namespace lemlib;

// ---------- drive / intake / pid utils ----------

void handleMidGoal(){
    if (backBottom.get_actual_velocity() < 0 && middleStage.get_actual_velocity() > 0){
        midGoal.set_value(HIGH);
    }
    else{
        midGoal.set_value(LOW);
    }
}

void adjustLongClose(){
    if (chassis.getPose().theta > 270){
        chassis.swingToHeading(270, DriveSide::LEFT, 500, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.minSpeed = 60}, false);
    }
    else if (chassis.getPose().theta < 270){
        chassis.swingToHeading(270, DriveSide::RIGHT, 500, {.direction=AngularDirection::CW_CLOCKWISE,.minSpeed = 60}, false);
    }
}

static pros::Task* handleMidGoalTaskPtr = nullptr;

static void handleMidGoalTaskFn(void*) {
    while (true) {
        handleMidGoal();
        pros::delay(20);
    }
}

void startHandleMidGoalTask() {
    if (handleMidGoalTaskPtr == nullptr) {
        handleMidGoalTaskPtr = new pros::Task(
            handleMidGoalTaskFn,
            nullptr,
            TASK_PRIORITY_DEFAULT,
            4096,
            "handleMidGoal");
    }
}

double xPos = 0;
double yPos = 0;
double theta = 0;

void setPose(){
     xPos = chassis.getPose().x;
     yPos = chassis.getPose().y;
     theta = chassis.getPose().theta;
}
void holdPush(){
   if (chassis.getPose().x != xPos || chassis.getPose().y != yPos || chassis.getPose().theta != theta){
       chassis.moveToPose(xPos, yPos, theta, 1000, {},false);
   }
    
}

void wing(){
    chassis.setPose(-31.232,-47.334,chassis.getPose().theta);
    delay(100);
    handleDescore();
    chassis.moveToPose(-55.692,-34.036,270,2000,{.forwards=true, .lead=.15,.minSpeed=60},false);
    
    turnToHeadingSmart(275,600,{},false);

    drivePID(-37,100,2000);
    while(true){
    chassis.swingToHeading(265,DriveSide::RIGHT,300,{.minSpeed=70},true); 
    delay(20);

    }
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motors.brake();
    right_motors.brake();
}

void spinIntake(double direction)
{
    firstStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    middleStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    backBottom.set_brake_mode(MOTOR_BRAKE_HOLD);
    firstStage.move(direction * 127);
    middleStage.move(direction * 127);
    backBottom.move(direction * 127);
}


void scoreMiddle(){
    firstStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    middleStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    backBottom.set_brake_mode(MOTOR_BRAKE_HOLD);
    firstStage.move(57);
    middleStage.move(57);
    backBottom.move(-57);
}

void stopIntake()
{
    firstStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    middleStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    backBottom.set_brake_mode(MOTOR_BRAKE_HOLD);
    firstStage.brake();
    middleStage.brake();
    backBottom.brake();
}

void scoreLongClose(int time){
    spinIntake(1);
   drivePID(-10,100,400);
chassis.turnToHeading(270,1000);
drivePID(-100,100,1000);
    
}

void scoreLongFar(int time){
    spinIntake(1);
    drivePID(-10,100,400);
chassis.turnToHeading(90,1000);
drivePID(-10,100,400);
}

void matchLoad(int timeMs){
    double start = pros::millis();
    const int wiggleInterval = 300;   // ms between each wiggle
    const int wigglePower = 40;     // motor power for wiggle
    bool turnLeft = true;
    while (pros::millis() - start < timeMs) {
        if (turnLeft) {
            left_motors.move(-wigglePower/2);
            right_motors.move(wigglePower);
        } else {
            left_motors.move(wigglePower);
            right_motors.move(-wigglePower/2);
        }
        turnLeft = !turnLeft;
        pros::delay(wiggleInterval);
    }
    left_motors.brake();
    right_motors.brake();
}

void handleHood()
{
    hood.set_value(hood.get_value() == LOW ? HIGH : LOW);
}

void handleDescore()
{
    descore.set_value(descore.get_value() == LOW ? HIGH : LOW);
}

void handleMLMech()
{
    matchloadMech.set_value(matchloadMech.get_value() == LOW ? HIGH : LOW);
}

double slewStep = 20.0;
double slewRate = 0.5;

double slew(double val, double fwdVal)
{
    static double prevVal = 0;
    static double prevFwdVal = 0;

    // Reset slew if target direction changes significantly
    if ((fwdVal >= 0 && prevFwdVal < 0) || (fwdVal < 0 && prevFwdVal >= 0))
    {
        prevVal = 0;
    }
    prevFwdVal = fwdVal;

    // Calculate the difference between target and current
    double difference = val - prevVal;

    // If difference is small, just return the target
    if (fabs(difference) < 0.5)
    {
        prevVal = val;
        return val;
    }

    // Calculate slew rate based on difference (adaptive)
    double currentSlewStep = slewStep + (fabs(difference) * slewRate);

    // Apply slew rate limiting
    if (difference > 0)
    {
        // Positive direction
        if (prevVal + currentSlewStep < val)
        {
            prevVal += currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }
    else
    {
        // Negative direction
        if (prevVal - currentSlewStep > val)
        {
            prevVal -= currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }

    return prevVal;
}

void drivePID(double fwdVal, double maxSpeedPercent, double timeout)
{
    
    double kP = 0.2; 
    double kI = 0.000000; 
    double kD = 0.17; 

    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;

    double num = fwdVal;
    double denom = (diameter * pi) * (inputGear / outputGear);
    double target = (num / denom) * 360;
    
    left_motors.tare_position();
    right_motors.tare_position();
    mbl.tare_position();
    mbr.tare_position();
    mfl.tare_position();
    mfr.tare_position();
    mml.tare_position();
    mmr.tare_position();

    double startTime = pros::millis();
    double error = 0;
    double prevError = 0;
    double integral = 0;
    
    const double pollingRate = 20;
    const double errorThreshold = 5.0;
    int inGoal = 0;
    const int goalsNeeded = 3;
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false;
    
    while (inGoal < goalsNeeded)
    {
        double leftPositions[3] = {
            mbl.get_position(), mfl.get_position(), mml.get_position()
        };
        double rightPositions[3] = {
            mbr.get_position(), mfr.get_position(), mmr.get_position()
        };
        
        std::nth_element(leftPositions, leftPositions + 1, leftPositions + 3);
        double leftMedian = leftPositions[1];
        
        std::nth_element(rightPositions, rightPositions + 1, rightPositions + 3);
        double rightMedian = rightPositions[1];
        
        double medianPos = (leftMedian + rightMedian);
        double processVariable = medianPos * 360;
        
        error = target - processVariable;
        
        // Track if robot has moved - for small movements, lower threshold and longer time
        // This prevents immediate exit at start while allowing small movements to work
        if (fabs(processVariable) > 5 || fabs(error) < errorThreshold || (pros::millis() - startTime) > 150) {
            hasMoved = true;
        }

        double errorInches = fabs(error) * (denom / 360.0);
        double currentKP = kP;
        double currentKD = kD;
        

       
        // Proportional term
        double P = error * currentKP;
        
        // Integral term with anti-windup 
        integral += error;
        double I = integral * kI;
        I = std::clamp(I, -50.0, 50.0); // Clamp integral contribution
        
        // Derivative term (using scheduled gain)
        double D = ((error - prevError) / pollingRate) * currentKD;
        
        // Calculate total PID output
        double motorPower = P + I + D;
        
        // Apply slew only at start for smooth acceleration
        if ((pros::millis() - startTime) < 300) {
            motorPower = slew(motorPower, fwdVal);
        }
        
        // Clamp output
        motorPower = std::clamp(motorPower, -maxMotor, maxMotor);

        // Move motors
        left_motors.move(motorPower);
        right_motors.move(motorPower);

        if (hasMoved && fabs(error) < errorThreshold)
        {
            inGoal++;
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0;
        }

        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        pros::delay(pollingRate);
    }
    
    left_motors.brake();
    right_motors.brake();
}

static std::atomic<bool> drivePidStopRequested(false);
static std::atomic<bool> drivePidRunning(false);
static pros::Task* drivePidTaskPtr = nullptr;

struct DrivePidArgs {
    double fwdVal;
    double maxSpeedPercent;
    double timeout;
};

static void drivePidTaskFn(void* rawArgs) {
    std::unique_ptr<DrivePidArgs> args(reinterpret_cast<DrivePidArgs*>(rawArgs));
    double fwdVal = args->fwdVal;
    double maxSpeedPercent = args->maxSpeedPercent;
    double timeout = args->timeout;

    drivePidRunning = true;
    drivePidStopRequested = false;

    double kP = 0.2; 
    double kI = 0.000000; 
    double kD = 0.17; 

    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 48;
    const double inputGear = 36;

    double num = fwdVal;
    double denom = (diameter * pi) * (inputGear / outputGear);
    double target = (num / denom) * 360;

    left_motors.tare_position();
    right_motors.tare_position();
    mbl.tare_position();
    mbr.tare_position();
    mfl.tare_position();
    mfr.tare_position();
    mml.tare_position();
    mmr.tare_position();

    double startTime = pros::millis();
    double error = 0;
    double prevError = 0;
    double integral = 0;

    const double pollingRate = 20;
    const double errorThreshold = 5.0;
    int inGoal = 0;
    const int goalsNeeded = 3;
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false;

    while (inGoal < goalsNeeded && !drivePidStopRequested) {
        double leftPositions[3] = {
            mbl.get_position(), mfl.get_position(), mml.get_position()
        };
        double rightPositions[3] = {
            mbr.get_position(), mfr.get_position(), mmr.get_position()
        };
        
        std::nth_element(leftPositions, leftPositions + 1, leftPositions + 3);
        double leftMedian = leftPositions[1];
        
        std::nth_element(rightPositions, rightPositions + 1, rightPositions + 3);
        double rightMedian = rightPositions[1];
        
        double medianPos = (leftMedian + rightMedian);
        double processVariable = medianPos * 360;
        
        error = target - processVariable;
        
        if (fabs(processVariable) > 5 || (pros::millis() - startTime) > 200) {
            hasMoved = true;
        }

        double errorInches = fabs(error) * (denom / 360.0);
        double currentKP = kP;
        double currentKD = kD;
        
       
        // Proportional term
        double P = error * currentKP;
        
        // Integral term with anti-windup 
        integral += error;
        double I = integral * kI;
        I = std::clamp(I, -50.0, 50.0); // Clamp integral contribution
        
        // Derivative term (using scheduled gain)
        double D = ((error - prevError) / pollingRate) * currentKD;

        // Calculate total PID output
        double motorPower = P + I + D;
        
        // Apply slew only at start for smooth acceleration
        if ((pros::millis() - startTime) < 300) {
            motorPower = slew(motorPower, fwdVal);
        }
        
        // Clamp output
        motorPower = std::clamp(motorPower, -maxMotor, maxMotor);

        // Move motors
        left_motors.move(motorPower);
        right_motors.move(motorPower);

        if (hasMoved && fabs(error) < errorThreshold && (pros::millis() - startTime) > 100)
        {
            inGoal++;
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0;
        }

        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        pros::delay(pollingRate);
    }

    left_motors.brake();
    right_motors.brake();

    drivePidRunning = false;
    return;
}

void drivePIDAsync(double fwdVal, double maxSpeedPercent, double timeout) {
    if (drivePidRunning) {
        drivePidStopRequested = true;
        for (int i = 0; i < 50 && drivePidRunning; ++i) {
            pros::delay(10);
        }
    }

    auto* args = new DrivePidArgs{fwdVal, maxSpeedPercent, timeout};

    if (drivePidTaskPtr != nullptr && !drivePidRunning) {
        delete drivePidTaskPtr;
        drivePidTaskPtr = nullptr;
    }

    drivePidTaskPtr = new pros::Task(
        drivePidTaskFn,
        args,
        TASK_PRIORITY_DEFAULT,
        4096,
        "drivePIDAsync");
}
// ...existing code...

void stopDrivePIDAsync() {
    drivePidStopRequested = true;
    for (int i = 0; i < 100 && drivePidRunning; ++i) {
        pros::delay(10);
    }
    if (drivePidTaskPtr != nullptr && !drivePidRunning) {
        delete drivePidTaskPtr;
        drivePidTaskPtr = nullptr;
    }
}

bool isDrivePIDRunning() {
    return static_cast<bool>(drivePidRunning);
}

void turnPID(double turnVal, double maxSpeedPercent)
{
    // Simple PID constants for turning
    double kP = 2.0; // Proportional constant
    double kD = 0.5; // Derivative constant
    // No integral for simplicity

    // Simple timing
    double startTime = pros::millis();
    double prevTime = startTime;
    double prevError = 0;
    double target = turnVal;

    // Convert percent cap to motor units
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    // Simple loop
    while (true)
    {
        double currentTime = pros::millis();
        double deltaTime = currentTime - prevTime;
        if (deltaTime == 0) deltaTime = 1; // Prevent division by zero
        prevTime = currentTime;

        // Get current heading
        double currentDeg = imu.get_heading();

        // Calculate error
        double error = target - currentDeg;

        // Normalize error to range [-180, 180]
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        // Calculate derivative
        double derivative = (error - prevError) / deltaTime;
        prevError = error;

        // Calculate output
        double output = (error * kP) + (derivative * kD);

        // Limit output to requested cap
        if (output > maxMotor)
            output = maxMotor;
        if (output < -maxMotor)
            output = -maxMotor;

        // Move motors (differential for turning)
        left_motors.move(output);
        right_motors.move(-output);

        // Simple stop condition
        if (fabs(error) < 3)
        {
            left_motors.brake();
            right_motors.brake();
            break;
        }

        pros::delay(20);
    }
}
void driveForTime(int power, int time)
{
    left_motors.move(power);
    right_motors.move(power);
    pros::delay(time);
    left_motors.brake();
    right_motors.brake();
}

// Back into long goal at full speed, then slow when close (for ML then goal).
// Uses back distance sensor only to switch to slow; slow phase runs for a fixed timeout then stops.
void backIntoLongGoalML()
{
    const double fullSpeed = 100.0;       // Full backup speed
    const double slowSpeed = 20.0;        // Speed when close to goal
    const double closeDistMm = 200.0;     // Below this distance (mm), switch to slow
    const double backupTimeout = 1300.0;  // ms max for full-speed phase (if we never get close)
    const double slowPhaseTimeout = 200.0; // ms to run at slow speed, then stop

    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    double startTime = pros::millis();
    double slowPhaseStart = -1;  // When we entered slow phase (-1 = not yet)

    while (true)
    {
        double elapsed = pros::millis() - startTime;
        double distMm = backDistance.get();
        bool isClose = (distMm > 0 && distMm < closeDistMm);

        if (isClose && slowPhaseStart < 0)
            slowPhaseStart = pros::millis();

        if (slowPhaseStart >= 0) {
            // In slow phase: run for slowPhaseTimeout then stop
            if (pros::millis() - slowPhaseStart >= slowPhaseTimeout)
                break;
            left_motors.move(-slowSpeed);
            right_motors.move(-slowSpeed);
        } else {
            // Full-speed phase: stop if total timeout (never got close)
            if (elapsed >= backupTimeout)
                break;
            left_motors.move(-fullSpeed);
            right_motors.move(-fullSpeed);
        }
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();
}

void turnToHeadingSmart(float theta, int timeout, TurnToHeadingParams params, bool async)
{
    // Get current heading
    float currentHeading = chassis.getPose().theta;
    
    // Calculate angle difference
    float angleDiff = theta - currentHeading;
    
    // Normalize to [-180, 180]
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    if (params.minSpeed == 0)
    {
        float absAngleDiff = fabs(angleDiff);
        
        if (absAngleDiff < 15.0)
        {
            params.minSpeed = 40; // Higher minimum speed for very small turns
            params.earlyExitRange = 2; // Tighter exit range for precision
        }
        else if (absAngleDiff < 30.0)
        {
            params.minSpeed = 30; // Moderate minimum speed
            params.earlyExitRange = 3;
        }
    }
    
    chassis.turnToHeading(theta, timeout, params, async);
}
