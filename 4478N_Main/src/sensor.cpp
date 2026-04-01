#include "sensor.h"
#include "devices.h"
#include <cstdio>
#include <numeric>
#include <algorithm>
#include <cstring>

using namespace pros;

// ---------- sd datalog ----------
volatile bool dataLoggingRunning = false;
pros::Task* dataLoggingTask = nullptr;
static char datalogFilename[64] = "";

const char* getDatalogFilename()
{
    if (strlen(datalogFilename) > 0)
    {
        return datalogFilename;
    }
    
    // Check if base datalog.csv exists
    FILE *testFile = std::fopen("/usd/datalog.csv", "r");
    bool baseExists = (testFile != nullptr);
    if (testFile)
    {
        std::fclose(testFile);
    }
    
    // If base file doesn't exist, use it
    if (!baseExists)
    {
        std::strncpy(datalogFilename, "/usd/datalog.csv", sizeof(datalogFilename) - 1);
        datalogFilename[sizeof(datalogFilename) - 1] = '\0';
        return datalogFilename;
    }
    
    // Base file exists, find next available numbered file
    int fileNum = 1;
    char testFilename[64];
    do
    {
        std::snprintf(testFilename, sizeof(testFilename), "/usd/datalog_%d.csv", fileNum);
        testFile = std::fopen(testFilename, "r");
        if (testFile)
        {
            // File exists, try next number
            std::fclose(testFile);
            fileNum++;
        }
        else
        {
            // Found available filename
            std::strncpy(datalogFilename, testFilename, sizeof(datalogFilename) - 1);
            datalogFilename[sizeof(datalogFilename) - 1] = '\0';
            return datalogFilename;
        }
    } while (fileNum < 1000); // Safety limit
    
    // Fallback (should never reach here)
    std::strncpy(datalogFilename, "/usd/datalog.csv", sizeof(datalogFilename) - 1);
    datalogFilename[sizeof(datalogFilename) - 1] = '\0';
    return datalogFilename;
}

void logData()
{
    const char* filename = getDatalogFilename();
    FILE *file = std::fopen(filename, "a");
    if (!file)
    {
        pros::lcd::set_text(0, "SD open fail");
        return;
    }
    static bool headerWritten = false;
    static char lastFilename[64] = "";

    if (std::strlen(lastFilename) == 0 || std::strcmp(filename, lastFilename) != 0)
    {
        headerWritten = false;
        std::strncpy(lastFilename, filename, sizeof(lastFilename) - 1);
        lastFilename[sizeof(lastFilename) - 1] = '\0';
    }
    
    if (!headerWritten)
    {
        std::fseek(file, 0, SEEK_END);
        if (std::ftell(file) == 0)
        {
            std::fprintf(file, "timestamp,left_temp,left_vel,left_deg,right_temp,right_vel,right_deg,rotation_deg,autonSelector_deg,hTracker_deg,imu_heading,mfl_temp,mfl_vel,mfl_deg,mml_temp,mml_vel,mml_deg,mbl_temp,mbl_vel,mbl_deg,mfr_temp,mfr_vel,mfr_deg,mmr_temp,mmr_vel,mmr_deg,mbr_temp,mbr_vel,mbr_deg,imu_rotation,imu_pitch,imu_roll\n");
        }
        headerWritten = true;
    }
    uint32_t timestamp = pros::millis();

    auto leftTemps = left_motors.get_temperature_all();
    auto leftVels = left_motors.get_actual_velocity_all();
    auto leftDegs = left_motors.get_position_all();
    double leftTemp = 0, leftVel = 0, leftDeg = 0;
    if (!leftTemps.empty())
        leftTemp = std::accumulate(leftTemps.begin(), leftTemps.end(), 0.0) / leftTemps.size();
    if (!leftVels.empty())
        leftVel = std::accumulate(leftVels.begin(), leftVels.end(), 0.0) / leftVels.size();
    if (!leftDegs.empty())
        leftDeg = std::accumulate(leftDegs.begin(), leftDegs.end(), 0.0) / leftDegs.size();

    auto rightTemps = right_motors.get_temperature_all();
    auto rightVels = right_motors.get_actual_velocity_all();
    auto rightDegs = right_motors.get_position_all();
    double rightTemp = 0, rightVel = 0, rightDeg = 0;
    if (!rightTemps.empty())
        rightTemp = std::accumulate(rightTemps.begin(), rightTemps.end(), 0.0) / rightTemps.size();
    if (!rightVels.empty())
        rightVel = std::accumulate(rightVels.begin(), rightVels.end(), 0.0) / rightVels.size();
    if (!rightDegs.empty())
        rightDeg = std::accumulate(rightDegs.begin(), rightDegs.end(), 0.0) / rightDegs.size();

    // median of 6 wheel encoders (mml/mmr negated)
    double positions[6] = {
        mbl.get_position(), mfl.get_position(), -mml.get_position(),
        mbr.get_position(), mfr.get_position(), -mmr.get_position()
    };
    std::sort(positions, positions + 6);
    double rotationDeg = (positions[2] + positions[3]) / 2.0;

    double autonSelectorDeg = autonSelector.get_position() / 100.0;
    double hTrackerDeg = hTracker.get_position() / 100.0;
    double imuHeading = imu2.get_heading();

    double mflTemp = mfl.get_temperature();
    double mflVel = mfl.get_actual_velocity();
    double mflDeg = mfl.get_position();
    
    double mmlTemp = mml.get_temperature();
    double mmlVel = -mml.get_actual_velocity();
    double mmlDeg = -mml.get_position();
    
    double mblTemp = mbl.get_temperature();
    double mblVel = mbl.get_actual_velocity();
    double mblDeg = mbl.get_position();

    double mfrTemp = mfr.get_temperature();
    double mfrVel = mfr.get_actual_velocity();
    double mfrDeg = mfr.get_position();
    
    double mmrTemp = mmr.get_temperature();
    double mmrVel = -mmr.get_actual_velocity();
    double mmrDeg = -mmr.get_position();
    
    double mbrTemp = mbr.get_temperature();
    double mbrVel = mbr.get_actual_velocity();
    double mbrDeg = mbr.get_position();
    
    double imuRotation = imu2.get_rotation();
    double imuPitch = imu2.get_pitch();
    double imuRoll = imu2.get_roll();

    std::fprintf(file, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        timestamp,
        leftTemp,
        leftVel,
        leftDeg,
        rightTemp,
        rightVel,
        rightDeg,
        rotationDeg,
        autonSelectorDeg,
        hTrackerDeg,
        imuHeading,
        mflTemp,
        mflVel,
        mflDeg,
        mmlTemp,
        mmlVel,
        mmlDeg,
        mblTemp,
        mblVel,
        mblDeg,
        mfrTemp,
        mfrVel,
        mfrDeg,
        mmrTemp,
        mmrVel,
        mmrDeg,
        mbrTemp,
        mbrVel,
        mbrDeg,
        imuRotation,
        imuPitch,
        imuRoll);
    
    std::fclose(file);
}

void dataloggingLoopFn(void *)
{
    while (dataLoggingRunning)
    {
        logData();
        pros::delay(100);
    }
}
void startDatalogging()
{
    if (dataLoggingRunning)
        return;
    
    datalogFilename[0] = '\0';

    dataLoggingRunning = true;
    dataLoggingTask = new pros::Task(dataloggingLoopFn, nullptr, "Datalogging Task");
}
void stopDatalogging()
{
    dataLoggingRunning = false;
    if (dataLoggingTask)
    {
        dataLoggingTask->remove();
        delete dataLoggingTask;
        dataLoggingTask = nullptr;
    }
    datalogFilename[0] = '\0';
}

