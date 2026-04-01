#include "devices.h"
#include "autonSelector.h"
#include <string>
using namespace pros;
using namespace lemlib;

const int numAutons = 15;

// ---------- auton selector (rotation sensor) ----------
int getAutonSelection()
{
    double rotation = autonSelector.get_position() / 100.0;

    while (rotation < 0)
        rotation += 360;
    rotation = fmod(rotation, 360);

    int segmentSize = 360 / numAutons;
    return (int)(rotation / segmentSize);
}

void displaySelection()
{
    int selection = getAutonSelection();
    pros::lcd::set_text(3, "Autonomous Selector");

    std::string autonName;
    switch (selection)
    {
    case 0:
        autonName = "Skills";
        break;
    case 1:
        autonName = "PID Testing";
        break;
    case 2:
        autonName = "Solo AWP";
        break;
    case 3:
        autonName = "Left Kim";
        break;
    case 4:
        autonName = "Left 4-3 Long First";
        break;
    case 5:
        autonName = "Right Kim";
        break;
    case 6:
        autonName = "Right 4-3 Long First";
        break;
    case 7:
        autonName = "Left 7 Ball Hold";
        break;
    case 8:
        autonName = "Left 7 Ball Wing";
        break;
    case 9:
        autonName = "Right 7 Ball Hold";
        break;
    case 10:
        autonName = "Right 7 Ball Wing";
        break;
    case 11:
        autonName = "Left 4 Wing";
        break;
    case 12:
        autonName = "Right 4 Wing";
        break;
    case 13:
        autonName = "Reserved";
        break;
    case 14:
        autonName = "Reserved 2";
        break;
    default:
        autonName = "Unknown";
    }

    // Show the selected routine name and the raw rotation sensor value
    pros::lcd::set_text(4, "Selected: " + autonName);
}