#pragma once

namespace DistanceOdom {
    // inches center to each distance sensor face (same numbers as cpp offsets)
    inline constexpr double kSensorCenterToFrontIn = 8.0;
    inline constexpr double kSensorCenterToBackIn = 4.0;
    inline constexpr double kSensorCenterToLeftIn = 4.5;
    inline constexpr double kSensorCenterToRightIn = 5.0;
}

void calcLeft();
void calcRight();
void calcFront();
void calcBack();
void resetOdom(bool Front, bool Back, bool Left, bool Right);
double mmToInches(double mm);
