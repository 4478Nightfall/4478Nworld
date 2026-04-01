#include "devices.h"
#include <string>
using namespace pros;
using namespace lemlib;

// ---------- color sort (optical) ----------
int count = 0;
bool currentBallDetected = false;
bool lastBallDetected = false;
const int detectionCountThreshold = 2000;

void colorLoopHigh()
{
  currentBallDetected = false;

  if (currentTeam == blue)
  {
    if (colorSort.get_hue() >= 0 && colorSort.get_hue() <= 20)
    {
      currentBallDetected = true;
    }
  }
  else if (currentTeam == red)
  {
    if (colorSort.get_hue() >= 195 && colorSort.get_hue() <= 230)

    {
      currentBallDetected = true;
    }
  }
  else if (currentTeam == None)
  {
    currentBallDetected = false;
  }

  if (currentBallDetected == true)
  {
    count++;
    middleStage.move(127);
  }

  if (count >= detectionCountThreshold)
  {
    count = 0;
    middleStage.brake();
  }
}

// void colorLoopLow()
// {
//   // Reset count if no ball currently detected
//   currentBallDetected = false; // Reset global variable

//   // Check for ball based on team color
//   if (currentTeam == blue)
//   {
//     // Blue balls typically have hue between 200-240 (blue range)
//     if (colorSort.get_hue() > 0 && colorSort.get_hue() < 30)
//     {
//       currentBallDetected = true;
//     }
//   }
//   else if (currentTeam == red)
//   {
//     // Red balls typically have hue between 0-20 or 340-360 (red range)
//     if (colorSort.get_hue() >= 195 && colorSort.get_hue() <= 230)

//     {
//       currentBallDetected = true;
//     }
//   }
//   else if (currentTeam == None)
//   {
//     currentBallDetected = false;
//   }

//   // Handle solenoid control
//   if (currentBallDetected == true)
//   {
//     count++;
//     hood.set_value(LOW);
//     backBottom.move(127);
//     middleStage.move(127);

//   }

//   // Reset after detection period
//   if (count >= detectionCountThreshold)
//   {
//     count = 0;
//     backBottom.brake();
//     middleStage.brake();
//     currentBallDetected = false;
//   }
// }

void selectColor()
{
  // 0 = RED, 1 = BLUE, 2 = NONE
  static int selection = 0;
  static bool lastButtonState = false;
  const char *teamNames[3] = {"RED", "BLUE", "NONE"};

  pros::lcd::set_text(5, "Select Team Color:");
  pros::lcd::set_text(6, "Press CENTER to cycle");

  bool currentButtonState = pros::lcd::read_buttons() & LCD_BTN_CENTER;

  // Check for button press (rising edge)
  if (currentButtonState && !lastButtonState)
  {
    // Cycle through 0,1,2
    selection = (selection + 1) % 3;
    // Map selection to currentTeam (assumes red, blue, None are defined elsewhere)
    if (selection == 0)
    {
      currentTeam = red;
    }
    else if (selection == 1)
    {
      currentTeam = blue;
    }
    else
    {
      currentTeam = None;
    }
    pros::lcd::set_text(7, std::string("Current: ") + teamNames[selection]);
  }

  lastButtonState = currentButtonState;
}
