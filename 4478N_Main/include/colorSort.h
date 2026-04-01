#pragma once

// required files for devices
#include "main.h"
#include "api.h"
#include "devices.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>

// namespace for declarations
using namespace pros;
using namespace lemlib;

extern void colorLoopHigh();
extern void selectColor();
extern bool currentBallDetected;  // Add extern declaration for global variable