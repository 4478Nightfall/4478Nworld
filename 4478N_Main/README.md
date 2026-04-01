# 4478N - VEX Robotics Competition Robot

A VEX V5 robotics competition robot built by Masuk 4478N Nightfall for the 2024-2025 VEX Robotics Competition season.

## Project Overview

This project contains the complete source code for a VEX V5 robot designed for competitive robotics. The robot features advanced autonomous capabilities, precise movement control, and a modular design for easy maintenance and upgrades.

## Robot Architecture

### Core Components
- **Chassis System**: Tank drive with left/right motor groups
- **Intake System**: Dual-stage intake with color sorting capabilities
- **Stage Mechanism**: Front stage for intaking
- **Gate System**: Configurable front and back gates for scoring
- **Match Load Mechanism**: Automated match loading system

### Sensors & Control
- **IMU**: Inertial Measurement Unit for orientation tracking
- **Optical Sensor**: Color detection and sorting
- **Rotation Sensors**: Position tracking and autonomous selection
- **V5 Controller**: Driver control interface

## Control Scheme

### Drive Control
- **Left Stick**: Left side tank drive
- **Right Stick**: Right side tank drive
- **Exponential Drive Curves**: Smooth acceleration and deceleration

### Mechanism Control
- **R1/R2**: Intake forward/reverse
- **L1/L2**: Front stage up/down
- **A Button**: Toggle back gate
- **Left Arrow**: Toggle front gate
- **B Button**: Toggle match load mechanism

### Autonomous Selection
- Use the rotation sensor on the robot to select autonomous routines
- LCD screen displays current selection and robot pose information
- Real-time position tracking with X, Y, and theta coordinates

## Technical Features

### Motion Control
- **LemLib Integration**: Advanced motion profiling and path following
- **PID Controllers**: Precise position and heading control
- **Odometry**: Real-time position tracking with tracking wheels
- **Chassis Calibration**: Automatic sensor calibration on startup

### Software Architecture
- **Modular Design**: Separated device definitions, autonomous routines, and movement functions
- **Task Management**: Background tasks for LCD updates and sensor monitoring
- **Error Handling**: Robust error handling and recovery systems

## Project Structure

```
4478N_Highlander/
├── src/                    # Source code files
│   ├── main.cpp           # Main robot program
│   ├── auton.cpp          # Autonomous routines
│   ├── devices.cpp        # Device initialization
│   ├── moveFunctions.cpp  # Movement and control functions
│   ├── sensor.cpp         # Sensor handling
│   └── mcl.cpp            # Monte Carlo localization
├── docs/                  # Design notes (e.g. MCL)
├── include/               # Header files
│   ├── devices.h          # Device declarations
│   ├── auton.h            # Autonomous function declarations
│   ├── moveFunctions.h    # Movement function declarations
│   ├── sensor.h           # Sensor function declarations
│   └── mcl.h              # MCL API
├── firmware/              # V5 firmware and libraries
├── bin/                   # Compiled binary output
└── Makefile               # Build configuration
```

## Dependencies

### Core Libraries
- **PROS 4.1.1**: VEX V5 development framework
- **LemLib 0.5.4**: Advanced motion control library
- **LVGL 8.3.8**: Graphics library for advanced UI

### Hardware Requirements
- VEX V5 Brain
- VEX V5 Controller
- V5 Smart Motors
- V5 Optical Sensor
- V5 IMU Sensor
- V5 Rotation Sensors
- ADI Digital Outputs (for gates)

## Getting Started

### Prerequisites

#### 1. Install Required Software
- **Git**: Download and install [Git](https://git-scm.com/downloads) for version control
- **PROS CLI**: Install [PROS CLI](https://pros.cs.purdue.edu/v5/getfixedting-started/index.html) for VEX V5 development
- **Code Editor**: Install [Visual Studio Code](https://code.visualstudio.com/) (recommended) or your preferred editor
- **GitHub Desktop** (optional): For easier Git operations

#### 2. Set Up Development Environment
```bash
# Verify Git installation
git --version

# Verify PROS CLI installation
pros --version

# Set up your Git identity (if first time)
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Setting Up the Project

#### 1. Clone the Repository
```bash
# Navigate to your desired development directory
cd C:\Users\YourUsername\Documents\VEX_Projects

# Clone the repository
git clone https://github.com/your-username/4478N_Highlander.git

# Navigate into the project directory
cd 4478N_Highlander
```

#### 2. Verify Project Structure
```bash
# List all files to ensure everything downloaded correctly
dir

# You should see:
# - src/ folder
# - include/ folder
# - Makefile
# - project.pros
# - README.md
```

### Building and Testing

#### 1. Build the Project
```bash
# Clean any previous builds
make clean

# Build the project
make

# If successful, you'll see "Build completed successfully"
```

#### 2. Upload to Robot
```bash
# Upload to V5 Brain (slot 1)
make upload

# Or specify a different slot
make upload SLOT=2
```

#### 3. Test Basic Functionality
- Verify robot responds to controller inputs
- Check LCD screen shows robot pose information
- Test autonomous selection with rotation sensor

### Development Workflow

#### 1. **Start Development**
```bash
# Open project in VS Code
code .

# Or open specific files
code src/main.cpp
code include/devices.h
```

#### 2. **Make Changes**
- Edit files in the `src/` directory
- Modify header files in `include/` directory
- Test changes incrementally

#### 3. **Build and Test Cycle**
```bash
# After making changes, build again
make

# If build succeeds, upload to robot
make upload

# Test functionality on robot
# Make adjustments as needed
```

#### 4. **Version Control Best Practices**
```bash
# Check what files have changed
git status

# Add your changes
git add .

# Commit with descriptive message
git commit -m "Added new autonomous routine for left side AWP"

# Push to remote repository (if working with team)
git push origin main
```

### Common Development Tasks

#### Adding New Autonomous Routines
1. **Edit `include/auton.h`**: Add function declaration
2. **Edit `src/auton.cpp`**: Implement the function
3. **Edit `src/main.cpp`**: Add case to autonomous switch statement
4. **Build and test**: Ensure new routine works correctly

#### Modifying Device Configuration
1. **Edit `include/devices.h`**: Update device declarations
2. **Edit `src/devices.cpp`**: Modify initialization code
3. **Update port numbers**: Ensure hardware matches software configuration

#### Adding New Sensors
1. **Include sensor header**: Add to `include/devices.h`
2. **Initialize sensor**: Add to `src/devices.cpp`
3. **Add to autonomous**: Integrate with existing routines

### Troubleshooting Development Issues

#### Build Errors
```bash
# Clean and rebuild
make clean
make

# Check for syntax errors in specific files
# Verify all includes are correct
```

#### Upload Issues
- Ensure V5 Brain is connected via USB
- Check that robot is powered on
- Verify correct slot selection
- Try different USB ports

#### Git Issues
```bash
# If you get merge conflicts
git status
git add .
git commit -m "Resolved merge conflicts"

# If you need to reset to last working version
git reset --hard HEAD~1
```

### Development Workflow
1. **Edit Source**: Modify files in `src/` directory
2. **Build**: Run `make` to compile
3. **Test**: Upload to robot and test functionality
4. **Iterate**: Make adjustments and repeat

## Competition Strategy

### Autonomous Phase
- **AWP Focus**: Prioritize Autonomous Win Points for early game advantage
- **Positioning**: Use precise odometry for consistent autonomous performance
- **Adaptive Selection**: Multiple autonomous routines for different field positions

## Troubleshooting

### Common Issues
- **Sensor Calibration**: Ensure IMU and tracking wheels are properly calibrated
- **Motor Configuration**: Verify motor ports and gear ratios in device setup
- **Autonomous Selection**: Check rotation sensor connection for autonomous routine selection

### Debug Features
- **LCD Display**: Real-time robot pose and sensor data
- **Console Output**: Detailed logging for troubleshooting
- **Sensor Testing**: Built-in sensor validation routines

## License

This project is developed for VEX Robotics Competition use. All code and designs are property of 4478N and Masuk Robotics.

## Support

For questions or support regarding this robot code:
- Check the [PROS Documentation](https://pros.cs.purdue.edu/)
- Review [LemLib Documentation](https://lemlib.com/)
- Contact Team 4478N for team-specific assistance

---

**Good luck in competition! **
