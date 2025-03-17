# FRC Team 9155 Robotics Code Documentation

This repository contains the FRC 9155 team robot code built using the [**Yet Another General Swerve Library (YAGSL)**](https://docs.yagsl.com/) library. Below, you will find instructions on initializing the project, setting up dependencies, and calibrating the robot.

## 📌 Table of Contents

- **[Project Initialization](#project-initialization)**
- **[Starting a New Project](#starting-a-new-project)**
- **[YAGSL Setup](#yagsl-setup)**
- **[Additional Resources](#additional-resources)**

## 🚀 Project Initialization

To set up this project on your local machine, follow these steps:

### Prerequisites

- Install [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- Install [Git](https://git-scm.com/)
- Install [VS Code with WPILib Extension](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/wpilib-setup-vscode.html)

### Cloning the Repository

```sh
# Clone the repository
git clone https://github.com/NewTechProgrammers/2025Reefscape.git
cd 2025Reefscape

# Open in VS Code
code .
```

### Building the Project

Ensure all dependencies are installed and then build the project:

```sh
./gradlew build
```

## 🛠️ Starting a New Project

To start a new FRC project with YAGSL:

1. Open VS Code and launch the WPILib extension.
2. Select **Create a New Project**.
3. Choose **SwerveBot** as the project template.
4. Configure the team number and package name.
5. Clone the [YAGSL Template](https://github.com/BobTrajectory/YAGSL) into your project:
   
   ```sh
   git clone https://github.com/BobTrajectory/YAGSL.git
   ```
6. Copy necessary files and integrate them with your project.
7. Modify `build.gradle` to include YAGSL dependencies.

## ⚙️ YAGSL Setup

- Ensure you have the correct dependencies in `build.gradle`:
  
  ```gradle
  dependencies {
      implementation 'com.pathplanner:PathPlannerLib:2025.0.0'
      implementation 'org.frcteam2910:swerve-lib:2025.1.0'
  }
  ```
- Define your swerve modules properly in your configuration file.
- Configure motor IDs, encoder offsets, and PID values based on your robot's requirements.

## 📚 Additional Resources

- [**YAGSL Lib API**](https://broncbotz3481.github.io/YAGSL-Lib/docs/)
- [**WPILib Lib API**](https://github.wpilib.org/allwpilib/docs/release/java/index.html)
- [**FRC GitHub Templates**](https://github.com/wpilibsuite)

For any issues, feel free to open an issue on this repository or consult the FRC and YAGSL communities!

