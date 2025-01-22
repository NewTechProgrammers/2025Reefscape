// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.SwerveJoystickCmd;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true); // Setting to ignore warning with "There is no joystick"
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
          swerveSubsystem,
          () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
          () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
          () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
          () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    
    configureBindings(); // Configure the trigger bindings
  }

  private void configureBindings() {  }

  public Command getAutonomousCommand() {
    return null;
  }
}
