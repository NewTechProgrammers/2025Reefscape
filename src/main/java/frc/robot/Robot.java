// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    // private XboxController xbox_stick = new XboxController(0);

    
    // /* Drive For Testing */
    // // Front Left
    // private CANSparkMax m1Drive = new CANSparkMax(8, MotorType.kBrushless);
    // private CANSparkMax m1Steer = new CANSparkMax(9, MotorType.kBrushless);
    // private CANcoder c1 = new CANcoder(13);

    // // Front Right
    // private CANSparkMax m2Drive = new CANSparkMax(7, MotorType.kBrushless);
    // private CANSparkMax m2Steer = new CANSparkMax(6, MotorType.kBrushless);
    // private CANcoder c2 = new CANcoder(12);

    
    
    // // Back Left
    // private CANSparkMax m3Drive = new CANSparkMax(2, MotorType.kBrushless);
    // private CANSparkMax m3Steer = new CANSparkMax(3, MotorType.kBrushless);
    // private CANcoder c3 = new CANcoder(10);
    
    // // Back Right
    // private CANSparkMax m4Drive = new CANSparkMax(4, MotorType.kBrushless);
    // private CANSparkMax m4Steer = new CANSparkMax(5, MotorType.kBrushless);
    // private CANcoder c4 = new CANcoder(11);
    

    // /* Shooting */
    // // private CANSparkMax m1 = new CANSparkMax(14, MotorType.kBrushless);
    // // private CANSparkMax m2 = new CANSparkMax(15, MotorType.kBrushless);
    // // private CANSparkMax m3 = new CANSparkMax(16, MotorType.kBrushless);
    // // private CANSparkMax m4 = new CANSparkMax(17, MotorType.kBrushless);
    
    

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // m1Drive.setInverted(true);
        // m1Steer.setInverted(true);
        // m1Drive.burnFlash();
        // m1Steer.burnFlash();

        // m2Drive.setInverted(true);
        // m2Steer.setInverted(false);
        // m2Drive.burnFlash();
        // m2Steer.burnFlash();

        // m3Drive.setInverted(true);
        // m3Steer.setInverted(true);
        // m3Drive.burnFlash();
        // m3Steer.burnFlash();

        // m4Drive.setInverted(true);
        // m4Steer.setInverted(true);
        // m4Drive.burnFlash();
        // m4Steer.burnFlash();
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        
        m_robotContainer = new RobotContainer();
        
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // SmartDashboard.putNumber("Front Left Drive:", m1Drive.getEncoder().getPosition());
        // SmartDashboard.putNumber("Front Left Steer", m1Steer.getEncoder().getPosition());
        // SmartDashboard.putNumber("Front Left can def: ", c1.getPosition().getValueAsDouble()); 
        // SmartDashboard.putNumber("Front Left can absolute: ", c1.getAbsolutePosition().getValueAsDouble()); 


        // SmartDashboard.putNumber("Front Right Drive:", m2Drive.getEncoder().getPosition());
        // SmartDashboard.putNumber("Front Right Steer", m2Steer.getEncoder().getPosition() * -1);
        // SmartDashboard.putNumber("Front Right can def: ", c2.getPosition().getValueAsDouble()); // 0 - 0.5 -0.5 - 0
        // SmartDashboard.putNumber("Front Right can absolute: ", c2.getAbsolutePosition().getValueAsDouble()); // dobrze ccw 0-1


        // SmartDashboard.putNumber("Back Left Drive:", m3Drive.getEncoder().getPosition());
        // SmartDashboard.putNumber("Back Left Steer", m3Steer.getEncoder().getPosition());
        // SmartDashboard.putNumber("Back Left can def: ", c3.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Back Left can absolute: ", c3.getAbsolutePosition().getValueAsDouble());


        // SmartDashboard.putNumber("Back Right Drive:", m4Drive.getEncoder().getPosition());
        // SmartDashboard.putNumber("Back Right Steer", m4Steer.getEncoder().getPosition());
        // SmartDashboard.putNumber("Back Right can def: ", c4.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Back Right can absolute: ", c4.getAbsolutePosition().getValueAsDouble());


    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic()
    {
    //    if(xbox_stick.getAButtonPressed()){
    //     m1Drive.getEncoder().setPosition(0);
    //     m2Drive.getEncoder().setPosition(0);
    //     m3Drive.getEncoder().setPosition(0);
    //     m4Drive.getEncoder().setPosition(0);

    //     m1Steer.getEncoder().setPosition(0);
    //     m2Steer.getEncoder().setPosition(0);
    //     m3Steer.getEncoder().setPosition(0);
    //     m4Steer.getEncoder().setPosition(0);
    //    }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}