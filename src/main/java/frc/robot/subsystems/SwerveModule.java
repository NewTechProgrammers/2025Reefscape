package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private SparkMaxConfig driveMotorConfig;
    private SparkMaxConfig turningMotorConfig;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public boolean zeroModuleFlag = false;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotorConfig = new SparkMaxConfig();
        turningMotorConfig = new SparkMaxConfig();

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = driveMotor.getEncoder();

        driveMotorConfig
            .inverted(driveMotorReversed);
        driveMotorConfig.encoder
            .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningMotorConfig
            .inverted(turningMotorReversed);
        turningMotorConfig.encoder
            .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveMotor.configure(driveMotorConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        System.out.println("Turn conv factor pos: " + ModuleConstants.kTurningEncoderRot2Rad);
        System.out.println("Turn conv factor vel: " + ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning,
                ModuleConstants.kDTurning);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        resetEncoders();
        
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {

        double angle = absoluteEncoder.getPosition().getValueAsDouble();

        angle *= 2.0 * Math.PI;

        angle -= absoluteEncoderOffsetRad;

        System.out.println("ANGLE: " + String.valueOf(angle) + " ABSOLUTE POS: "
                + absoluteEncoder.getPosition().getValueAsDouble());
        System.out.println("ANGLE return: " + angle * (absoluteEncoderReversed ? -1.0 : 1.0));

        if (absoluteEncoderReversed) {
            return angle * -1;
        } else {
            return angle * 1;
        }

    }

    public double getAbsoluteEncoderPos() {
        return absoluteEncoder.getPosition().getValueAsDouble();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        System.out.println("TURNING ENCODER: " + String.valueOf(turningEncoder.getPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // turningMotor.set(turningPidController.setReference())
        if (zeroModuleFlag == false) {
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        }
        else if(zeroModuleFlag == true){
            turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));
        }

    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void wheelZero() {
        turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));
    }

    public void zeroModuleFlagChange(){
        zeroModuleFlag = !zeroModuleFlag;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }
}