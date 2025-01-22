package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final SparkMax          driveMotor;
    private final SparkMax          steerMotor;

    private final SparkMaxConfig    driveMotorConfig;
    private final SparkMaxConfig    steerMotorConfig;

    private final CANcoder          absoluteEncoder;
    private final boolean           absoluteEncoderReversed;
    private final double            absoluteEncoderOffsetRad;

    private final PIDController     steerPidController;

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        driveMotorConfig = new SparkMaxConfig();
        steerMotorConfig = new SparkMaxConfig();

        driveMotorConfig
                .inverted(driveMotorReversed);
        steerMotorConfig
                .inverted(steerMotorReversed);

        driveMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kSteerEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kSteerEncoderRPM2RadPerSec);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steerPidController = new PIDController(ModuleConstants.kPSteer, 0, 0);
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getSteerPosition() {
        return steerMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public double getSteerVelocity() {
        return steerMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderPos() {
        return absoluteEncoder.getPosition().getValueAsDouble();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition().getValueAsDouble();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        steerMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(steerPidController.calculate(getSteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());

    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
