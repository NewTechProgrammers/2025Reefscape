package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import com.studica.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftSteerMotorPort, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftSteerEncoderReversed,
        DriveConstants.kFrontLeftAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightSteerMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightSteerEncoderReversed,
        DriveConstants.kFrontRightAbsoluteEncoderPort,
        DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftSteerMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftSteerEncoderReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightSteerMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightSteerEncoderReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightAbsoluteEncoderReversed);

    public AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
     }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FrontLeft Steer:", frontLeft.getSteerPosition());
        SmartDashboard.putNumber("FrontRight Steer:", frontRight.getSteerPosition());
        SmartDashboard.putNumber("BackLeft Steer:", backLeft.getSteerPosition());
        SmartDashboard.putNumber("BackRight Steer:", backRight.getSteerPosition());

        SmartDashboard.putNumber("CANFrontLeft:", frontLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANFrontRight:", frontRight.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANBackLeft:", backLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANBackRight:", backRight.getAbsoluteEncoderPos());

        SmartDashboard.putNumber("Gyro angle: ", (0 - (gyro.getAngle())));
        SmartDashboard.putNumber("Gyro yaw: ", (0 - (gyro.getYaw())));
        SmartDashboard.putNumber("Gyro roll: ", (0 - (gyro.getRoll())));
        SmartDashboard.putNumber("Gyro pitch: ", (0 - (gyro.getPitch())));

        SmartDashboard.putNumber("Gyro X: ", gyro.getRawGyroX());
        SmartDashboard.putNumber("Gyro Y: ", gyro.getRawGyroY());
        SmartDashboard.putNumber("Gyro Z: ", gyro.getRawGyroZ());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
