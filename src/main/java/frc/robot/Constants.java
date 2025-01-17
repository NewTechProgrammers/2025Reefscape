package frc.robot;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants {
        // Colson wheel standard diameter in meters
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        // Distance between front and back wheels
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

        // Colson wheel standard radius in meters
        public static final double kDriveMotorGearRatio = 1/6.12; // 6.12
        public static final double kTurningMotorGearRatio = 1/21.4285714286; // 150/7

        // public static final double kDriveMotorGearRatio = 6.12;
        // public static final double kTurningMotorGearRatio = 150/7;

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        public static final double kDriveEncoderPositionConversionFactor = (2 * Math.PI * kWheelRadiusMeters) / kDriveMotorGearRatio;
        public static final double kDriveEncoderVelocityConversionFactor = (2 * Math.PI * kWheelRadiusMeters) / (kDriveMotorGearRatio * 60);
        
        // // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // // This is the the angle through an entire rotation (2 * pi) divided by the
        // // encoder resolution.
        public static final double kSteerEncoderPositionConversionFactor = (2 * Math.PI )/ kTurningMotorGearRatio;
        public static final double kSteerEncoderVelocityConversionFactor = (2 * Math.PI )/ (kTurningMotorGearRatio * 60);

        // Default variables from guide
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;   
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.13; // 0.32
        public static final double kITurning = 0.0; // 0.6
        public static final double kDTurning = 0.000;

    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(23);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23);
        // Distance between front and back wheels
        // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //         new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontLeft
        //         new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // frontRight
        //         new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // backLeft
        //         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // backRight

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // frontLeft // - +
        // new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontRight
        // new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // backLeft // + - backleft
        // new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); // backRight // + - // frontleft

        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontLeft // - +
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // frontRight
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // backLeft // + - backleft
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // backRight // + - // frontleft

        /*public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontLeft
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // frontRight
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // backLeft
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // backRight
        */
        // 7, 6 (left CAN12) - 5, 4 (right CAN11) front
        // 8, 9 (left CAN13) - 2, 3 (right CAN10) back

        public static final int kFrontLeftDriveMotorPort = 7; // 4 // 7
        public static final int kBackLeftDriveMotorPort = 3; //  2 // 
        public static final int kFrontRightDriveMotorPort = 9; // 7
        public static final int kBackRightDriveMotorPort = 4; // 9 //

        public static final int kFrontLeftTurningMotorPort = 6; // 5
        public static final int kBackLeftTurningMotorPort = 2; // 3 //
        public static final int kFrontRightTurningMotorPort = 8; // 6
        public static final int kBackRightTurningMotorPort = 5; // 8

        // CANcoders / AbsoluteEncoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10; // 11 // 
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11; // 10
        public static final int kBackRightDriveAbsoluteEncoderPort = 13;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // Default encoders
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // Encoder values (absolute + default) should be increasing while ccw run (from the top view)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; // -2.88081592 rad - -0.45849609375 percent;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; // // 1.88986433 rad - 0.30078125 percent
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; // -1.49869923 rad - -0.238525390625 percent
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0; // -1.19497103 rad - -0.190185546875 percent

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8768; //5
        public static final double kPhysicalMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // 2 * 2 * Math.PI
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecondSquared = 2 * 2 * Math.PI; // 2 * 2 * Math.PI
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5; // /4 // drive speed
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 1.2; // /4
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; // 3
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; // 3

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kPhysicalMaxAngularSpeedRadiansPerSecond, kPhysicalMaxAngularSpeedRadiansPerSecondSquared);
}

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kSupportControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}
