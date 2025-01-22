// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters =         Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio =         1 / 6.12;
      public static final double kSteerMotorGearRatio =         1 / 21.4285714286;
      public static final double kDriveEncoderRot2Meter =       kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kSteerEncoderRot2Rad =         kSteerMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kSteerEncoderRPM2RadPerSec =   kSteerEncoderRot2Rad / 60;
      public static final double kPSteer =                      0.38;
  }

  public static final class DriveConstants {
      public static final double kTrackWidth = Units.inchesToMeters(22.2); // Distance between centers of front and back wheels
      public static final double kWheelBase = Units.inchesToMeters(22.2); // Distance between centers of right and left wheels

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final double kPhysicalMaxSpeedMetersPerSecond = 1.0;

      public static final int kFrontRightDriveMotorPort = 3;
      public static final int kFrontRightSteerMotorPort = 2;

      public static final int kFrontLeftDriveMotorPort = 4;
      public static final int kFrontLeftSteerMotorPort = 5;

      public static final int kBackRightDriveMotorPort = 7;
      public static final int kBackRightSteerMotorPort = 6;

      public static final int kBackLeftDriveMotorPort = 9;
      public static final int kBackLeftSteerMotorPort = 8;

      // CANcoders / AbsoluteEncoders
      public static final int kFrontLeftAbsoluteEncoderPort = 13;
      public static final int kFrontRightAbsoluteEncoderPort = 12;
      public static final int kBackLeftAbsoluteEncoderPort = 11;
      public static final int kBackRightAbsoluteEncoderPort = 10;

      public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
      public static final boolean kBackLeftAbsoluteEncoderReversed = false;
      public static final boolean kFrontRightAbsoluteEncoderReversed = false;
      public static final boolean kBackRightAbsoluteEncoderReversed = false;

      // Default encoders
      public static final boolean kFrontLeftSteerEncoderReversed = false;
      public static final boolean kBackLeftSteerEncoderReversed = false;
      public static final boolean kFrontRightSteerEncoderReversed = false;
      public static final boolean kBackRightSteerEncoderReversed = false;

      public static final boolean kFrontLeftDriveEncoderReversed = true;
      public static final boolean kBackLeftDriveEncoderReversed = true;
      public static final boolean kFrontRightDriveEncoderReversed = true;
      public static final boolean kBackRightDriveEncoderReversed = true;

      // Encoder values (absolute + default) should be increasing while ccw run (from the top view)
      public static final double kFrontLeftAbsoluteEncoderOffsetRad = 0;
      public static final double kBackLeftAbsoluteEncoderOffsetRad = 0;
      public static final double kFrontRightAbsoluteEncoderOffsetRad = 0;
      public static final double kBackRightAbsoluteEncoderOffsetRad = 0;

      public static final double kPhysicalMaxAccelerationMetersPerSecondSquared = 2;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecondSquared = 2 * 2 * Math.PI;
      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
              kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

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
