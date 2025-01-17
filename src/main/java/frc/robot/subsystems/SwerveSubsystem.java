package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import java.io.*; 

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    public boolean flagZero = false;

    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        
        
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,

        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,

        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        
        
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,

        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);


    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,

        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);




    
    public AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI); // = new AHRS(SPI.Port.kMXP);

    
    //public CANSparkMax rotate = new CANSparkMax(23, MotorType.kBrushless);

    // public final AHRS gyro = new AHRS(SPI.Port.kMXP);

    double ChangeX, ChangeY, ChangeZ = 0;

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, gyro.getRotation2d(),
    new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
    }, //HERE IS THY STARTING ORIENTATION AND POSITION IT SHALL'ETH BE CHANGED VIA A VARIABLE
     new Pose2d(0, 0, new Rotation2d()));


    public SwerveSubsystem() {
        new Thread(() -> {
            try {

                Thread.sleep(1000);
                zeroHeading();

                
            } catch (Exception e) {  }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
        //gyro.setInverted(true);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle()*-1, 360);
    }

    // public void shoot(){
    //     shoot1.set(-0.95);
    //     shoot2.set(-0.25);
    //     shoot3.set(-0.95);
    // }

    // public void shootOff(){
    //     shoot1.set(0);
    //     shoot2.set(0);
    //     shoot3.set(0);
    // }

    // public void rotateP(){
    //     rotate.set(-0.15);
    // }

    // public void rotateN(){
    //     rotate.set(0.15);


    // }

    // public void rotateZ(){
    //     rotate.set(0);
    // }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(getRotation2d(),new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()
        },pose);
    }


    public void tx(){
        System.out.println("YYYY!");
    }

    @Override 
    public void periodic() {
        var gyroAngle = gyro.getRotation2d();
        

        // Update the pose
        odometry.update(gyroAngle,
        new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()
        });

        ChangeX += gyro.getRawGyroX();
        ChangeY += gyro.getRawGyroY();
        ChangeZ += gyro.getRawGyroZ();

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location meters", getPose().getTranslation().toString());
        //SmartDashboard.putString("Robot Location", Position);
        SmartDashboard.putNumber("FrontLeft Steer:", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FrontRight Steer:", frontRight.getTurningPosition());
        SmartDashboard.putNumber("BackLeft Steer:", backLeft.getTurningPosition());
        SmartDashboard.putNumber("BackRight Steer:", backRight.getTurningPosition());

        SmartDashboard.putNumber("CANFrontLeft:", frontLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANFrontRight:", frontRight.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANBackLeft:", backLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("CANBackRight:", backRight.getAbsoluteEncoderPos());

        SmartDashboard.putBoolean("FLAG FL: ", frontLeft.zeroModuleFlag);
        SmartDashboard.putBoolean("FLAG FR: ", frontRight.zeroModuleFlag);
        SmartDashboard.putBoolean("FLAG BL: ", backLeft.zeroModuleFlag);
        SmartDashboard.putBoolean("FLAG BR: ", backRight.zeroModuleFlag);

        SmartDashboard.putNumber("Gyro angle: ", (0 - (gyro.getAngle())));
        SmartDashboard.putNumber("Gyro yaw: ", (0 - (gyro.getYaw())));
        SmartDashboard.putNumber("Gyro roll: ", (0 - (gyro.getRoll())));
        SmartDashboard.putNumber("Gyro pitch: ", (0 - (gyro.getPitch())));

        SmartDashboard.putNumber("Gyro X: ", gyro.getRawGyroX());
        SmartDashboard.putNumber("Gyro Y: ", gyro.getRawGyroY());
        SmartDashboard.putNumber("Gyro Z: ", gyro.getRawGyroZ());
        SmartDashboard.putNumber("ChangeX: ", ChangeX);
        SmartDashboard.putNumber("ChangeY: ", ChangeY);
        SmartDashboard.putNumber("ChangeZ: ", ChangeZ);
        
        
    }

    public void flagChanging(){
        flagZero = !flagZero;
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // .normalizeWheelSpeeds() 
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public void wheelZeroing()
    {
        frontLeft.zeroModuleFlagChange();
        frontRight.zeroModuleFlagChange();
        backLeft.zeroModuleFlagChange();
        backRight.zeroModuleFlagChange();
    }

}