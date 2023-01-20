// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.FRC5010.sensors.GenericGyro;

/** Add your docs here. */
public class SwerveDrivetrain extends GenericDrivetrain{

    private ChassisSpeeds chassisSpeeds;

    private GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

    private GenericGyro gyro;

    // public variables
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.76835;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;


    public static final double kFrontLeftAbsoluteOffsetRad = 2.884; 
    public static final double kFrontRightAbsoluteOffsetRad = 0;
    public static final double kBackLeftAbsoluteOffsetRad = 5.224; 
    public static final double kBackRightAbsoluteOffsetRad = 6.074;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 0.05;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = .08;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = .4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI;

    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
);

    private SwerveDriveOdometry odometry;

    private SwerveModulePosition[] modulePositions;

    public SwerveDrivetrain(Mechanism2d mechVisual, GenericSwerveModule frontLeft, GenericSwerveModule frontRight, GenericSwerveModule backLeft, GenericSwerveModule backRight, GenericGyro genericGyro) {
        super(mechVisual);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.gyro = genericGyro;

        setDrivetrainPoseEstimator(poseEstimator);



        modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.frontLeft.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.frontRight.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.backLeft.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.backRight.getAbsoluteEncoderRad())))
        };

        new Thread(() -> {
            try{
              Thread.sleep(1000);
            }catch(Exception e){}
            zeroGyroscope();
          }).start();
        //TODO Auto-generated constructor stub
    }

    @Override
    public void drive(ChassisSpeeds direction) {
        this.chassisSpeeds = direction;
        
    }

    public void zeroGyroscope() {
        gyro.setAngle(0);
      }


    public void setModuleStates(SwerveModuleState[] setDesiredStates){
        SwerveModuleState[] states = setDesiredStates;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, kPhysicalMaxSpeedMetersPerSecond);
        
        if(states[0].speedMetersPerSecond > 0.0001){
                frontLeft.setState(states[0]);
                frontRight.setState(states[1]);
                backLeft.setState(states[2]);
                backRight.setState(states[3]);
        }else{
                frontLeft.setState(new SwerveModuleState(0, new Rotation2d(45*5)));
                frontRight.setState(new SwerveModuleState(0, new Rotation2d(45*7)));
                backLeft.setState(new SwerveModuleState(0, new Rotation2d(45*3)));
                backRight.setState(new SwerveModuleState(0, new Rotation2d(45)));
        }
    }

    public void stop(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        SwerveModulePosition[] newPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(Units.radiansToDegrees(this.frontLeft.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(Units.radiansToDegrees(this.frontRight.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(Units.radiansToDegrees(this.backLeft.getAbsoluteEncoderRad()))), 
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(Units.radiansToDegrees(this.backRight.getAbsoluteEncoderRad())))
        };

        odometry.update(new Rotation2d(gyro.getAngle()), newPositions);
    }
    

}
