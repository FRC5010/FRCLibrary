// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.pose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.FRC5010.drive.GenericSwerveModule;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SwervePose extends GenericPose {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final GenericSwerveModule frontLeft;
    private final GenericSwerveModule frontRight;
    private final GenericSwerveModule backLeft;
    private final GenericSwerveModule backRight;
    private final SwerveModulePosition[] modulePositions;

    public SwervePose(GenericGyro gyro, SwerveDriveKinematics kDriveKinematics,
        GenericSwerveModule frontLeft, GenericSwerveModule frontRight, 
        GenericSwerveModule backLeft, GenericSwerveModule backRight) {
        super(gyro);

         modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(frontLeft.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(0, new Rotation2d(frontRight.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(0, new Rotation2d(backLeft.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(0, new Rotation2d(backRight.getAbsoluteEncoderRad()))
          };
        
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, new Rotation2d(0), modulePositions, new Pose2d());
        // poseEstimator.
    }

    @Override
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void updateVisionMeasurements(Pose2d robotPose, double imageCaptureTime) {
        //System.out.println("pose: " + robotPose + " image: " + imageCaptureTime);
        // //if (imageCaptureTime < 0.05) {
        //     poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
        // }
        poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }

    @Override
    public void updateLocalMeasurements() {
        SwerveModulePosition[] newPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getAbsoluteEncoderRad())), 
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getAbsoluteEncoderRad()))
          };

        poseEstimator.update(getGyroRotation2d(), newPositions);
    }

    @Override
    public Pose2d getCurrentPose() {
        //System.out.println(poseEstimator.getEstimatedPosition());
        return poseEstimator.getEstimatedPosition();
    }

    public void resetToPose(Pose2d pose) {
        poseEstimator.resetPosition(pose.getRotation(), modulePositions,pose);
    }
}
