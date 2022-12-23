// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.GenericPose;
import frc.robot.FRC5010.GenericSwerveModule;

/** Add your docs here. */
public class SwervePose extends GenericPose {
    private final SwerveDriveOdometry poseEstimator;
    private final GenericSwerveModule frontLeft;
    private final GenericSwerveModule frontRight;
    private final GenericSwerveModule backLeft;
    private final GenericSwerveModule backRight;

    public SwervePose(GenericGyro gyro, SwerveDriveKinematics kDriveKinematics,
        GenericSwerveModule frontLeft, GenericSwerveModule frontRight, 
        GenericSwerveModule backLeft, GenericSwerveModule backRight) {
        super(gyro);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        poseEstimator = new SwerveDriveOdometry(
        kDriveKinematics, new Rotation2d(0));
    }

    @Override
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void updateVision(Pose2d robotPose, double imageCaptureTime) {
        poseEstimator.resetPosition(robotPose, robotPose.getRotation());
        // m_poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }

    @Override
    public void updatePhysics() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), 
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    @Override
    public Pose2d getCurrentPose() {
        return poseEstimator.getPoseMeters();
    }

    public void resetToPose(Pose2d pose) {
        poseEstimator.resetPosition(pose, getGyroRotation2d());
    }
}
