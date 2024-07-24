// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SwervePose extends GenericPose {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveModulePosition[] modulePositions;
  private SwerveDrivetrain drivetrain;

  public SwervePose(
      GenericGyro gyro, SwerveDriveKinematics kDriveKinematics, SwerveDrivetrain drivetrain) {
    super(gyro);

    this.drivetrain = drivetrain;
    modulePositions = drivetrain.getModulePositions();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kDriveKinematics, new Rotation2d(0), modulePositions, new Pose2d());
    // poseEstimator.
  }

  public SwervePose(
      GenericGyro gyro, SwerveDrivetrain drivetrain, SwerveDrivePoseEstimator poseEstimator) {
    super(gyro);
    this.drivetrain = drivetrain;
    modulePositions = drivetrain.getModulePositions();
    this.poseEstimator = poseEstimator;
  }

  @Override
  public void resetEncoders() {
    drivetrain.resetEncoders();
  }

  @Override
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Vector<N3> stdVector) {
    poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
  }

  @Override
  public void updateLocalMeasurements() {
    poseEstimator.update(getGyroRotation2d(), drivetrain.getModulePositions());
  }

  @Override
  public Pose2d getCurrentPose() {
    // System.out.println(poseEstimator.getEstimatedPosition());
    return poseEstimator.getEstimatedPosition();
  }

  public void resetToPose(Pose2d pose) {
    gyro.reset();
    poseEstimator.resetPosition(pose.getRotation(), modulePositions, pose);
    gyro.setAngle(pose.getRotation().getDegrees());
  }
}
