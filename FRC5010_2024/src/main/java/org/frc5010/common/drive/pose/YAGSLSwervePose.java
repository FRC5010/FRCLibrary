// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.gyro.GenericGyro;

public class YAGSLSwervePose extends GenericPose {
  private YAGSLSwerveDrivetrain drivetrain;

  public YAGSLSwervePose(GenericGyro gyro, YAGSLSwerveDrivetrain drivetrain) {
    super(gyro);
    this.drivetrain = drivetrain;
    field2d = drivetrain.getField2d();
  }

  @Override
  public void resetEncoders() {
    drivetrain.resetEncoders();
  }

  @Override
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Vector<N3> stdVector) {
    drivetrain.updateVisionMeasurements(robotPose, imageCaptureTime, stdVector);
  }

  @Override
  public void updateLocalMeasurements() {}

  @Override
  public Pose2d getCurrentPose() {
    return drivetrain.getPose();
  }

  @Override
  public void resetToPose(Pose2d pose) {
    drivetrain.resetOdometry(pose);
  }

  @Override
  public Rotation2d getGyroRotation2d() {
    return drivetrain.getHeading();
  }
}
