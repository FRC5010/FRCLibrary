// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.frc5010.common.drive.pose.DrivetrainPoseEstimator;

/** Add your docs here. */
public class VisionLimeLightSim extends VisionSystem {

  protected DrivetrainPoseEstimator drivetrainPoseEstimator = null;

  public VisionLimeLightSim(
      String name, int colIndex, AprilTagFieldLayout fieldLayout, Transform3d cameraToRobot) {
    super(name, colIndex, fieldLayout);
    this.cameraToRobot = cameraToRobot;
  }

  public void setDrivetrainPoseEstimator(DrivetrainPoseEstimator drivetrainPoseEstimator) {
    this.drivetrainPoseEstimator = drivetrainPoseEstimator;
  }

  @Override
  public void setPipeline(int pipeline) {}

  @Override
  public void setLight(boolean on) {}

  @Override
  public boolean isLightOn() {
    return false;
  }

  @Override
  public void flashLight() {}

  @Override
  public void setSnapshotMode(int snapVal) {}

  @Override
  public void update() {}
}
