// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.drive.pose.DrivetrainPoseEstimator;

public class DriverDisplaySubsystem extends SubsystemBase {
  /** Creates a new DriverDisplaySubsystem. */
  private DrivetrainPoseEstimator poseEstimator;

  private String idToStation;

  public DriverDisplaySubsystem(DrivetrainPoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (poseEstimator.getClosestTagToRobot()) {
      case 1:
        idToStation = "Red 1";
        break;
      case 2:
        idToStation = "Red 2";
        break;
      case 3:
        idToStation = "Red 3";
        break;
      case 4:
        idToStation = "Blue Loading";
        break;
      case 5:
        idToStation = "Red Loading";
        break;
      case 6:
        idToStation = "Blue 1";
        break;
      case 7:
        idToStation = "Blue 2";
        break;
      case 8:
        idToStation = "Blue 3";
        break;
      default:
        idToStation = "Room Tag / Unknown Position";
    }
    SmartDashboard.putString("Station", idToStation);
    SmartDashboard.putNumber("Closest Tag To Robot", poseEstimator.getClosestTagToRobot());
  }
}
